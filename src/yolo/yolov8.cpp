//
// Created by  triple-Mu     on 24-1-2023.
// Modified by Q-engineering on  6-3-2024
//

#include "yolov8.hpp"
#include <cuda_runtime_api.h>
#include <cuda.h>

//----------------------------------------------------------------------------------------
// using namespace det;
//----------------------------------------------------------------------------------------
// const char *class_names[] = {
//     "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
//     "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
//     "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
//     "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
//     "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
//     "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
//     "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
//     "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
//     "hair drier", "toothbrush"};
const char *class_names[] = {"car_red", "car_blue", "car_unknow", "watcher_red", "watcher_blue", "watcher_unknow", "armor_red", "armor_blue", "armor_grey"};

//----------------------------------------------------------------------------------------
YOLOv8::YOLOv8(const std::string &engine_file_path)
{
    std::ifstream file(engine_file_path, std::ios::binary);
    assert(file.good());
    file.seekg(0, std::ios::end);
    auto size = file.tellg();
    file.seekg(0, std::ios::beg);
    char *trtModelStream = new char[size];
    assert(trtModelStream);
    file.read(trtModelStream, size);
    file.close();
    initLibNvInferPlugins(&this->gLogger, "");
    this->runtime = nvinfer1::createInferRuntime(this->gLogger);
    assert(this->runtime != nullptr);

    this->engine = this->runtime->deserializeCudaEngine(trtModelStream, size);
    assert(this->engine != nullptr);
    delete[] trtModelStream;
    this->context = this->engine->createExecutionContext();

    assert(this->context != nullptr);
    cudaStreamCreate(&this->stream);
    this->num_bindings = this->engine->getNbBindings();

    for (int i = 0; i < this->num_bindings; ++i)
    {
        Binding binding;
        nvinfer1::Dims dims;
        nvinfer1::DataType dtype = this->engine->getBindingDataType(i);
        std::string name = this->engine->getBindingName(i);
        binding.name = name;
        binding.dsize = type_to_size(dtype);

        bool IsInput = engine->bindingIsInput(i);
        if (IsInput)
        {
            this->num_inputs += 1;
            dims = this->engine->getProfileDimensions(i, 0, nvinfer1::OptProfileSelector::kMAX);
            binding.size = get_size_by_dims(dims);
            binding.dims = dims;
            this->input_bindings.push_back(binding);
            // set max opt shape
            this->context->setBindingDimensions(i, dims);
        }
        else
        {
            dims = this->context->getBindingDimensions(i);
            binding.size = get_size_by_dims(dims);
            binding.dims = dims;
            this->output_bindings.push_back(binding);
            this->num_outputs += 1;
        }
    }
}
//----------------------------------------------------------------------------------------
YOLOv8::~YOLOv8()
{
    this->context->destroy();
    this->engine->destroy();
    this->runtime->destroy();
    cudaStreamDestroy(this->stream);
    for (auto &ptr : this->device_ptrs)
    {
        CHECK(cudaFree(ptr));
    }

    for (auto &ptr : this->host_ptrs)
    {
        CHECK(cudaFreeHost(ptr));
    }
}
//----------------------------------------------------------------------------------------
void YOLOv8::MakePipe(bool warmup)
{
#ifndef CUDART_VERSION
#error CUDART_VERSION Undefined!
#endif

    for (auto &bindings : this->input_bindings)
    {
        void *d_ptr;
#if (CUDART_VERSION < 11000)
        CHECK(cudaMalloc(&d_ptr, bindings.size * bindings.dsize));
#else
        CHECK(cudaMallocAsync(&d_ptr, bindings.size * bindings.dsize, this->stream));
#endif
        this->device_ptrs.push_back(d_ptr);
    }

    for (auto &bindings : this->output_bindings)
    {
        void *d_ptr, *h_ptr;
        size_t size = bindings.size * bindings.dsize;
#if (CUDART_VERSION < 11000)
        CHECK(cudaMalloc(&d_ptr, bindings.size * bindings.dsize));
#else
        CHECK(cudaMallocAsync(&d_ptr, bindings.size * bindings.dsize, this->stream));
#endif
        CHECK(cudaHostAlloc(&h_ptr, size, 0));
        this->device_ptrs.push_back(d_ptr);
        this->host_ptrs.push_back(h_ptr);
    }

    if (warmup)
    {
        for (int i = 0; i < 10; i++)
        {
            for (auto &bindings : this->input_bindings)
            {
                size_t size = bindings.size * bindings.dsize;
                void *h_ptr = malloc(size);
                memset(h_ptr, 0, size);
                CHECK(cudaMemcpyAsync(this->device_ptrs[0], h_ptr, size, cudaMemcpyHostToDevice, this->stream));
                free(h_ptr);
            }
            this->Infer();
        }
    }
}
#include <opencv2/opencv.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudabgsegm.hpp>
#include <opencv2/cudawarping.hpp>

//----------------------------------------------------------------------------------------
void YOLOv8::Letterbox(const cv::Mat &image, cv::Mat &out, cv::Size &size)
{
    // const float inp_h = size.height;
    // const float inp_w = size.width;
    // float height = image.rows;
    // float width = image.cols;
    // auto start = std::chrono::high_resolution_clock::now(); ///////////////////////////
    // // 计算缩放比例
    // float r = std::min(inp_h / height, inp_w / width);
    // int padw = std::round(width * r);
    // int padh = std::round(height * r);

    // // 使用GPU存储和处理图像
    // cv::cuda::GpuMat d_image, tmp;
    // d_image.upload(image);

    // // 如果图像需要缩放，进行缩放
    // if ((int)width != padw || (int)height != padh)
    // {
    //     cv::cuda::resize(d_image, tmp, cv::Size(padw, padh));
    // }
    // else
    // {
    //     tmp = d_image.clone();
    // }

    // // 计算边界扩展所需的宽高差
    // float dw = inp_w - padw;
    // float dh = inp_h - padh;

    // dw /= 2.0f;
    // dh /= 2.0f;
    // int top = int(std::round(dh - 0.1f));
    // int bottom = int(std::round(dh + 0.1f));
    // int left = int(std::round(dw - 0.1f));
    // int right = int(std::round(dw + 0.1f));

    // // 在GPU上进行边界扩展
    // cv::cuda::GpuMat tmp_with_border;
    // cv::cuda::copyMakeBorder(tmp, tmp_with_border, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    // cv::Mat tmp_with_border_cpu(tmp_with_border.size(), tmp_with_border.type());
    // tmp_with_border.download(tmp_with_border_cpu);

    // cv::dnn::blobFromImage(tmp_with_border_cpu, out, 1 / 255.f, cv::Size(), cv::Scalar(0, 0, 0), true, false, CV_32F);
    // auto end = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // std::cout << "代码运行时间: " << duration.count() << " 毫秒\n";

    // // 设置参数
    // this->pparam.ratio = 1 / r;
    // this->pparam.dw = dw;
    // this->pparam.dh = dh;
    // this->pparam.height = height;
    // this->pparam.width = width;

    const float inp_h = size.height;
    const float inp_w = size.width;
    float height = image.rows;
    float width = image.cols;

    float r = std::min(inp_h / height, inp_w / width);
    int padw = std::round(width * r);
    int padh = std::round(height * r);

    cv::Mat tmp;
    if ((int)width != padw || (int)height != padh)
    {
        cv::resize(image, tmp, cv::Size(padw, padh));
    }
    else
    {
        tmp = image.clone();
    }

    float dw = inp_w - padw;
    float dh = inp_h - padh;

    dw /= 2.0f;
    dh /= 2.0f;
    int top = int(std::round(dh - 0.1f));
    int bottom = int(std::round(dh + 0.1f));
    int left = int(std::round(dw - 0.1f));
    int right = int(std::round(dw + 0.1f));

    cv::copyMakeBorder(tmp, tmp, top, bottom, left, right, cv::BORDER_CONSTANT, {114, 114, 114});

    cv::dnn::blobFromImage(tmp, out, 1 / 255.f, cv::Size(), cv::Scalar(0, 0, 0), true, false, CV_32F);
    this->pparam.ratio = 1 / r;
    this->pparam.dw = dw;
    this->pparam.dh = dh;
    this->pparam.height = height;
    this->pparam.width = width;
}
//----------------------------------------------------------------------------------------
void YOLOv8::CopyFromMat(const cv::Mat &image)
{
    cv::Mat nchw;
    auto &in_binding = this->input_bindings[0];
    auto width = in_binding.dims.d[3];
    auto height = in_binding.dims.d[2];
    cv::Size size{width, height};
    this->Letterbox(image, nchw, size);

    this->context->setBindingDimensions(0, nvinfer1::Dims{4, {1, 3, height, width}});

    CHECK(cudaMemcpyAsync(
        this->device_ptrs[0], nchw.ptr<float>(), nchw.total() * nchw.elemSize(), cudaMemcpyHostToDevice, this->stream));
}
//----------------------------------------------------------------------------------------
void YOLOv8::CopyFromMat(const cv::Mat &image, cv::Size &size)
{
    cv::Mat nchw;
    this->Letterbox(image, nchw, size);
    this->context->setBindingDimensions(0, nvinfer1::Dims{4, {1, 3, size.height, size.width}});
    CHECK(cudaMemcpyAsync(
        this->device_ptrs[0], nchw.ptr<float>(), nchw.total() * nchw.elemSize(), cudaMemcpyHostToDevice, this->stream));
}
//----------------------------------------------------------------------------------------
void YOLOv8::Infer()
{

    this->context->enqueueV2(this->device_ptrs.data(), this->stream, nullptr);
    for (int i = 0; i < this->num_outputs; i++)
    {
        size_t osize = this->output_bindings[i].size * this->output_bindings[i].dsize;
        CHECK(cudaMemcpyAsync(
            this->host_ptrs[i], this->device_ptrs[i + this->num_inputs], osize, cudaMemcpyDeviceToHost, this->stream));
    }
    cudaStreamSynchronize(this->stream);
}
//----------------------------------------------------------------------------------------
void YOLOv8::PostProcess(std::vector<Object> &objs, float score_thres, float iou_thres, int topk, int num_labels)
{
    objs.clear();
    auto num_channels = this->output_bindings[0].dims.d[1];
    auto num_anchors = this->output_bindings[0].dims.d[2];

    auto &dw = this->pparam.dw;
    auto &dh = this->pparam.dh;
    auto &width = this->pparam.width;
    auto &height = this->pparam.height;
    auto &ratio = this->pparam.ratio;

    std::vector<cv::Rect> bboxes;
    std::vector<float> scores;
    std::vector<int> labels;
    std::vector<int> indices;

    cv::Mat output = cv::Mat(num_channels, num_anchors, CV_32F, static_cast<float *>(this->host_ptrs[0]));
    output = output.t();
    for (int i = 0; i < num_anchors; i++)
    {
        auto row_ptr = output.row(i).ptr<float>();
        auto bboxes_ptr = row_ptr;
        auto scores_ptr = row_ptr + 4;
        auto max_s_ptr = std::max_element(scores_ptr, scores_ptr + num_labels);
        float score = *max_s_ptr;
        if (score > score_thres)
        {
            float x = *bboxes_ptr++ - dw;
            float y = *bboxes_ptr++ - dh;
            float w = *bboxes_ptr++;
            float h = *bboxes_ptr;

            float x0 = clamp((x - 0.5f * w) * ratio, 0.f, width);
            float y0 = clamp((y - 0.5f * h) * ratio, 0.f, height);
            float x1 = clamp((x + 0.5f * w) * ratio, 0.f, width);
            float y1 = clamp((y + 0.5f * h) * ratio, 0.f, height);

            int label = max_s_ptr - scores_ptr;
            cv::Rect_<float> bbox;
            bbox.x = x0;
            bbox.y = y0;
            bbox.width = x1 - x0;
            bbox.height = y1 - y0;

            bboxes.push_back(bbox);
            labels.push_back(label);
            scores.push_back(score);
        }
    }

#ifdef BATCHED_NMS
    cv::dnn::NMSBoxesBatched(bboxes, scores, labels, score_thres, iou_thres, indices);
#else
    cv::dnn::NMSBoxes(bboxes, scores, score_thres, iou_thres, indices);
#endif

    int cnt = 0;
    for (auto &i : indices)
    {
        if (cnt >= topk)
        {
            break;
        }
        Object obj;
        obj.rect = bboxes[i];
        obj.prob = scores[i];
        obj.label = labels[i];
        objs.push_back(obj);
        cnt += 1;
    }
}
//----------------------------------------------------------------------------------------
void YOLOv8::DrawObjects(cv::Mat &bgr, const std::vector<Object> &objs)
{
    char text[256];

    for (auto &obj : objs)
    {
        cv::rectangle(bgr, obj.rect, cv::Scalar(255, 255, 255));

        sprintf(text, "%s %.1f%%", class_names[obj.label], obj.prob * 100);

        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

        int x = (int)obj.rect.x;
        int y = (int)obj.rect.y - label_size.height - baseLine;

        if (y < 0)
            y = 0;
        if (y > bgr.rows)
            y = bgr.rows;
        if (x + label_size.width > bgr.cols)
            x = bgr.cols - label_size.width;

        cv::rectangle(bgr, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)), cv::Scalar(255, 255, 255), -1);

        cv::putText(bgr, text, cv::Point(x, y + label_size.height), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
    }
}
//----------------------------------------------------------------------------------------
