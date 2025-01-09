/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-10 16:21:41
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2025-01-09 22:35:29
 * @FilePath: /success2025/src/cuda/inRange_gpu.cu
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */

//---------------------inRange_gpu.cu-----start-----------
#include "./inRange_gpu.cuh"
#include <opencv2/core/cuda.hpp>
#include <cuda_runtime.h>
#include <cuda.h>
struct RectF
{
    float x;
    float y;
    float width;
    float height;
};
__global__ void maskImageOutsideRect(cv::cuda::PtrStepSzb d_image, int width, int height, RectF *rects, int num_rects)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    // 检查是否在图像范围内
    if (x < width && y < height)
    {
        bool inside_any_rect = false;

        // 遍历每个矩形框，检查当前像素是否在任意一个矩形框内
        for (int i = 0; i < num_rects; i++)
        {
            // 获取矩形框的 x, y, width, height
            int rect_x_min = static_cast<int>(rects[i].x);
            int rect_y_min = static_cast<int>(rects[i].y);
            int rect_x_max = static_cast<int>(rects[i].x + rects[i].width);
            int rect_y_max = static_cast<int>(rects[i].y + rects[i].height);

            // 如果像素在当前矩形框内
            if (x >= rect_x_min && x < rect_x_max && y >= rect_y_min && y < rect_y_max)
            {
                inside_any_rect = true;
                break; // 找到一个矩形框包含该像素，跳出循环
            }
        }

        // 如果像素不在任何矩形框内，将其设置为黑色
        if (!inside_any_rect)
        {
            d_image(y, x) = 0; // 设置该像素为黑色 (对于 3 通道图像，uchar3 的每个通道都为 0)
        }
    }
}

__global__ void inRange_kernel(const cv::cuda::PtrStepSz<uchar3> src, cv::cuda::PtrStepSzb dst,
                               int lbc0, int ubc0, int lbc1, int ubc1, int lbc2, int ubc2)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x >= src.cols || y >= src.rows)
        return;

    uchar3 v = src(y, x);
    if (v.x >= lbc0 && v.x <= ubc0 && v.y >= lbc1 && v.y <= ubc1 && v.z >= lbc2 && v.z <= ubc2)
        dst(y, x) = 255;
    else
        dst(y, x) = 0;
}

// void inRange_gpu(cv::cuda::GpuMat &src, cv::Scalar &lowerb, cv::Scalar &upperb,
//                  cv::cuda::GpuMat &dst)
// {
//     const int m = 32;
//     int numRows = src.rows, numCols = src.cols; // 行，列
//     if (numRows == 0 || numCols == 0)
//         return;
//     // Attention! Cols Vs. Rows are reversed
//     const dim3 gridSize(ceil((float)numCols / m), ceil((float)numRows / m), 1);
//     const dim3 blockSize(m, m, 1);

//     inRange_kernel<<<gridSize, blockSize>>>(src, dst, lowerb[0], upperb[0], lowerb[1], upperb[1],
//                                             lowerb[2], upperb[2]);
// }

void inRange_gpu(cv::cuda::GpuMat &src, cv::Scalar &lowerb, cv::Scalar &upperb,
                 cv::cuda::GpuMat &dst, std::vector<Object> rect, cv::cuda::Stream &stream)
{
    const int m = 32;
    int numRows = src.rows, numCols = src.cols; // 行，列
    if (numRows == 0 || numCols == 0)
        return;
    // Attention! Cols Vs. Rows are reversed
    const dim3 gridSize(ceil((float)numCols / m), ceil((float)numRows / m), 1);
    const dim3 blockSize(m, m, 1);

    cudaStream_t cuStream = reinterpret_cast<cudaStream_t>(stream.cudaPtr());
    // 异步执行内核，传入CUDA流
    // inRange_kernel<<<gridSize, blockSize, 0, cuStream>>>(src, dst, lowerb[0], upperb[0], lowerb[1], upperb[1],
    //                                                      lowerb[2], upperb[2]);
    inRange_kernel<<<gridSize, blockSize>>>(src, dst, lowerb[0], upperb[0], lowerb[1], upperb[1],
                                            lowerb[2], upperb[2]);
    std::vector<cv::Rect_<float>> rect_armor;
    for (uint16_t i = 0; i < rect.size(); i++)
    {
        if (rect[i].label == 7 || rect[i].label == 8 || rect[i].label == 6) // TODO换
            rect_armor.push_back(rect[i].rect);
    }
    std::vector<RectF> rects;
    rects.reserve(rect_armor.size());
    for (const auto &rect : rect_armor)
    {
        rects.push_back({rect.x, rect.y, rect.width, rect.height});
    }
    RectF *d_rects;
    size_t size = rects.size() * sizeof(RectF);
    cudaMalloc(&d_rects, size);
    cudaMemcpy(d_rects, rects.data(), size, cudaMemcpyHostToDevice);
    int temp = rect_armor.size();
    maskImageOutsideRect<<<gridSize, blockSize>>>(dst, numCols, numRows, d_rects, temp);
}

//---------------------inRange_gpu.cu-----end-----------
