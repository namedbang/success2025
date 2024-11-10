/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-08 10:06:09
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-10 21:14:30
 * @FilePath: /success2025/src/process/process_opencv.cpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */

#include "process_opencv.hpp"
#include "../cuda/inRange_gpu.cuh"

#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>

cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

PROCESS_state process_opencv_cuda::processing()
{
    // if (!this->Picture_p->preImage.empty())
    //     return PROCESUNSUCCESS;
    cv::cuda::GpuMat G_image;
    cv::cuda::GpuMat HSV;
    cv::cuda::GpuMat inRange(cv::Size(this->Picture_p->preImage.cols, this->Picture_p->preImage.rows), CV_8UC1, cv::Scalar(255));
    cv::cuda::GpuMat filter_open;
    cv::cuda::GpuMat filter_close;
    G_image.upload(this->Picture_p->preImage);
    cv::cuda::cvtColor(G_image, HSV, cv::COLOR_BGR2HSV);
    inRange_gpu(HSV, *this->lowerFilter, *this->higherFilter, inRange);
    cv::Ptr<cv::cuda::Filter> morph_filter_open = cv::cuda::createMorphologyFilter(cv::MORPH_CLOSE, inRange.type(), close_kernel);
    cv::Ptr<cv::cuda::Filter> morph_filter_close = cv::cuda::createMorphologyFilter(cv::MORPH_OPEN, inRange.type(), open_kernel);
    morph_filter_open->apply(inRange, filter_open);
    morph_filter_close->apply(filter_open, filter_close);
    inRange.download(this->Picture_p->endImage);
    return PROCESSUCCESS;
}
