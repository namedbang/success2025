/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-08 10:06:09
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-10 23:23:16
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
#include <opencv2/cudaobjdetect.hpp>

cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(20, 20));
cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(20, 20));
std::vector<cv::Vec4i> hierarchy;

std::vector<std::vector<cv::Point>> contours;
PROCESS_state process_opencv_cuda::processing()
{
    // if (!this->Picture_p->preImage.empty())
    //     return PROCESUNSUCCESS;
    cv::Mat findContour;
    cv::cuda::GpuMat G_image;
    cv::cuda::GpuMat HSV;
    cv::cuda::GpuMat inRange(cv::Size(this->Picture_p->preImage.cols, this->Picture_p->preImage.rows), CV_8UC1, cv::Scalar(255));
    cv::cuda::GpuMat filter_open;
    cv::cuda::GpuMat filter_close;
    G_image.upload(this->Picture_p->preImage);
    cv::cuda::cvtColor(G_image, HSV, cv::COLOR_BGR2HSV);
    inRange_gpu(HSV, *this->lowerFilter, *this->higherFilter, inRange);
    // cv::Ptr<cv::cuda::Filter> morph_filter_open = cv::cuda::createMorphologyFilter(cv::MORPH_CLOSE, inRange.type(), open_kernel);
    // cv::Ptr<cv::cuda::Filter> morph_filter_close = cv::cuda::createMorphologyFilter(cv::MORPH_OPEN, inRange.type(), close_kernel);
    // morph_filter_open->apply(inRange, filter_open);
    // morph_filter_close->apply(inRange, filter_close);
    inRange.download(this->Picture_p->endImage);
    cv::findContours(this->Picture_p->endImage, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
    return PROCESSUCCESS;
}
