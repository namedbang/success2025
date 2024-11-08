/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-08 10:06:09
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-08 18:35:35
 * @FilePath: /success2025/src/process/process_opencv.cpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */

#include "process_opencv.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>

PROCESS_state process_opencv_cuda::processing()
{
    // if (!this->Picture_p->preImage.empty())
    //     return PROCESUNSUCCESS;
    cv::cuda::GpuMat G_image;
    G_image.upload(this->Picture_p->preImage);
    cv::cuda::GpuMat gray;
    cv::cuda::cvtColor(G_image, gray, cv::COLOR_BGR2GRAY);
    gray.download(this->Picture_p->endImage);
    return PROCESSUCCESS;
}
