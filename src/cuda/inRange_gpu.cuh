/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-10 16:22:17
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2025-01-09 20:59:56
 * @FilePath: /success2025/src/cuda/inRange_gpu.cuh
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
#ifndef __INRANGE_H__
#define __INRANGE_H__

#include "../yolo/yolov8.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <stdio.h>

// extern "C" void inRange_gpu(cv::cuda::GpuMat &src, cv::Scalar &lowerb, cv::Scalar &upperb,
//                             cv::cuda::GpuMat &dst);
extern "C" void inRange_gpu(cv::cuda::GpuMat &src, cv::Scalar &lowerb, cv::Scalar &upperb,
                            cv::cuda::GpuMat &dst, std::vector<Object> rect, cv::cuda::Stream &stream);
#endif