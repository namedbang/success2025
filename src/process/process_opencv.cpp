/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-08 10:06:09
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-12 21:04:23
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
cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

std::vector<cv::Vec4i> hierarchy;
std::vector<std::vector<cv::Point>> contours;
cv::Rect boundRect;

PROCESS_state process_opencv_cuda::processing()
{
    // if (!this->Picture_p->preImage.empty())
    //     return PROCESUNSUCCESS;
    cv::Rect point_array[20];
    cv::Mat findContour;
    cv::cuda::GpuMat G_image;
    cv::cuda::GpuMat HSV;
    cv::cuda::GpuMat inRange(cv::Size(this->Picture_p->preImage.cols, this->Picture_p->preImage.rows), CV_8UC1, cv::Scalar(255));
    cv::cuda::GpuMat filter_open;
    cv::cuda::GpuMat filter_close;
    G_image.upload(this->Picture_p->preImage);
    cv::cuda::cvtColor(G_image, HSV, cv::COLOR_BGR2HSV);
    inRange_gpu(HSV, *this->lowerFilter, *this->higherFilter, inRange);
    cv::Ptr<cv::cuda::Filter> morph_filter_open = cv::cuda::createMorphologyFilter(cv::MORPH_CLOSE, inRange.type(), open_kernel);
    // cv::Ptr<cv::cuda::Filter> morph_filter_close = cv::cuda::createMorphologyFilter(cv::MORPH_OPEN, inRange.type(), close_kernel);
    morph_filter_open->apply(inRange, filter_open);
    // morph_filter_close->apply(filter_open, filter_close);
    filter_open.download(this->Picture_p->endImage);
    cv::findContours(this->Picture_p->endImage, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
    int index = 0;
    for (int i = 0; i < contours.size(); i++)
    {
        // box = minAreaRect(Mat(contours[i]));
        // box.points(boxPts.data());
        boundRect = cv::boundingRect(cv::Mat(contours[i]));
        // rectangle(frame, boundRect.tl(), boundRect.br(), (0, 255, 0), 2,8 ,0);
        try
        {
            if (double(boundRect.height / boundRect.width) >= 1.3 && boundRect.height > 36 && boundRect.height > 20)
            {
                point_array[index] = boundRect;
                index++;
            }
        }
        catch (const char *msg)
        {
            std::cout << msg << std::endl;
            // continue;
        }
    }
    int point_near[2] = {0, 0};
    int min = 10000;
    for (int i = 0; i < index - 1; i++)
    {
        for (int j = i + 1; j < index; j++)
        {
            int value = abs(point_array[i].area() - point_array[j].area());
            if (value < min)
            {
                min = value;
                point_near[0] = i;
                point_near[1] = j;
            }
        }
    }
    try
    {
        cv::Rect rectangle_1 = point_array[point_near[0]];
        cv::Rect rectangle_2 = point_array[point_near[1]];
        if (rectangle_2.x == 0 || rectangle_1.x == 0)
        {
            throw "not enough points";
        }
        cv::Point point1 = cv::Point(rectangle_1.x + rectangle_1.width / 2, rectangle_1.y);
        cv::Point point2 = cv::Point(rectangle_1.x + rectangle_1.width / 2, rectangle_1.y + rectangle_1.height);
        cv::Point point3 = cv::Point(rectangle_2.x + rectangle_2.width / 2, rectangle_2.y);
        cv::Point point4 = cv::Point(rectangle_2.x + rectangle_2.width / 2, rectangle_2.y + rectangle_2.height);
        cv::Point p[4] = {point1, point2, point4, point3};
        std::cout << p[0] << p[1] << p[2] << p[3] << std::endl;
        for (int i = 0; i < 4; i++)
        {
            cv::line(this->Picture_p->endImage, p[i % 4], p[(i + 1) % 4], cv::Scalar(255, 255, 255), 2);
        }
    }
    catch (const char *msg)
    {
        // std::cout << msg << std::endl;
        // continue;
    }
    return PROCESSUCCESS;
}
