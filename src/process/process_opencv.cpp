/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-08 10:06:09
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-12 23:21:22
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

cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
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
            if (double(boundRect.height / boundRect.width) >= 1.3 && boundRect.height > 36 && boundRect.width > 20)
            {
                point_array[index] = boundRect;
                index++; // 记录有多少矩形
            }
        }
        catch (const char *msg)
        {
            std::cout << msg << std::endl;
            // continue;
        }
    }
    int min = 100000; // 根据识别到的矩形对矩形进行排序
    for (int i = 0; i < index - 1; i++)
    {
        for (int j = i + 1; j < index; j++)
        {
            int value = abs(point_array[i].area() - point_array[j].area()); // 根据其面积相差的值进行匹配
            if (value < min)
            {
                min = value;
                this->EnemyInform_p->point_near[0] = i; // 将矩形index保存//
                this->EnemyInform_p->point_near[1] = j; //               //
            }
        }
    }
    try
    {
        cv::Rect rectangle_1 = point_array[this->EnemyInform_p->point_near[0]];
        cv::Rect rectangle_2 = point_array[this->EnemyInform_p->point_near[1]];
        if (rectangle_2.x == 0 || rectangle_1.x == 0)
        {
            throw "not enough points";
        }
        cv::Point point1 = cv::Point(rectangle_1.x + rectangle_1.width / 2, rectangle_1.y);
        cv::Point point2 = cv::Point(rectangle_1.x + rectangle_1.width / 2, rectangle_1.y + rectangle_1.height);
        cv::Point point3 = cv::Point(rectangle_2.x + rectangle_2.width / 2, rectangle_2.y);
        cv::Point point4 = cv::Point(rectangle_2.x + rectangle_2.width / 2, rectangle_2.y + rectangle_2.height);
        cv::Point p[4] = {point1, point2, point4, point3};
        for (u_char i = 0; i < 4; i++)
        {
            this->EnemyInform_p->p[i] = p[i];
        }
        cv::Point Center1 = cv::Point((p[0].x + p[2].x) / 2, (p[0].y + p[2].y) / 2);
        cv::Point Center2 = cv::Point((p[1].x + p[3].x) / 2, (p[1].y + p[3].y) / 2);
        this->EnemyInform_p->CenterPoint = cv::Point((Center1.x + Center2.x) / 2, (Center1.y + Center2.y) / 2);
        // std::cout << p[0] << p[1] << p[2] << p[3] << std::endl;
        for (int i = 0; i < 4; i++) // 画四个点
        {
            cv::line(this->Picture_p->endImage, p[i % 4], p[(i + 1) % 4], cv::Scalar(255, 255, 255), 2);
        }
        cv::circle(this->Picture_p->endImage, this->EnemyInform_p->CenterPoint, 10, cv::Scalar(255, 255, 255)); // 画中心点坐标
    }
    catch (const char *msg)
    {
        // std::cout << msg << std::endl;
        // continue;
    }
    return PROCESSUCCESS;
}

cv::Point process::getEuler()
{
    cv::solvePnP() return cv::Point();
}
