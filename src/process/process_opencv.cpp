/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-08 10:06:09
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-15 22:31:41
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

#include <vector>

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
            if (double(boundRect.height / boundRect.width) >= 1.3 && boundRect.height > 36 && boundRect.width > 1)
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
    if (index >= 2)
    {
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
        // if (this->EnemyInform_p->p.empty() == false)
        // {
        this->getEuler();
        // }
    }
    return PROCESSUCCESS;
}

PROCESS_state process::getEuler()
{
    std::vector<cv::Point3d> obj = std::vector<cv::Point3d>{
        cv::Point3d(-HALF_LENGTH_LENGHT, -HALF_LENGTH_WIDTH, 0), // tl
        cv::Point3d(-HALF_LENGTH_LENGHT, HALF_LENGTH_WIDTH, 0),  // bl
        cv::Point3d(HALF_LENGTH_LENGHT, HALF_LENGTH_WIDTH, 0),   // br
        cv::Point3d(HALF_LENGTH_LENGHT, -HALF_LENGTH_WIDTH, 0),  // tr
    };
    if (true == cv::solvePnP(obj, this->EnemyInform_p->p, this->Reader->camera_matrix, this->Reader->distort_coefficient, this->EnemyInform_p->rvec, this->EnemyInform_p->tvec))
    {
        // 平移向量比较重要，旋转向量没那么重要了 经过变换，x轴朝右，y轴超前，z轴朝上

        double temp = this->EnemyInform_p->tvec.ptr<double>(0)[1];                                 // x = x
        this->EnemyInform_p->tvec.ptr<double>(0)[1] = this->EnemyInform_p->tvec.ptr<double>(0)[2]; // y = z
        this->EnemyInform_p->tvec.ptr<double>(0)[2] = -temp;                                       // z = -y

        cv2eigen(this->EnemyInform_p->tvec, e_T);  // 平移向量没有问题
        Rodrigues(this->EnemyInform_p->rvec, m_R); // 旋转顺序没有改变
        cv2eigen(m_R, e_R);

        // std::cout << this->EnemyInform_p->rvec << this->EnemyInform_p->tvec << std::endl;
        float distance = sqrt(this->EnemyInform_p->tvec.at<double>(0, 0) * this->EnemyInform_p->tvec.at<double>(0, 0) + this->EnemyInform_p->tvec.at<double>(1, 0) * this->EnemyInform_p->tvec.at<double>(1, 0) + this->EnemyInform_p->tvec.at<double>(2, 0) * this->EnemyInform_p->tvec.at<double>(2, 0)) / 10;
        // std::cout << distance << std::endl;
        this->EnemyInform_p->yaw = this->EnemyInform_p->rvec.at<double>(0, 2) * 2 * Pi;
        // std::cout << this->EnemyInform_p->yaw << std::endl;

        cv::putText(this->Picture_p->endImage, std::to_string(distance), this->EnemyInform_p->CenterPoint, 1, 1, cv::Scalar(255, 255, 255));

        // cv::Mat Rvec;
        // cv::Mat_<float> Tvec;
        // this->EnemyInform_p->rvec.convertTo(Rvec, CV_32F); // 旋转向量转换格式
        // this->EnemyInform_p->tvec.convertTo(Tvec, CV_32F); // 平移向量转换格式
        // cv::Mat_<float> rotMat(3, 3);
        // Rodrigues(Rvec, rotMat);
        // cv::Mat P_oc;
        // P_oc = -rotMat.inv() * Tvec;
        // std::cout << P_oc << std::endl;
    }

    return PROCESS_state();
}
