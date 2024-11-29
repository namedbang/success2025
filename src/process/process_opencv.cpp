/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-08 10:06:09
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-29 23:34:15
 * @FilePath: /success2025/src/process/process_opencv.cpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */

#include "process_opencv.hpp"
#include "../cuda/inRange_gpu.cuh"
#include "../hardware/uart/Serial_Port.h"

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaobjdetect.hpp>

#include <vector>
#include <stdio.h>
#include <cstring> // 包含 memset 的头文件

using namespace std;
using namespace cv;

cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

std::vector<cv::Vec4i> hierarchy;
std::vector<std::vector<cv::Point>> contours;
cv::Rect boundRect;

PROCESS_state process_opencv_cuda::processing()
{
    vector<RotatedRect> rotatedRects;
    vector<RotatedRect> point_array;
    // std::cout << "1" << std::endl;
    if (this->Picture_p->preImage.empty())
        return PROCESUNSUCCESS;
    // cv::Rect point_array[20];
    // std::vector<cv::RotatedRect> point_array;
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
    // morph_filter_close->apply(filter_open, filter_close);
    inRange.download(this->Picture_p->endImage);
    cv::findContours(this->Picture_p->endImage, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
    int index = 0;
    for (size_t i = 0; i < contours.size(); i++)
    {
        RotatedRect box = minAreaRect(Mat(contours[i]));
        rotatedRects.push_back(box);
    }
    for (const RotatedRect &box : rotatedRects)
    {
        if (box.size.height > 7 && box.size.width > 1)
        {
            point_array.push_back(box);
            index++;
        }
        // cout << box.size.height << " " << box.size.width << endl;
    }
    cout << point_array.size() << endl;
    // // 根据面积差异找到最接近的两个矩形
    int min = 100000;
    // int point_near[2] = {-1, -1}; // 初始化索引为无效值
    for (int i = 0; i < index - 1; i++)
    {
        for (int j = i + 1; j < index; j++)
        {
            double approxArea1 = point_array[i].size.width * point_array[i].size.height;
            double approxArea2 = point_array[j].size.width * point_array[j].size.height;
            double areaDiff = std::abs(approxArea1 - approxArea2);
            // double areaDiff = abs(point_array[i].area() - point_array[j].area());
            if (areaDiff < min)
            {
                min = areaDiff;
                this->EnemyInform_p->point_near[0] = i;
                this->EnemyInform_p->point_near[1] = j;
            }
        }
    }

    // // for (int i = 0; i < contours.size(); i++)
    // // {
    // //     // box = minAreaRect(Mat(contours[i]));
    // //     // box.points(boxPts.data());
    // //     boundRect = cv::boundingRect(cv::Mat(contours[i]));
    // //     // rectangle(frame, boundRect.tl(), boundRect.br(), (0, 255, 0), 2,8 ,0);
    // //     try
    // //     {
    // //         // if (double(boundRect.height / boundRect.width) >= 1.3 && boundRect.height > 36 && boundRect.width > 1)
    // //         if (boundRect.height > 36 && boundRect.width > 1)
    // //         {
    // //             point_array[index] = boundRect;
    // //             index++; // 记录有多少矩形
    // //         }
    // //     }
    // //     catch (const char *msg)
    // //     {
    // //         std::cout << msg << std::endl;
    // //         // continue;
    // //     }
    // // }
    // // int min = 100000; // 根据识别到的矩形对矩形进行排序
    // // for (int i = 0; i < index - 1; i++)
    // // {
    // //     for (int j = i + 1; j < index; j++)
    // //     {
    // //         int value = abs(point_array[i].area() - point_array[j].area()); // 根据其面积相差的值进行匹配
    // //         if (value < min)
    // //         {
    // //             min = value;
    // //             this->EnemyInform_p->point_near[0] = i; // 将矩形index保存//
    // //             this->EnemyInform_p->point_near[1] = j; //               //
    // //         }
    // //     }
    // // }
    if (index >= 2)
    {
        // // 判断其x轴的大小
        // if (point_array[this->EnemyInform_p->point_near[0]].y < point_array[this->EnemyInform_p->point_near[1]].x)
        // {
        //     int temp = this->EnemyInform_p->point_near[0];
        //     this->EnemyInform_p->point_near[0] = this->EnemyInform_p->point_near[1];
        //     this->EnemyInform_p->point_near[1] = temp;
        // } // 如果...就交换两矩形位置

        try
        {
            // cv::Rect rectangle_1 = point_array[this->EnemyInform_p->point_near[0]];
            // cv::Rect rectangle_2 = point_array[this->EnemyInform_p->point_near[1]];
            cv::RotatedRect rectangle_1 = point_array[this->EnemyInform_p->point_near[0]];
            cv::RotatedRect rectangle_2 = point_array[this->EnemyInform_p->point_near[1]];
            if (rectangle_1.size.width <= 0 || rectangle_1.size.height <= 0 ||
                rectangle_2.size.width <= 0 || rectangle_2.size.height <= 0)
            {
                throw std::runtime_error("Invalid rectangle dimensions");
            }
            // cv::Point point1 = cv::Point(rectangle_1.x + rectangle_1.width / 2, rectangle_1.y);
            // cv::Point point2 = cv::Point(rectangle_1.x + rectangle_1.width / 2, rectangle_1.y + rectangle_1.height);
            // cv::Point point3 = cv::Point(rectangle_2.x + rectangle_2.width / 2, rectangle_2.y);
            // cv::Point point4 = cv::Point(rectangle_2.x + rectangle_2.width / 2, rectangle_2.y + rectangle_2.height);
            // cv::Point p[4] = {point1, point2, point4, point3};//坑死我了，坑我一下午
            cv::Point2f points1[4];
            rectangle_1.points(points1); // 获取rectangle_1的四个角点
            cv::Point2f points2[4];
            rectangle_2.points(points2); // 获取rectangle_2的四个角点
            cv::Point point1 = cv::Point((points1[0].x + points1[1].x) / 2, (points1[0].y + points1[1].y) / 2);
            cv::Point point2 = cv::Point((points2[0].x + points2[1].x) / 2, (points2[0].y + points2[1].y) / 2);
            cv::Point point3 = cv::Point((points1[2].x + points1[3].x) / 2, (points1[2].y + points1[3].y) / 2);
            cv::Point point4 = cv::Point((points2[2].x + points2[3].x) / 2, (points2[2].y + points2[3].y) / 2);
            cv::Point p[4] = {point3, point1, point2, point4};
            for (u_char i = 0; i < 4; i++)
            {
                this->EnemyInform_p->p[i] = p[i];
            }
            cv::Point Center1 = cv::Point((p[0].x + p[2].x) / 2, (p[0].y + p[2].y) / 2);
            cv::Point Center2 = cv::Point((p[1].x + p[3].x) / 2, (p[1].y + p[3].y) / 2);
            this->EnemyInform_p->CenterPoint = cv::Point((Center1.x + Center2.x) / 2, (Center1.y + Center2.y) / 2);
            std::cout << p[0] << p[1] << p[2] << p[3] << std::endl;
            for (int i = 0; i < 4; i++) // 画四个点
            {
                cv::line(this->Picture_p->preImage, p[i % 4], p[(i + 1) % 4], cv::Scalar(255, 255, 255), 5);
                cv::putText(this->Picture_p->preImage, std::to_string(i), p[i], cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255));
            }
            cv::circle(this->Picture_p->preImage, this->EnemyInform_p->CenterPoint, 10, cv::Scalar(255, 255, 255)); // 画中心点坐标
        }
        catch (const char *msg)
        {
            // std::cout << msg << std::endl;
            // continue;
        }
        // if (this->EnemyInform_p->p.empty() == false)
        // {
        // this->getEuler();
        this->EnemyInform_p->enemy_exist = 1; // 敌人存在
        // }
    }
    else
    {
        this->EnemyInform_p->enemy_exist = 0; // 敌人不存在
    }
    return PROCESSUCCESS;
}

PROCESS_state process::getEuler()
{
    std::vector<cv::Point3d> obj = std::vector<cv::Point3d>{
        // 顺时针
        cv::Point3d(-HALF_LENGTH_LENGHT, HALF_LENGTH_WIDTH, 0),  // tl
        cv::Point3d(HALF_LENGTH_LENGHT, HALF_LENGTH_WIDTH, 0),   // bl
        cv::Point3d(HALF_LENGTH_LENGHT, -HALF_LENGTH_WIDTH, 0),  // br
        cv::Point3d(-HALF_LENGTH_LENGHT, -HALF_LENGTH_WIDTH, 0), // tr
    };
    /*下面这个是错的只为旋转角服务*/
    std::vector<cv::Point3d> obj_fR = std::vector<cv::Point3d>{
        // 顺时针
        cv::Point3d(-HALF_LENGTH_LENGHT, -HALF_LENGTH_WIDTH, 0), // 0
        cv::Point3d(HALF_LENGTH_LENGHT, -HALF_LENGTH_WIDTH, 0),  // 1
        cv::Point3d(HALF_LENGTH_LENGHT, HALF_LENGTH_WIDTH, 0),   // 2
        cv::Point3d(-HALF_LENGTH_LENGHT, -HALF_LENGTH_WIDTH, 0), // 3////这行是错的**********
    };
    cv::Mat Trvec;
    cv::Mat Ttvec;
    cv::Mat temp_tvec;
    /*这里必须要说一下，不知道为什么只有上面的错的（*****这个位置）才能得出不跳变的旋转向量*/
    /*有大神帮忙解释以下哇，不然我需要做两次pnp，我真的，卡一下午了*/
    if (true == cv::solvePnP(obj, this->EnemyInform_p->p, this->Reader->camera_matrix, this->Reader->distort_coefficient, Trvec, temp_tvec, false, cv::SOLVEPNP_IPPE_SQUARE))
    // if (true == cv::solvePnP(obj, this->EnemyInform_p->p, this->Reader->camera_matrix, this->Reader->distort_coefficient, this->EnemyInform_p->rvec, temp_tvec, false))
    {
        cv::solvePnP(obj_fR, this->EnemyInform_p->p, this->Reader->camera_matrix, this->Reader->distort_coefficient, this->EnemyInform_p->rvec, Ttvec, false, cv::SOLVEPNP_IPPE_SQUARE);
        // 平移向量比较重要，旋转向量没那么重要了以下是pnp后的平移向量[x,y,z]
        /*
            -y
            ^
            |   /z
            |  /
            | / ______>x   //没有人比我更懂形象
            相机
        */
        // 朝坐标轴发射方向看去，顺时针负为逆时针为正
        /*相机相对坐标*/ /*转换为相对于世界坐标系的坐标*/
        this->EnemyInform_p->tvec = this->CoordinateSystemChange(temp_tvec);
        // std::cout << this->EnemyInform_p->tvec << std::endl;
        /*debug------------------------------------ */
        char buffer[100];
        memset(buffer, 0, sizeof(buffer));
        sprintf(buffer, ":%f,%f,%f\n", this->EnemyInform_p->rvec.at<double>(0, 0) / M_PI * 180, this->EnemyInform_p->rvec.at<double>(0, 1) / M_PI * 180, this->EnemyInform_p->rvec.at<double>(0, 2) / M_PI * 180);
        SerialPortWriteBuffer(Uart_inf.UID0, buffer, sizeof(buffer));
        /*debug------------------------------------ */
        this->EnemyInform_p->distance = sqrt(this->EnemyInform_p->tvec.at<double>(0, 0) * this->EnemyInform_p->tvec.at<double>(0, 0) + this->EnemyInform_p->tvec.at<double>(1, 0) * this->EnemyInform_p->tvec.at<double>(1, 0) + this->EnemyInform_p->tvec.at<double>(2, 0) * this->EnemyInform_p->tvec.at<double>(2, 0)) / 10;
        this->EnemyInform_p->yaw = atan2(this->EnemyInform_p->tvec.at<double>(0, 0), this->EnemyInform_p->tvec.at<double>(0, 2)) / M_PI * 180;
        std::cout << this->EnemyInform_p->yaw << std::endl;
        this->EnemyInform_p->pitch = atan2(-this->EnemyInform_p->tvec.at<double>(0, 1), sqrt(pow(this->EnemyInform_p->tvec.at<double>(0, 0), 2) + pow(this->EnemyInform_p->tvec.at<double>(0, 2), 2))) / M_PI * 180;
        cv::putText(this->Picture_p->endImage, std::to_string(this->EnemyInform_p->distance), this->EnemyInform_p->CenterPoint, 1, 1, cv::Scalar(255, 255, 255));
        std::cout << this->EnemyInform_p->pitch << std::endl;

        /* 构建外参矩阵 */
        /*
        | R T | * | x |  //R:(3,3);
        | 0 1 |   | y |  //T:(3,1);
                  | z |
                  | 1 |
        */
        cv::Rodrigues(this->EnemyInform_p->rvec, this->EnemyInform_p->R);
        cv::Mat T = cv::Mat::zeros(4, 4, CV_64F); // 齐次变换矩阵（4x4）
        this->EnemyInform_p->R.copyTo(T(cv::Rect(0, 0, 3, 3)));
        T.at<double>(0, 3) = 0; // 填充平移向量部分
        T.at<double>(1, 3) = 0;
        T.at<double>(2, 3) = 0;
        T.at<double>(3, 3) = 1.0; // 齐次坐标的最后一行是[0, 0, 0, 1]
                                  // 相机坐标系中的一个点（齐次坐标）

        cv::Mat point_camera = (cv::Mat_<double>(4, 1) <<                         //
                                    this->EnemyInform_p->tvec.at<double>(0, 0),   //
                                this->EnemyInform_p->tvec.at<double>(0, 1),       //
                                this->EnemyInform_p->tvec.at<double>(0, 2), 1.0); //
                                                                                  // 应用齐次变换矩阵将点从相机坐标系转换到世界坐标系
        cv::Mat point_world = T.inv() * point_camera;
        // cv::Mat point_world = T * point_camera;
        if (point_world.at<double>(2, 0) != 0)
        {
            double Xw = point_world.at<double>(0, 0);
            double Yw = point_world.at<double>(1, 0);
            double Zw = point_world.at<double>(2, 0);
            /*debug------------------------------------ */
            // char buffer[30];
            // memset(buffer, 0, sizeof(buffer));
            // sprintf(buffer, " :%.2f,%.2f,%.2f\n", Xw, Yw, Zw);
            // SerialPortWriteBuffer(Uart_inf.UID0, buffer, sizeof(buffer));
            // std::cout << Xw << ", " << Yw << ", " << Zw << std::endl;
            /*debug------------------------------------ */
        }
    }

    return PROCESS_state();
}

cv::Mat process::CoordinateSystemChange(cv::Mat xyz)
{
    cv::Mat XYZ;
    XYZ = xyz;
    XYZ.at<double>(0, 0) += this->Reader->TVEC.at<double>(0, 0);
    XYZ.at<double>(0, 1) += this->Reader->TVEC.at<double>(0, 1);
    XYZ.at<double>(0, 2) += this->Reader->TVEC.at<double>(0, 2);
    return XYZ;
}
