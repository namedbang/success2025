/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-08 10:06:09
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2025-01-10 21:37:54
 * @FilePath: /success2025/src/process/process_opencv.cpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */

#include "process_opencv.hpp"
#include "../cuda/inRange_gpu.cuh"
#include "../hardware/uart/Serial_Port.h"
#include "./predict.hpp"
#include "../utils/LowPassFilter.hpp"
#include "../hardware/gpio/GPIO.hpp"
#include "../yolo/yolov8.hpp"

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaobjdetect.hpp>
#include <jetson-utils/videoSource.h>
#include <jetson-utils/videoOutput.h>

#include <vector>
#include <stdio.h>
#include <cstring> // 包含 memset 的头文件
#include <algorithm>
#include <cmath>
#include <chrono>
#include <thread>
#include <list>
#include "../can/can.hpp"

using namespace std;
using namespace cv;
using namespace chrono;

cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

std::vector<cv::Vec4i> hierarchy;
std::vector<std::vector<cv::Point>> contours;
cv::Rect boundRect;
LowPassFilter lpf;

extern videoOutput *output;
extern Gpio gpio_h;
extern imu_angle_t imu_angle;

std::mutex mtx_k;           // 互斥量，用于同步访问共享资源
std::mutex EnemyInform_mtx; // 定义互斥锁

bool compareClockwise(const cv::Point2f &p1, const cv::Point2f &p2, const cv::Point2f &center)
{
    // 计算 p1 和 p2 与中心点的角度
    float angle1 = std::atan2(p1.y - center.y, p1.x - center.x);
    float angle2 = std::atan2(p2.y - center.y, p2.x - center.x);
    return angle1 < angle2;
}

void sortPointsClockwise(cv::Point2f points[4])
{
    // 计算四个点的中心点
    cv::Point2f center(0, 0);
    for (int i = 0; i < 4; ++i)
    {
        center += points[i];
    }
    center /= 4;

    // 对点进行排序，使它们按顺时针方向排列
    std::vector<cv::Point2f> sortedPoints(points, points + 4);
    std::sort(sortedPoints.begin(), sortedPoints.end(), [&center](const cv::Point2f &p1, const cv::Point2f &p2)
              { return compareClockwise(p1, p2, center); });

    // 更新原始 points 数组
    for (int i = 0; i < 4; ++i)
    {
        points[i] = sortedPoints[i];
    }
}
cv::cuda::GpuMat G_image;
cv::cuda::GpuMat temp_image;
std::vector<Object> objs;
extern YOLOv8 *yolov8;
extern std::mutex displayImg_mtx;
extern std::mutex perImg_mtx;

PROCESS_state process_opencv_cuda::processing()
{
    vector<RotatedRect> rotatedRects;
    vector<RotatedRect> point_array;

    cv::Mat PerImg;
    {
        std::lock_guard<std::mutex> lock(perImg_mtx);
        PerImg = this->Picture_p->preImage.clone();
    }

    if (PerImg.empty())
        return PROCESUNSUCCESS;
    /*interface yolo*/
    /********************************************************** */
    cv::cuda::Stream stream; // 创建CUDA流

    cv::cuda::GpuMat HSV;
    cv::cuda::GpuMat inRange(cv::Size(PerImg.cols, PerImg.rows), CV_8UC1, cv::Scalar(255));
    // cv::cuda::GpuMat filter_open;
    // cv::cuda::GpuMat filter_close;
    G_image.upload(PerImg, stream);
    cv::cuda::cvtColor(G_image, HSV, cv::COLOR_BGR2HSV, 0, stream);
    // TODO config
    cv::Size im_size(640, 640);
    const int num_labels = 9;
    const int topk = 9;
    const float score_thres = 0.25f;
    const float iou_thres = 0.65f;
    float f;
    std::chrono::steady_clock::time_point Tbegin, Tend;
    auto start = std::chrono::high_resolution_clock::now(); ///////////////////////////
    yolov8->CopyFromMat(PerImg, im_size);                   // 8ms
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "代码运行时间: " << duration.count() << " 毫秒\n";
    yolov8->Infer(); // 8ms
    {                // 加锁改变objs
        std::lock_guard<std::mutex> lock(displayImg_mtx);
        yolov8->PostProcess(objs, score_thres, iou_thres, topk, num_labels); // 1ms
    }

    // cv::Ptr<cv::cuda::Filter> morph_filter_open = cv::cuda::createMorphologyFilter(cv::MORPH_CLOSE, inRange.type(), open_kernel);
    // cv::Ptr<cv::cuda::Filter> morph_filter_close = cv::cuda::createMorphologyFilter(cv::MORPH_OPEN, inRange.type(), close_kernel);
    // morph_filter_open->apply(inRange, filter_open);
    // morph_filter_close->apply(filter_open, filter_close);
    // return PROCESS_state();
    if (gpio_h.SwitchVal == 1)
    {
        inRange_gpu(HSV, *this->lowerFilter, *this->higherFilter, inRange, objs, stream);
    }
    else
    {
        inRange_gpu(HSV, *this->lowerFilter_blue, *this->higherFilter_blue, inRange, objs, stream);
    }
    inRange.download(this->Picture_p->endImage, stream);
    stream.waitForCompletion(); // 等待CUDA流完成所有操作

    /***************************************************************************** */
    cv::findContours(this->Picture_p->endImage, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (size_t i = 0; i < contours.size(); i++)
    {
        RotatedRect box = minAreaRect(Mat(contours[i]));
        if (this->isValidLightBarBlob(box))
        {
            rotatedRects.emplace_back(box);
        }
    }
    // cout << rotatedRects.size() << endl;
    sort(rotatedRects.begin(), rotatedRects.end(), [](const RotatedRect &a, const RotatedRect &b) -> bool
         {
        if(a.center.x != b.center.x) return a.center.x < b.center.x;
        return a.center.y > b.center.y; });
    if (rotatedRects.size() >= 2)
    {
        try
        {
            cv::RotatedRect rectangle_1 = rotatedRects[0];
            cv::RotatedRect rectangle_2 = rotatedRects[1];
            if (rectangle_1.size.width <= 0 || rectangle_1.size.height <= 0 ||
                rectangle_2.size.width <= 0 || rectangle_2.size.height <= 0)
            {
                throw std::runtime_error("Invalid rectangle dimensions");
            }
            cv::Point2f points1[4];
            rectangle_1.points(points1); // 获取rectangle_1的四个角点
            cv::Point2f points2[4];
            rectangle_2.points(points2); // 获取rectangle_2的四个角点
            sortPointsClockwise(points1);
            sortPointsClockwise(points2);
            cv::Point point1 = cv::Point((points1[0].x + points1[1].x) / 2, (points1[0].y + points1[1].y) / 2);
            cv::Point point2 = cv::Point((points2[0].x + points2[1].x) / 2, (points2[0].y + points2[1].y) / 2);
            cv::Point point3 = cv::Point((points1[2].x + points1[3].x) / 2, (points1[2].y + points1[3].y) / 2);
            cv::Point point4 = cv::Point((points2[2].x + points2[3].x) / 2, (points2[2].y + points2[3].y) / 2);
            cv::Point p[4] = {point1, point2, point4, point3};             // 坑死我了，坑我一下午
            std::lock_guard<std::mutex> EnemyInform_lock(EnemyInform_mtx); // 操作this->EnemyInform_p加锁
            for (u_char i = 0; i < 4; i++)
            {
                this->EnemyInform_p->p[i] = p[i];
            }
            cv::Point Center1 = cv::Point((p[0].x + p[2].x) / 2, (p[0].y + p[2].y) / 2);
            cv::Point Center2 = cv::Point((p[1].x + p[3].x) / 2, (p[1].y + p[3].y) / 2);
            this->EnemyInform_p->CenterPoint = cv::Point((Center1.x + Center2.x) / 2, (Center1.y + Center2.y) / 2);
        }
        catch (const char *msg)
        {
            // std::cout << msg << std::endl;
            // continue;
        }
        bool is_point_in_rect = false;
        std::lock_guard<std::mutex> EnemyInform_lock(EnemyInform_mtx); // 操作this->EnemyInform_p加锁
        for (uint16_t i = 0; i < objs.size(); i++)
        {
            if (objs[i].rect.contains(this->EnemyInform_p->CenterPoint) && (objs[i].label == 7 || objs[i].label == 8 || objs[i].label == 6))
            {
                is_point_in_rect = true;
            }
        }
        if (is_point_in_rect)
        {
            this->getEuler();
            this->EnemyInform_p->enemy_exist = 1; // 敌人存在
        }
        else
        {
            this->EnemyInform_p->enemy_exist = 0; // 敌人不存在
        }
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
        cv::Point3d(-HALF_LENGTH_LENGHT, -HALF_LENGTH_WIDTH, 0), // 0//顺时针
        cv::Point3d(HALF_LENGTH_LENGHT, -HALF_LENGTH_WIDTH, 0),  // 1
        cv::Point3d(HALF_LENGTH_LENGHT, HALF_LENGTH_WIDTH, 0),   // 2
        cv::Point3d(-HALF_LENGTH_LENGHT, HALF_LENGTH_WIDTH, 0),  // 3
    };
    cv::Mat temp_tvec;
    /*这里必须要说一下，不知道为什么只有上面的错的（*****这个位置）才能得出不跳变的旋转向量*/
    /*有大神帮忙解释以下哇，不然我需要做两次pnp，我真的，卡一下午了*/
    // if (true == cv::solvePnP(obj, this->EnemyInform_p->p, this->Reader->camera_matrix, this->Reader->distort_coefficient, Trvec, temp_tvec, false, cv::SOLVEPNP_IPPE_SQUARE))
    if (true == cv::solvePnP(obj, this->EnemyInform_p->p, this->Reader->camera_matrix, this->Reader->distort_coefficient, this->EnemyInform_p->rvec, temp_tvec, false, cv::SOLVEPNP_IPPE))
    {
        // 平移向量比较重要，旋转向量没那么重要了以下是pnp后的平移向量[x,y,z]
        /*
            -y
            ^
            |   /z
            |  /
            | / ______>x   //没有人比我更懂形象
            相机
        */
        // 朝坐标轴发射方向看去，顺时针正为逆时针为负
        /*相机相对坐标*/ /*转换为相对于世界坐标系的坐标*/
        this->EnemyInform_p->tvec = this->CoordinateSystemChange(temp_tvec);
        this->EnemyInform_p->rvec.at<double>(0, 0) = imu_angle.Pitch_imu / 180.0 * M_PI; // 不需要x轴方向的旋转，其偏差过的，pnp问题透视问题
        this->EnemyInform_p->rvec.at<double>(0, 1) = imu_angle.yaw_imu / 180.0 * M_PI;
        this->EnemyInform_p->rvec.at<double>(0, 2) = imu_angle.roll_imu / 180.0 * M_PI;
        // https://github.com/opencv/opencv/issues/8813 pnp算法问题，不是错误
        /*debug------------------------------------ */
        // char buffer[100];
        // memset(buffer, 0, sizeof(buffer));
        // sprintf(buffer, ":%f,%f,%f\n", this->EnemyInform_p->rvec.at<double>(0, 0) / M_PI * 180, this->EnemyInform_p->rvec.at<double>(0, 1) / M_PI * 180, this->EnemyInform_p->rvec.at<double>(0, 2) / M_PI * 180);
        // SerialPortWriteBuffer(Uart_inf.UID0, buffer, sizeof(buffer));
        /*debug------------------------------------ */
        this->EnemyInform_p->distance = sqrt(this->EnemyInform_p->tvec.at<double>(0, 0) * this->EnemyInform_p->tvec.at<double>(0, 0) + this->EnemyInform_p->tvec.at<double>(1, 0) * this->EnemyInform_p->tvec.at<double>(1, 0) + this->EnemyInform_p->tvec.at<double>(2, 0) * this->EnemyInform_p->tvec.at<double>(2, 0)) / 10;
        this->EnemyInform_p->yaw = atan2(this->EnemyInform_p->tvec.at<double>(0, 0), this->EnemyInform_p->tvec.at<double>(0, 2)) / M_PI * 180;
        // std::cout << this->EnemyInform_p->yaw << std::endl;
        this->EnemyInform_p->pitch = atan2(-this->EnemyInform_p->tvec.at<double>(0, 1), sqrt(pow(this->EnemyInform_p->tvec.at<double>(0, 0), 2) + pow(this->EnemyInform_p->tvec.at<double>(0, 2), 2))) / M_PI * 180;
        // cv::putText(this->Picture_p->endImage, std::to_string(this->EnemyInform_p->distance), this->EnemyInform_p->CenterPoint, 1, 1, cv::Scalar(255, 255, 255));
        // std::cout << this->EnemyInform_p->pitch << std::endl;

        /* 构建外参矩阵 */
        /*
        | R T | * | x |  //R:(3,3);
        | 0 1 |   | y |  //T:(3,1);
                  | z |
                  | 1 |
        */
        std::lock_guard<std::mutex> lock(mtx_k); // 加锁--------------------------------------------
        cv::Rodrigues(this->EnemyInform_p->rvec, this->EnemyInform_p->R);
        this->EnemyInform_p->T = cv::Mat::zeros(4, 4, CV_64F); // 齐次变换矩阵（4x4）
        this->EnemyInform_p->R.copyTo(this->EnemyInform_p->T(cv::Rect(0, 0, 3, 3)));
        this->EnemyInform_p->T.at<double>(0, 3) = 0; // 填充平移向量部分
        this->EnemyInform_p->T.at<double>(1, 3) = 0;
        this->EnemyInform_p->T.at<double>(2, 3) = 0;
        this->EnemyInform_p->T.at<double>(3, 3) = 1.0; // 齐次坐标的最后一行是[0, 0, 0, 1]
                                                       // 相机坐标系中的一个点（齐次坐标）

        cv::Mat point_camera = (cv::Mat_<double>(4, 1) <<                         //
                                    this->EnemyInform_p->tvec.at<double>(0, 0),   //
                                this->EnemyInform_p->tvec.at<double>(0, 1),       //
                                this->EnemyInform_p->tvec.at<double>(0, 2), 1.0); //
                                                                                  // 应用齐次变换矩阵将点从相机坐标系转换到世界坐标系
        cv::Mat point_world = this->EnemyInform_p->T.inv() * point_camera;
        // cv::Mat point_world = T * point_camera;

        this->EnemyInform_p->Xw = point_world.at<double>(0, 0);
        this->EnemyInform_p->Yw = point_world.at<double>(1, 0);
        this->EnemyInform_p->Zw = point_world.at<double>(2, 0);

        this->EnemyInform_p->yaw_world = atan2(this->EnemyInform_p->Xw, this->EnemyInform_p->Zw) / M_PI * 180;
        this->EnemyInform_p->pitch_world = atan2(-this->EnemyInform_p->Yw, sqrt(pow(this->EnemyInform_p->Xw, 2) + pow(this->EnemyInform_p->Zw, 2))) / M_PI * 180;

        /*debug------------------------------------ */
        // char buffer[100];
        // memset(buffer, 0, sizeof(buffer));
        // sprintf(buffer, " :%f,%f,%f\n", this->EnemyInform_p->Xw, this->EnemyInform_p->Yw, this->EnemyInform_p->Zw);
        // SerialPortWriteBuffer(Uart_inf.UID0, buffer, sizeof(buffer));
        // std::cout << Xw << ", " << Yw << ", " << Zw << std::endl;

        // char buffer[100];
        // memset(buffer, 0, sizeof(buffer));
        // sprintf(buffer, " :%f,%f\n", this->EnemyInform_p->yaw, this->EnemyInform_p->pitch);
        // SerialPortWriteBuffer(Uart_inf.UID0, buffer, sizeof(buffer));

        // char buffer[100];
        // memset(buffer, 0, sizeof(buffer));
        // sprintf(buffer, " :%f,%f\n", this->EnemyInform_p->yaw_world, this->EnemyInform_p->pitch_world);
        // SerialPortWriteBuffer(Uart_inf.UID0, buffer, sizeof(buffer));
        /*debug------------------------------------ */
        /*测速 两点之间的速度*/
        MeasureSpeed(this->EnemyInform_p->Xw, this->EnemyInform_p->Yw, this->EnemyInform_p->Zw);
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

bool process::checkAngle(const RotatedRect &rrect)
{
    double angle = rrect.angle;
    return rrect.size.width < rrect.size.height ? angle <= 30 : angle >= 60;
}

bool process::checkArea(double area)
{
    return area >= 30 && area < 400 * 100;
}

bool process::checkAspectRatio(double ratio)
{
    return true;
    // cout << ratio << endl;
    return ratio <= 10 && ratio >= 2.5 / 2 || ratio <= 2. / 2.5 && ratio >= 1. / 10;
    // return ratio <= 10 && ratio >= 1. / 10;
}

bool process::isValidLightBarBlob(const RotatedRect &rrect)
{
    if (
        checkAspectRatio(rrect.size.aspectRatio()) &&
        checkArea(rrect.size.area()) &&
        checkAngle(rrect))
    {
        return true;
    }
    // DLOG(INFO) << "not lightbar: " << rrect.size.aspectRatio() << " " << rrect.size.area() << " " << rrect.angle;
    return false;
}

void process::MeasureSpeed(const double x, const double y, const double z)
{
    // 获取当前时间戳/*测速 两点之间的速度*/
    system_clock::time_point now = system_clock::now(); // 使用 system_clock
    Position currentPosition;
    currentPosition.x = x;
    currentPosition.y = y;
    currentPosition.z = z;
    // cout << x << endl;
    // 将时间戳和位置插入到链表中
    timestamps.push_back(TimestampNode(now, currentPosition));
    // 如果链表超出了最大大小，移除最旧的节点，模拟循环链表
    if (timestamps.size() > max_size)
    {
        timestamps.pop_front(); // 移除链表头部元素
    }
    if (timestamps.size() > 1)
    {
        // 获取链表中的最后两个节点
        auto lastNode = timestamps.back();                    // 获取最后一个节点
        auto secondLastNode = *std::prev(--timestamps.end()); // 获取倒数第二个节点

        // 计算时间差（以毫秒为单位）
        auto duration = duration_cast<milliseconds>(lastNode.timestamp - secondLastNode.timestamp);
        float timeDiff = duration.count(); // 时间差（毫秒）

        // 计算各个方向上的位移
        float displacement_x = lastNode.position.x - secondLastNode.position.x;
        float displacement_y = lastNode.position.y - secondLastNode.position.y;
        float displacement_z = lastNode.position.z - secondLastNode.position.z;

        // cout << lastNode.position.x << secondLastNode.position.x << endl;
        // 计算速度：速度 = 位移 / 时间差
        if (timeDiff > 0)
        {
            float velocity_x = displacement_x / timeDiff * 1000; // x轴速度（单位：mm/s）
            float velocity_y = displacement_y / timeDiff * 1000; // y轴速度（单位：mm/s）
            float velocity_z = displacement_z / timeDiff * 1000; // z轴速度（单位：mm/s）
                                                                 // float velocity_x = displacement_x / timeDiff; // x轴速度（单位：mm/ms）
                                                                 // float velocity_y = displacement_y / timeDiff; // y轴速度（单位：mm/ms）
                                                                 // float velocity_z = displacement_z / timeDiff; // z轴速度（单位：mm/ms）
            auto X_X = lpf.update(velocity_x, timeDiff / 1000, 1);
            auto Y_Y = lpf.update(velocity_y, timeDiff / 1000, 1);
            auto Z_Z = lpf.update(velocity_z, timeDiff / 1000, 1);

            this->EnemyInform_p->Xvw = X_X;
            this->EnemyInform_p->Yvw = Y_Y;
            this->EnemyInform_p->Zvw = Z_Z;
            /*debug------------------------------------ */
            // cout << timeDiff << "  "
            //      << velocity_x << "  "
            //      << velocity_y << "  "
            //      << velocity_z << endl;
            // char buffer[50];
            // memset(buffer, 0, sizeof(buffer));
            // sprintf(buffer, " :%f,%f,%f\n", X_X, velocity_z, Z_Z);
            // SerialPortWriteBuffer(Uart_inf.UID0, buffer, sizeof(buffer));
            /*debug------------------------------------ */
        }
        else
        {
            cout << "Time difference is too small, skipping velocity calculation." << endl;
        }
    }
}
