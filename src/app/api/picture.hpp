/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-02 12:50:18
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2025-01-15 17:36:24
 * @FilePath: /success2025/src/app/api/picture.hpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
#ifndef PICTURE_HPP
#define PICTURE_HPP

#include <opencv2/imgproc/types_c.h>
#include "opencv2/highgui/highgui.hpp"
#include "../ConfigurationReader.hpp"
#include "../../process/enemy_Inform.hpp"
#include "../../hardware/uart/Serial_Port.h"
#include "../../yolo/yolov8.hpp"
#include <atomic>

extern cv::Point KalmanPoint;
extern cv::Mat point_Kalman_image;
extern std::mutex point_Kalman_image_mtx; // 互斥锁
extern std::mutex displayImg_mtx;
extern std::vector<Object> objs;
// extern YOLOv8 *yolov8;
extern std::mutex EnemyInform_mtx;
class Picture
{
private:
    ConfigurationReader *Config;
    EnemyInform *EnemyInform_p;
    double T1; // to FPS
    double T2;

public:
    double spendTime;
    cv::Mat preImage;
    cv::Mat endImage;
    cv::Mat displayImage;
    char ImgShow() // in while
    {
        if (displayImage.empty())
            return cv::waitKey(1) == 'q';
        cv::namedWindow("success2025", cv::WINDOW_AUTOSIZE);
        cv::imshow("success2025", displayImage);
        return cv::waitKey(1) == 'q';
    }
    void CalculateTime()
    {
        this->spendTime = (this->T2 - this->T1) * 1000 / (cv::getTickFrequency());
    }
    void TimeBegin()
    {
        T1 = cv::getTickCount();
    }
    void TimeEnd()
    {
        T2 = cv::getTickCount();
    }
    void CvPutTextOnUI()
    {
        // if (static_cast<int>(1000 / spendTime) < 70)
        //     std::cout << "low_fps_warn  " << static_cast<int>(spendTime) << std::endl;
        if ((Config->enable_show == "true") && !(preImage.empty()))
        {
            // 定义字体（例如，HERSHEY_SIMPLEX）
            int fontFace = cv::FONT_HERSHEY_SIMPLEX;
            // 定义字体比例
            double fontScale = 1;
            // 定义文本的线宽（如果为负数或FONT_THICKNESS，则填充文本）
            int thickness = 3;
            // 定义文本颜色（例如，白色）
            cv::Scalar color(255, 255, 255);
            std::ostringstream oss;
            std::string result;
            oss.clear();
            oss.str("");
            if (Config->FPS_show == "true")
                oss << "FPS :  " << static_cast<int>(1000 / spendTime) << "   ";
            result = oss.str();
            cv::Point orgFPS(50, 50);
            cv::putText(displayImage, result, orgFPS, fontFace, fontScale, color, thickness, cv::LINE_AA);
            oss.clear();
            oss.str("");
            if (Config->time_show == "true")
                oss << "Time :  " << static_cast<int>(spendTime) << " ms" << "   ";
            result = oss.str();
            // 定义文本的位置（图像左下角的坐标）
            cv::Point orgTime(50, 100);
            cv::putText(displayImage, result, orgTime, fontFace, fontScale, color, thickness, cv::LINE_AA);
            if (this->EnemyInform_p->enemy_exist == 1)
            {
                {
                    std::lock_guard<std::mutex> EnemyInform_lock(EnemyInform_mtx); // 操作this->EnemyInform_p加锁
                    for (int i = 0; i < 4; i++)                                    // 画四个点
                    {
                        cv::line(this->displayImage, this->EnemyInform_p->p[i % 4], this->EnemyInform_p->p[(i + 1) % 4], cv::Scalar(255, 255, 255), 5);
                        cv::putText(this->displayImage, std::to_string(i), this->EnemyInform_p->p[i], cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255));
                    }
                    cv::circle(this->displayImage, this->EnemyInform_p->CenterPoint, 10, cv::Scalar(255, 255, 255)); // 画中心点坐标
                    {
                        std::lock_guard<std::mutex> lock(point_Kalman_image_mtx); // 加锁保护对 point_image 的访问
                        cv::circle(this->displayImage, KalmanPoint, 5, cv::Scalar(0, 255, 255));
                    }
                }
            }
            // {
            //     std::lock_guard<std::mutex> lock(displayImg_mtx);
            //     yolov8->DrawObjects(this->displayImage, objs);
            // }
        }
        /*debug------------------------------------------------------------------------------------------------ */
        if (this->Config->Debug_FPS == "true")
        {
            char buffer[50];
            memset(buffer, 0, sizeof(buffer));
            sprintf(buffer, " :%f,%d\n", spendTime, static_cast<int>(1000 / spendTime)); // y(0), y(1), y(2) 分别是 x, y, z
            SerialPortWriteBuffer(Uart_inf.UID0, buffer, sizeof(buffer));
        }
        /*debug------------------------------------------------------------------------------------------------ */
    }
    void CvPutTextOnUI(YOLOv8 *yolo)
    {
        // if (static_cast<int>(1000 / spendTime) < 70)
        //     std::cout << "low_fps_warn  " << static_cast<int>(spendTime) << std::endl;
        if ((Config->enable_show == "true") && !(preImage.empty()))
        {
            // 定义字体（例如，HERSHEY_SIMPLEX）
            int fontFace = cv::FONT_HERSHEY_SIMPLEX;
            // 定义字体比例
            double fontScale = 1;
            // 定义文本的线宽（如果为负数或FONT_THICKNESS，则填充文本）
            int thickness = 3;
            // 定义文本颜色（例如，白色）
            cv::Scalar color(255, 255, 255);
            std::ostringstream oss;
            std::string result;
            oss.clear();
            oss.str("");
            if (Config->FPS_show == "true")
                oss << "FPS :  " << static_cast<int>(1000 / spendTime) << "   ";
            result = oss.str();
            cv::Point orgFPS(50, 50);
            cv::putText(displayImage, result, orgFPS, fontFace, fontScale, color, thickness, cv::LINE_AA);
            oss.clear();
            oss.str("");
            if (Config->time_show == "true")
                oss << "Time :  " << static_cast<int>(spendTime) << " ms" << "   ";
            result = oss.str();
            // 定义文本的位置（图像左下角的坐标）
            cv::Point orgTime(50, 100);
            cv::putText(displayImage, result, orgTime, fontFace, fontScale, color, thickness, cv::LINE_AA);
            if (this->EnemyInform_p->enemy_exist == 1)
            {
                {
                    std::lock_guard<std::mutex> EnemyInform_lock(EnemyInform_mtx); // 操作this->EnemyInform_p加锁
                    for (int i = 0; i < 4; i++)                                    // 画四个点
                    {
                        cv::line(this->displayImage, this->EnemyInform_p->p[i % 4], this->EnemyInform_p->p[(i + 1) % 4], cv::Scalar(255, 255, 255), 5);
                        cv::putText(this->displayImage, std::to_string(i), this->EnemyInform_p->p[i], cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255));
                    }
                    cv::circle(this->displayImage, this->EnemyInform_p->CenterPoint, 10, cv::Scalar(255, 255, 255)); // 画中心点坐标
                    {
                        std::lock_guard<std::mutex> lock(point_Kalman_image_mtx); // 加锁保护对 point_image 的访问
                        cv::circle(this->displayImage, KalmanPoint, 5, cv::Scalar(10, 100, 255), -1);
                    }
                }
            }
            {
                std::lock_guard<std::mutex> lock(displayImg_mtx);
                yolo->DrawObjects(this->displayImage, objs);
            }
        }
    }
    Picture(ConfigurationReader *Config_p, EnemyInform *EnemyInform_p)
        : Config(Config_p), EnemyInform_p(EnemyInform_p) {}
    ~Picture() = default;
};

#endif // PICTURE_HPP
