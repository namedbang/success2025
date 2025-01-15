/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-12 21:25:15
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2025-01-15 18:45:43
 * @FilePath: /success2025/src/process/enemy_Inform.hpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
#ifndef __ENEMY_INFORM_H__
#define __ENEMY_INFORM_H__

#include <opencv2/opencv.hpp>
#include <vector>
#include <atomic>

struct EnemyInform
{
    int point_near[2] = {0, 0}; // 最近矩形的index
    // cv::Point p[4];             // 识别出的矩形四个点1243
    std::vector<cv::Point2d> p = {
        cv::Point2d(0, 0), // 第一个点
        cv::Point2d(0, 0), // 第二个点
        cv::Point2d(0, 0), // 第三个点
        cv::Point2d(0, 0)  // 第四个点
    };
    /*
    1           2
    I           I
    I           I
    4           3
     */
    cv::Mat rvec; // 输出的旋转向量
    cv::Mat tvec; // 输出的平移向量
    cv::Mat R;    // 旋转矩阵
    cv::Mat T;
    cv::Point CenterPoint;
    // cv::Point3d CenterPoint3d;
    double distance;
    // std::atomic<double> yaw;   // 预测前的yaw
    // std::atomic<double> pitch; // 预测前的pitch
    double yaw;   // 预测前的yaw
    double pitch; // 预测前的pitch
    /*世界坐标系下------------------------- */
    std::atomic<double> yaw_world;   // 预测前的yaw世界坐标系
    std::atomic<double> pitch_world; // 预测前的pitch世界坐标系
    double Xw = 0;
    double Yw = 0;
    double Zw = 0;
    double Xvw;
    double Yvw;
    double Zvw;
    /*世界坐标系下------------------------- */
    bool enemy_exist; // 1 :敌人存在 2：敌人不存在
    /*kalman*/
    std::atomic<double> yaw_kalman;
    std::atomic<double> pitch_kalman;
};

#endif /* __ENEMY_INFORM_H__ */