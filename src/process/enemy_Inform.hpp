/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-12 21:25:15
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-27 21:00:19
 * @FilePath: /success2025/src/process/enemy_Inform.hpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
#ifndef __ENEMY_INFORM_H__
#define __ENEMY_INFORM_H__

#include <opencv2/opencv.hpp>
#include <vector>

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
    1           3
    I           I
    I           I
    2           4
     */
    cv::Mat rvec; // 输出的旋转向量
    cv::Mat tvec; // 输出的平移向量
    cv::Mat R;    // 旋转矩阵
    cv::Point CenterPoint;
    // cv::Point3d CenterPoint3d;
    double distance;
    double yaw;           // 预测前的yaw
    double pitch;         // 预测前的pitch
    bool enemy_exist = 0; // 1 :敌人存在 2：敌人不存在
};

#endif /* __ENEMY_INFORM_H__ */