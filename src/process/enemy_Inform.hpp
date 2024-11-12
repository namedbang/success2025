/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-12 21:25:15
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-12 22:20:11
 * @FilePath: /success2025/src/process/enemy_Inform.hpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
#ifndef __ENEMY_INFORM_H__
#define __ENEMY_INFORM_H__

#include <opencv2/opencv.hpp>

struct EnemyInform
{
    int point_near[2] = {0, 0}; // 最近矩形的index
    cv::Point p[4];             // 识别出的矩形四个点1243
    /*
    1           3
    I           I
    I           I
    2           4
     */
    cv::Point CenterPoint;
};

#endif /* __ENEMY_INFORM_H__ */