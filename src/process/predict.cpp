/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-27 19:56:21
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-27 22:11:10
 * @FilePath: /success2025/src/process/predict.cpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
#include "./predict.hpp"

MYKalmanFilter::KalmanFilterInit()
{
    int n = 6; // Number of states
    int m = 3; // Number of measurements

    double dt = 1.0 / 30; // Time step 预测步长

    Eigen::MatrixXd A(n, n); // System dynamics matrix          系统状态转移矩阵
    Eigen::MatrixXd C(m, n); // Output matrix                   输出矩阵（测量转移矩阵）
    Eigen::MatrixXd Q(n, n); // Process noise covariance        预测噪声协方差
    Eigen::MatrixXd R(m, m); // Measurement noise covariance    测量噪声协方差
    Eigen::MatrixXd P(n, n); // Estimate error covariance       估计协方差

    // Discrete LTI projectile motion, measuring position only
    //   A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
    A << 1, 0, 0, dt, 0, 0,
        0, 1, 0, 0, dt, 0,
        0, 0, 1, 0, 0, dt,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;
    C << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0;
    // Reasonable covariance matrices
    Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
    R << 5;
    P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;
}
