/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-27 19:56:21
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2025-01-05 19:01:24
 * @FilePath: /success2025/src/process/predict.cpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
#include "./predict.hpp"
#include "../utils/KalmanFilter/kalman.hpp"
#include "Eigen/Dense"
#include "predict.hpp"

double MYKalmanFilter::computeADTime(double v0, double x_target_mm)
{
    // 将目标距离从毫米转换为米
    double x_target = x_target_mm / 1000.0;
    if (AIR_RESISTANCE_COEFFICIENT == 0)
    {                                  // 无空气阻力时使用匀速运动公式
        return (x_target / v0) * 1000; // 返回毫秒单位
    }
    // 计算时间 (单位为秒)
    double time_seconds = (MASS / (AIR_RESISTANCE_COEFFICIENT * v0)) * (exp((AIR_RESISTANCE_COEFFICIENT / MASS) * x_target) - 1);
    // 将时间从秒转换为毫秒
    return time_seconds * 1000;
}

void MYKalmanFilter::KalmanFilterInit()
{
    int n = 6;      // Number of states
    int m = 3;      // Number of measurements
    double dt = 10; // Time step 预测步长
    double t = 0;
    // Discrete LTI projectile motion, measuring position only
    Eigen::MatrixXd A(n, n); // System dynamics matrix          系统状态转移矩阵
    Eigen::MatrixXd C(m, n); // Output matrix                   输出矩阵（测量转移矩阵）
    Eigen::MatrixXd Q(n, n); // Process noise covariance        预测噪声协方差
    Eigen::MatrixXd R(m, m); // Measurement noise covariance    测量噪声协方差
    Eigen::MatrixXd P(n, n); // Estimate error covariance       估计协方差

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
    // Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
    // R << 5;
    // P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;
    // 将读出来的Q,P和R转化为Eigen格式
    std::copy(reader_p->Q.ptr<double>(0), reader_p->Q.ptr<double>(0) + reader_p->Q.total(), Q.data()); // Q
    std::copy(reader_p->R.ptr<double>(0), reader_p->R.ptr<double>(0) + reader_p->R.total(), R.data()); // R
    std::copy(reader_p->P.ptr<double>(0), reader_p->P.ptr<double>(0) + reader_p->P.total(), P.data()); // P

    std::cout << "A: \n"
              << A << std::endl;
    std::cout << "C: \n"
              << C << std::endl;
    std::cout << "Q: \n"
              << Q << std::endl;
    std::cout << "R: \n"
              << R << std::endl;
    std::cout << "P: \n"
              << P << std::endl;
    this->kf = new KalmanFilter(dt, A, C, Q, R, P);
    // kf->init(t, xyzV2Eigen(this->enemy_p->Xw, this->enemy_p->Yw, this->enemy_p->Zw, this->enemy_p->Xvw, this->enemy_p->Yvw, this->enemy_p->Zvw));
    kf->init(t, xyzV2Eigen(this->enemy_p->Xw, this->enemy_p->Yw, this->enemy_p->Zw, 10, 10, 10));

    // return kf;
}

void MYKalmanFilter::KalmanUpdate(Eigen::VectorXd &y, uint64_t adtime, uint8_t reset)
{
    uint64_t adpower = adtime / Kalman_cycle; // 单位ms
    this->kf->update(y, adpower, reset);
    // this->kf->update(y);
}

Eigen::VectorXd MYKalmanFilter::xyzV2Eigen(double x, double y, double z, double vx, double vy, double vz)
{
    Eigen::VectorXd X(6);
    X << x, y, z, vx, vy, vz;
    return X;
}

Eigen::VectorXd MYKalmanFilter::xyzV2Eigen(double x, double y, double z)
{
    Eigen::VectorXd X(3);
    X << x, y, z;
    return X;
}

cv::Mat MYKalmanFilter::xyzV2Mat4(double x, double y, double z)
{
    cv::Mat point_camera = (cv::Mat_<double>(4, 1)
                                << x,
                            y,
                            z, 1.0);
    // 应用齐次变换矩阵将点从相机坐标系转换到世界坐标系
    return point_camera;
}