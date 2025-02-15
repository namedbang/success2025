/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-27 19:51:14
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2025-01-15 17:10:53
 * @FilePath: /success2025/src/utils/KalmanFilter/kalman.cpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
/**
 * Implementation of KalmanFilter class.
 *
 * @author: Hayk Martirosyan
 * @date: 2014.11.15
 */

#include <iostream>
#include <stdexcept>

#include "./kalman.hpp"
#include "kalman.hpp"

using namespace Kalman;

KalmanFilter::KalmanFilter(
    double dt,
    const Eigen::MatrixXd &A,
    const Eigen::MatrixXd &C,
    const Eigen::MatrixXd &Q,
    const Eigen::MatrixXd &R,
    const Eigen::MatrixXd &P)
    : A(A), C(C), Q(Q), R(R), P0(P),
      m(C.rows()), n(A.rows()), dt(dt), initialized(false),
      I(n, n), x_hat(n), x_hat_new(n)
{
  I.setIdentity();
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, const Eigen::VectorXd &x0)
{
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  initialized = true;
}

void KalmanFilter::init()
{
  x_hat.setZero();
  P = P0;
  t0 = 0;
  t = t0;
  initialized = true;
}

void KalmanFilter::update(const Eigen::VectorXd &y)
{

  if (!initialized)
    throw std::runtime_error("Filter is not initialized!");

  x_hat_new = A * x_hat;
  P = A * P * A.transpose() + Q;
  K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
  x_hat_new += K * (y - C * x_hat_new);
  P = (I - K * C) * P;
  x_hat = x_hat_new;
  t += dt;
}

// using namespace Eigen;

// 使用单位矩阵和for循环计算矩阵的幂
Eigen::MatrixXd matrixPower(const Eigen::MatrixXd &matrix, uint64_t power)
{
  if (power == 0)
    return Eigen::MatrixXd::Identity(matrix.rows(), matrix.cols()); // A^0 = I
  if (power == 1)
    return matrix; // A^1 = A

  Eigen::MatrixXd result = matrix;
  for (uint64_t i = 1; i < power; ++i)
    result *= matrix;

  return result;
}

void KalmanFilter::update(const Eigen::VectorXd &y, uint64_t adpower, uint8_t reset)
{

  if (!initialized)
    throw std::runtime_error("Filter is not initialized!");

  x_hat_new = A * x_hat;
  P = A * P * A.transpose() + Q;
  K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
  x_hat_new += K * (y - C * x_hat_new);
  P = (I - K * C) * P;
  x_hat = x_hat_new;
  Eigen::MatrixXd TM = matrixPower(A, adpower); // A 的 adpower 次幂
  x_hat_out = TM * x_hat;
  t += dt;
  // 如果需要复位
  if (reset == 0)
  {
    x_hat_out = y;
    P.setIdentity(); // 协方差矩阵重置为单位矩阵
    x_hat.setZero(); // 状态重置为零
  }
}

void KalmanFilter::update(const Eigen::VectorXd &y, double dt, const Eigen::MatrixXd A)
{

  this->A = A;
  this->dt = dt;
  update(y);
}
