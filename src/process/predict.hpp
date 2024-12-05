/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-27 19:54:16
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-12-06 02:29:28
 * @FilePath: /success2025/src/process/predict.hpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
#if !defined(PREDICT)
#define PREDICT

#include "../app/ConfigurationReader.hpp"
#include "./enemy_Inform.hpp"
#include "../utils/KalmanFilter/kalman.hpp"
#include <iostream>
#include <chrono>
#include <list>
#include <cmath>
#include <thread>

using namespace std;
using namespace chrono;
using namespace Kalman;

constexpr uint16_t Kalman_cycle = 10;

// 三维位置结构体
struct Position
{
    float x, y, z;

    // 计算两点之间的欧几里得距离（位移）
    float distanceTo(const Position &other) const
    {
        return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2) + pow(z - other.z, 2));
    }
};

// 存储时间戳和位置的结构体
struct TimestampNode
{
    system_clock::time_point timestamp; // 使用 system_clock
    Position position;
    // 构造函数，接收值类型参数
    TimestampNode(system_clock::time_point ts, Position pos)
        : timestamp(ts), position(pos)
    {
    }
};

class MYKalmanFilter
{
private:
    ConfigurationReader *reader_p;
    EnemyInform *enemy_p;

    Eigen::VectorXd y;

public:
    Kalman::KalmanFilter *kf;
    MYKalmanFilter(ConfigurationReader *reader, EnemyInform *enemy) : reader_p(reader),
                                                                      enemy_p(enemy), y(3) {

                                                                      };
    ~MYKalmanFilter()
    {
        delete this->kf;
    }
    void KalmanFilterInit();
    void KalmanUpdate(Eigen::VectorXd &y, uint64_t adtime, uint8_t reset = 1);
    Eigen::VectorXd xyzV2Eigen(double x, double y, double z, double vx, double vy, double vz);
    Eigen::VectorXd xyzV2Eigen(double x, double y, double z);
    cv::Mat xyzV2Mat4(double x, double y, double z);
};

class predict
{
private:
    ConfigurationReader *reader_p;
    EnemyInform *enemy_p;
    MYKalmanFilter Kf;

public:
    predict(ConfigurationReader *reader, EnemyInform *enemy) : reader_p(reader),
                                                               enemy_p(enemy), Kf(reader, enemy) {};
    ~predict() {};
    // KalmanInit();
    // KalmanUpdate();
    // kalmanPredict();
};

#endif // PREDICT
