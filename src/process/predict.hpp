/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-27 19:54:16
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-28 18:53:31
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

class MYKalmanFilter
{
private:
    ConfigurationReader *reader_p;
    EnemyInform *enemy_p;

public:
    MYKalmanFilter(ConfigurationReader *reader, EnemyInform *enemy) : reader_p(reader),
                                                                      enemy_p(enemy) {

                                                                      };
    ~MYKalmanFilter() {}
    void KalmanFilterInit();
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
