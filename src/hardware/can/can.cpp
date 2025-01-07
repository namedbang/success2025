/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-12-03 13:15:10
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2025-01-07 15:21:17
 * @FilePath: /success2025/src/hardware/can/can.cpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
#include <iostream>
#include "./can.hpp"
#include <string.h>
#include <memory>
#include "../uart/Serial_Port.h"

rm_imu_data_t rm_imu_data;
imu_angle_t imu_angle;

/**
 * @description: 先yaw后pitch
 * @return {*}
 */
template <typename... Args>
GM6020::GM6020(const char *can_interface, int GM6020_number, Args... ids)
    : bus(can_interface) // 初始化 CAN 总线实例
{
    // 使用参数包初始化 id 向量
    id = {static_cast<uint16_t>(ids)...};

    // 检查 ID 数量是否与电机数量匹配
    if (id.size() != GM6020_number)
    {
        throw invalid_argument("ID 数量与电机数量不匹配");
    }

    // 根据电机数量创建电机对象并添加到列表
    for (int i = 0; i < GM6020_number; i++)
    {
        GM6020_Typedef_Structure *motor = new GM6020_Typedef_Structure();
        motor->can_id = id[i];   // 设置电机 ID
        GMList.push_back(motor); // 添加到列表
    }
}
// 显式实例化模板
// 显式实例化模板构造函数

GM6020::~GM6020()
{
    for (auto motor : GMList)
    {
        delete motor; // 释放动态分配的内存
    }
    GMList.clear(); // 清空列表
}

struct RxMessage
{
    uint8_t Data[8]; // 8 字节数据
    uint8_t DLC;
};
// RxMessage *rx_message;

void GM6020::GM6020_read()
{
    uint8_t rx_data[8]; // 接收数据缓冲区，最大8字节
    std::unique_ptr<RxMessage> rx_message = std::make_unique<RxMessage>();
    uint32_t id_R;
    if (bus.recv(&id_R, rx_data, &len, &req))
    {
        memcpy(rx_message->Data, rx_data, 8);
        rx_message->DLC = len;
        switch (id_R)
        {
        case 0x205:
        {
            GMList[0]->rotor_angle = ((rx_data[0] << 8) | rx_data[1]) * 180 / 8192; // 电机转子角度
            // GMList[0]->rotor_angle = ((rx_data[0] << 8) | rx_data[1]);    // 电机转子角度
            GMList[0]->rotor_speed = ((rx_data[2] << 8) | rx_data[3]);    // 电机转子速度
            GMList[0]->torque_current = ((rx_data[4] << 8) | rx_data[5]); // 电机扭矩电流
            GMList[0]->temp = rx_data[6];                                 // 电机温度
        }
        case RM_IMU_PARAM_ID:
        {
            rm_imu_data.accel_rangle = rx_message->Data[0] & 0x0F;
            rm_imu_data.gyro_rangle = (rx_message->Data[0] & 0xF0) >> 4;
            rm_imu_data.sensor_control_temperature = rx_message->Data[2];
            rm_imu_data.imu_sensor_rotation = rx_message->Data[3] & 0x1F;
            rm_imu_data.ahrs_rotation_sequence = (rx_message->Data[3] & 0xE0) >> 5;
            rm_imu_data.quat_euler = rx_message->Data[4] & 0x01;
            switch (rm_imu_data.gyro_rangle)
            {
            case 0:
                rm_imu_data.gyro_sen = GYRO_2000_SEN;
                break;
            case 1:
                rm_imu_data.gyro_sen = GYRO_1000_SEN;
                break;
            case 2:
                rm_imu_data.gyro_sen = GYRO_500_SEN;
                break;
            case 3:
                rm_imu_data.gyro_sen = GYRO_250_SEN;
                break;
            case 4:
                rm_imu_data.gyro_sen = GYRO_125_SEN;
                break;
            }
            switch (rm_imu_data.accel_rangle)
            {
            case 0:
                rm_imu_data.accel_sen = ACCEL_3G_SEN;
                break;
            case 1:
                rm_imu_data.accel_sen = ACCEL_6G_SEN;
                break;
            case 2:
                rm_imu_data.accel_sen = ACCEL_12G_SEN;
                break;
            case 3:
                rm_imu_data.accel_sen = ACCEL_24G_SEN;
                break;
            }
            break;
        }
        case RM_IMU_QUAT_ID:
        {
            // if (rm_imu_data.quat_euler && rx_message->DLC == 6)
            if (rx_message->DLC == 6)
            {
                memcpy(rm_imu_data.euler_angle, rx_message->Data, rx_message->DLC);
                rm_imu_data.euler_angle_fp32[0] = rm_imu_data.euler_angle[0] * 0.0001f;
                rm_imu_data.euler_angle_fp32[1] = rm_imu_data.euler_angle[1] * 0.0001f;
                rm_imu_data.euler_angle_fp32[2] = rm_imu_data.euler_angle[2] * 0.0001f;

                imu_angle.yaw_imu = rm_imu_data.euler_angle_fp32[0] * 180.0 / M_PI;
                imu_angle.Pitch_imu = rm_imu_data.euler_angle_fp32[1] * 180.0 / M_PI;
                // imu_angle.roll_imu = rm_imu_data.euler_angle_fp32[3] * 180.0 / M_PI;
            }
            else if (rm_imu_data.quat_euler == 0 && rx_message->DLC == 8)
            {
                memcpy(rm_imu_data.quat, rx_message->Data, rx_message->DLC);
                rm_imu_data.quat_fp32[0] = rm_imu_data.quat[0] * 0.0001f;
                rm_imu_data.quat_fp32[1] = rm_imu_data.quat[1] * 0.0001f;
                rm_imu_data.quat_fp32[2] = rm_imu_data.quat[2] * 0.0001f;
                rm_imu_data.quat_fp32[3] = rm_imu_data.quat[3] * 0.0001f;
            }
            break;
        }
        case RM_IMU_GYRO_ID:
        {
            memcpy(rm_imu_data.gyro_int16, rx_message->Data, 6);
            rm_imu_data.gyro_fp32[0] = rm_imu_data.gyro_int16[0] * rm_imu_data.gyro_sen;
            rm_imu_data.gyro_fp32[1] = rm_imu_data.gyro_int16[1] * rm_imu_data.gyro_sen;
            rm_imu_data.gyro_fp32[2] = rm_imu_data.gyro_int16[2] * rm_imu_data.gyro_sen;
            rm_imu_data.sensor_temperature = (int16_t)((rx_message->Data[6] << 3) | (rx_message->Data[7] >>
                                                                                     5));
            if (rm_imu_data.sensor_temperature > 1023)
            {
                rm_imu_data.sensor_temperature -= 2048;
            }
            break;
        }
        case RM_IMU_ACCEL_ID:
        {
            memcpy(rm_imu_data.accel_int16, rx_message->Data, 6);
            rm_imu_data.accel_fp32[0] = rm_imu_data.accel_int16[0] * rm_imu_data.accel_sen;
            rm_imu_data.accel_fp32[1] = rm_imu_data.accel_int16[1] * rm_imu_data.accel_sen;
            rm_imu_data.accel_fp32[2] = rm_imu_data.accel_int16[2] * rm_imu_data.accel_sen;
            memcpy(&rm_imu_data.sensor_time, (rx_message->Data + 6), 2);
            break;
        }
        case RM_IMU_MAG_ID:
        {
            memcpy(rm_imu_data.mag_int16, rx_message->Data, 6);
            break;
        }
        /*debug------------------------------------ */
        // char buffer[50];
        // memset(buffer, 0, sizeof(buffer));
        // sprintf(buffer, " :%d\n", GMList[0]->rotor_angle);
        // SerialPortWriteBuffer(Uart_inf.UID0, buffer, sizeof(buffer));
        /*debug------------------------------------ */
        break;
        }
    }
}

void GM6020::GM6020_init()
{
    if (bus.begin())
    {
        std::cout << "can打开成功" << std::endl;
    }
    else
    {
        std::cout << "can打开失败" << std::endl;
    }
}
template GM6020::GM6020(const char *can_interface, int GM6020_number, uint16_t);
