/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-12-03 13:15:10
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-12-04 11:55:27
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

void GM6020::GM6020_read()
{
    uint8_t rx_data[8]; // 接收数据缓冲区，最大8字节
    uint32_t id_R;
    if (bus.recv(&id_R, rx_data, &len, &req))
    {
        switch (id_R)
        {
        case 0x205:
            GMList[0]->rotor_angle = ((rx_data[0] << 8) | rx_data[1]) * 180 / 8192; // 电机转子角度
            GMList[0]->rotor_speed = ((rx_data[2] << 8) | rx_data[3]);              // 电机转子速度
            GMList[0]->torque_current = ((rx_data[4] << 8) | rx_data[5]);           // 电机扭矩电流
            GMList[0]->temp = rx_data[6];                                           // 电机温度

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
