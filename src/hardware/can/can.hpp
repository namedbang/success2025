/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-12-03 13:03:45
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-12-04 11:55:29
 * @FilePath: /success2025/src/hardware/can/can.hpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-12-03 13:03:45
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-12-04 11:10:54
 * @FilePath: /success2025/src/hardware/can/can.hpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
#ifndef __CAN_H__
#define __CAN_H__
#include <stdint.h>
#include <vector>
#include <string>
#include "./can.hpp"
#include "./canbus.hpp"

using namespace std;
using namespace wlp;

enum
{
    yaw = 0,
    pitch,
} yuntai;

#pragma pack(push, 1) // 指定 1 字节对齐
typedef struct
{
    uint16_t can_id;        // 电机ID
    int16_t set_voltage;    // 设定的电压值
    uint16_t rotor_angle;   // 机械角度
    int16_t rotor_speed;    // 转速
    int16_t torque_current; // 扭矩电流
    uint8_t temp;           // 温度
} GM6020_Typedef_Structure;
#pragma pack(pop) // 恢复默认对齐

class GM6020
{
private:
    vector<uint16_t> id;
    uint8_t len;
    bool req;
    canbus bus; // CAN 总线实例
public:
    vector<GM6020_Typedef_Structure *> GMList;
    // 模板构造函数
    template <typename... Args>
    GM6020(const char *can_interface, int GM6020_number, Args... ids);

    ~GM6020();
    void GM6020_init();
    void GM6020_read();
};

#endif /* __CAN_H__ */