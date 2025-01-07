/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-12-03 13:03:45
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2025-01-06 17:53:21
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
#include <atomic>

typedef float fp32;

#define M_PI 3.14159265358979323846

#define RM_IMU_QUAT_ID 0x401
#define RM_IMU_GYRO_ID 0x402
#define RM_IMU_ACCEL_ID 0x403
#define RM_IMU_MAG_ID 0x404
#define RM_IMU_PARAM_ID 0x405

#define ACCEL_3G_SEN 0.0047809375000f
#define ACCEL_6G_SEN 0.00179443359375f
#define ACCEL_12G_SEN 0.0035888671875f
#define ACCEL_24G_SEN 0.007177734375f

// 转换成 rad/s
#define GYRO_2000_SEN 0.00106526443603169529841533860381f
#define GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define GYRO_500_SEN 0.00026631610900792382460383465095346f
#define GYRO_250_SEN 0.00013315805450396191230191732547673f
#define GYRO_125_SEN 0.000066579027251980956150958662738366f

typedef struct
{
    uint8_t quat_euler : 1;
    uint8_t gyro_rangle : 3;
    uint8_t accel_rangle : 2;
    uint8_t imu_sensor_rotation : 5;
    uint8_t ahrs_rotation_sequence : 3;
    int16_t quat[4];
    fp32 quat_fp32[4];
    int16_t euler_angle[3];
    fp32 euler_angle_fp32[3];
    int16_t gyro_int16[3];
    int16_t accel_int16[3];
    int16_t mag_int16[3];
    fp32 gyro_fp32[3];
    fp32 accel_fp32[3];
    uint16_t sensor_time;
    uint16_t sensor_temperature;
    int16_t sensor_control_temperature;
    fp32 gyro_sen;
    fp32 accel_sen;
} rm_imu_data_t;

typedef struct
{
    std::atomic<double> yaw_imu;
    std::atomic<double> Pitch_imu;
    std::atomic<double> roll_imu;
} imu_angle_t;

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