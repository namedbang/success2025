/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-12-15 14:50:49
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2025-01-12 20:22:55
 * @FilePath: /success2025/src/hardware/uart/modbus.c
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
#include "./modbus.h"
#include <stdint.h>
#include "./Serial_Port.h"
#include <stdio.h>

extern Serial_Port_infom Uart_inf;
/*
 * @name CRC_Check
 * @brief CRC校验
 * @param CRC_Ptr->数组指针，LEN->长度
 * @retval CRC校验值
 */
uint16_t CRC_Check(uint8_t *CRC_Ptr, uint8_t LEN)
{
    uint16_t CRC_Value = 0;
    uint8_t i = 0;
    uint8_t j = 0;
    CRC_Value = 0xffff;
    for (i = 0; i < LEN; i++)
    {
        CRC_Value ^= *(CRC_Ptr + i);
        for (j = 0; j < 8; j++)
        {
            if (CRC_Value & 0x00001)
                CRC_Value = (CRC_Value >> 1) ^ 0xA001;
            else
                CRC_Value = (CRC_Value >> 1);
        }
    }
    CRC_Value = ((CRC_Value >> 8) + (CRC_Value << 8)); // 交换高低字节
    return CRC_Value;
}

int modbus_write_registers_noreply(unsigned char slave_id, unsigned char function_code, float yaw, float pitch, bool enemy)
{
    switch (function_code)
    {
    case MB_WRITE_AUTOMATIC_AIMING_REGISTERS:
    {
        // uint32_t yaw_int, pitch_int;
        // memcpy(&yaw_int, &yaw, sizeof(yaw));
        // memcpy(&pitch_int, &pitch, sizeof(pitch));
        YunTai YunTai2stm32;
        YunTai2stm32.stm32_id = MB_STM32_YUNTAI_ID;
        YunTai2stm32.function_code = MB_WRITE_AUTOMATIC_AIMING_REGISTERS;
        YunTai2stm32.enemy = enemy;
        // YunTai2stm32.yaw_h = (yaw_int >> 16) & 0xFFFF;
        // YunTai2stm32.yaw_l = yaw_int & 0xFFFF;
        // YunTai2stm32.pitch_h = (pitch_int >> 16) & 0xFFFF;
        // YunTai2stm32.pitch_l = pitch_int & 0xFFFF;
        YunTai2stm32.yaw = yaw;
        YunTai2stm32.pitch = pitch;
        YunTai2stm32.CRC = CRC_Check((uint8_t *)&YunTai2stm32, 11);
        SerialPortWriteBuffer(Uart_inf.UID2STM32, &YunTai2stm32, sizeof(YunTai2stm32));
        break;
    }
    default:
    {
        printf("没有实现发送格式\n");
        break;
    }
    }
    return MB_OK;
}
