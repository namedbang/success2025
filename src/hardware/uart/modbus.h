/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-12-15 14:50:49
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2025-01-12 20:22:47
 * @FilePath: /success2025/src/hardware/uart/modbus.h
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
#ifndef _MODBUS_H
#define _MODBUS_H

#include <termios.h>
#include <unistd.h>
#include <stdbool.h>

unsigned short CRC16(unsigned char *puchMsg, unsigned short usDataLen);
int modbus_write_registers_noreply(unsigned char slave_id, unsigned char function_code, float yaw, float pitch, bool enemy);
#define MB_STM32_YUNTAI_ID 0x01
#define MB_WRITE_AUTOMATIC_AIMING_REGISTERS 0x16
#define MB_WRITE_ 0x10
#define MB_OK 0
#define MB_CRC_ERROR -1
#define MB_EXCEPTION -2
#define MB_ERROR -3
#define MB_TIMEOUT -4

#pragma pack(push, 1) // 指定 1 字节对齐
typedef struct
{
   __uint8_t stm32_id;
   __uint8_t function_code;
   bool enemy;
   // __uint16_t yaw_h;
   // __uint16_t yaw_l;
   // __uint16_t pitch_h;
   // __uint16_t pitch_l;
   float yaw;
   float pitch;
   __uint16_t CRC;
} YunTai;
#pragma pack(pop) // 恢复默认对齐

#endif
