/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2025-01-05 16:39:02
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2025-01-05 17:42:10
 * @FilePath: /success2025/src/hardware/gpio/GPIO.cpp
 * @Description:
 *
 * Copyright (c) 2025 by CDTU-Success, All Rights Reserved.
 */
#include "./GPIO.hpp"

using namespace GPIO; // optional

void Gpio::GpioInit()
{
    // Pin Setup.
    GPIO::setmode(GPIO::CVM);
    GPIO::setup(SwitchPort, GPIO::IN); // channel must be int or std::string
}
