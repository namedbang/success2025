/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-09-24 13:56:59
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-03 18:01:40
 * @FilePath: /success2025/src/main.cpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "iostream"
#include "CameraApi.h"

#include "./hardware/api/camera.hpp"
#include "./app/api/picture.hpp"

int main(int argc, char *argv[])
{
    bool chank;
    Picture *picture = new Picture();                 // 创建视频管道
    BsaeCamera *BsaeCamera = new MindCamera(picture); // 将相机接入管道
    chank = BsaeCamera->camera_chank();
    delete BsaeCamera;
    delete picture;
    return 0;
}
