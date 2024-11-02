/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-09-24 13:56:59
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-02 18:03:56
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
    Picture *picture = new Picture();
    BsaeCamera *BsaeCamera = new MindCamera(picture);
    BsaeCamera->camera_chank();
    delete BsaeCamera;
    delete picture;
    return 0;
}
