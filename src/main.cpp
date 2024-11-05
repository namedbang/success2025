/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-09-24 13:56:59
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-05 15:14:06
 * @FilePath: /success2025/src/main.cpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
#include "iostream"
#include "CameraApi.h"

#include "./hardware/api/camera.hpp"
#include "./app/api/picture.hpp"
#include "./app/ConfigurationReader.hpp"

int main(int argc, char *argv[])
{
    bool imfom;
    Picture *picture = new Picture();                 // 创建视频管道
    BsaeCamera *BsaeCamera = new MindCamera(picture); // 将相机接入管道
    // chank = BsaeCamera->camera_chank();               // debug时用
    ConfigurationReader Reader("../config.yaml");
    Reader.ConfigurationRead();
    if (true == BsaeCamera->MYCameraInit())
    {
        std::cout << "相机初始化成功" << std::endl;
    }
    else
    {
        std::cout << "相机初始化失败" << std::endl;
        return 0;
    }
    while (true)
    {
        BsaeCamera->camera_read_once(BsaeCamera->iCameraCounts);
        if (true == picture->ImgShow())
            break;
    }
    delete BsaeCamera;
    delete picture;
    return 0;
}
