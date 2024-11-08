/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-09-24 13:56:59
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-08 20:08:56
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
#include "./process/process_opencv.hpp"

#include <memory>

int main(int argc, char *argv[])
{
    /*配置文件的读取 */
    bool imfom;
    ConfigurationReader *reader_p = new ConfigurationReader("../config.yaml");

    Picture *picture = new Picture(reader_p);         // 创建视频管道
    BsaeCamera *BsaeCamera = new MindCamera(picture); // 将Mind相机接入管道
    // chank = BsaeCamera->camera_chank();            // debug时用
    process *process_p = new process_opencv_cuda(picture);

    reader_p->ConfigurationRead();
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
        picture->TimeBegin();
        BsaeCamera->camera_read_once(BsaeCamera->iCameraCounts);
        process_p->processing();
        picture->TimeEnd();
        picture->CalculateTime();
        picture->CvPutTextOnUI();
        if (true == picture->ImgShow())
            break;
    }
    delete BsaeCamera;
    delete picture;
    return 0;
}
