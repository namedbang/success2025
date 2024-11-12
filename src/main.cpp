/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-09-24 13:56:59
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-12 20:59:58
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
#include <unistd.h>

int main(int argc, char *argv[])
{
    /*配置文件的读取 */
    bool imfom;
    ConfigurationReader *reader_p = new ConfigurationReader("../config.yaml");
    reader_p->ConfigurationRead();

    Picture *picture = new Picture(reader_p);                            // 创建视频管道
    BsaeCamera *BsaeCamera = new MindCamera_software(picture, reader_p); // 将Mind相机接入管道
    // chank = BsaeCamera->camera_chank();                               // debug时用
    process *process_p = new process_opencv_cuda(picture, reader_p); // 将视频数据传入处理类

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
    { /// 8ms

        BsaeCamera->camera_read_once(BsaeCamera->iCameraCounts); // 读图像数据
        picture->TimeBegin();
        process_p->processing(); // 处理图像数据
        picture->TimeEnd();
        picture->CalculateTime();
        picture->CvPutTextOnUI();
        if (true == picture->ImgShow())
            break;
        // usleep(1000);
    }
    delete BsaeCamera;
    delete picture;
    delete process_p;
    delete reader_p;
    return 0;
}
