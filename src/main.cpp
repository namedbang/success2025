/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-09-24 13:56:59
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-17 23:13:51
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
#include "./process/enemy_Inform.hpp"
#include "./app/ThreadPool.h"

#include <memory>
#include <unistd.h>
#include <chrono>

int main(int argc, char *argv[])
{
    /*配置文件的读取 */
    bool imfom;
    ConfigurationReader *reader_p = new ConfigurationReader("../config.yaml");
    reader_p->ConfigurationRead();

    Picture *picture = new Picture(reader_p);                            // 创建视频管道
    BsaeCamera *BsaeCamera = new MindCamera_software(picture, reader_p); // 将Mind相机接入管道
    // chank = BsaeCamera->camera_chank();                               // debug时用
    EnemyInform *EnemyInform_p = new EnemyInform();
    process *process_p = new process_opencv_cuda(picture, reader_p, EnemyInform_p); // 将视频数据传入处理类

    if (true == BsaeCamera->MYCameraInit())
    {
        std::cout << "相机初始化成功" << std::endl;
    }
    else
    {
        std::cout << "相机初始化失败" << std::endl;
        delete BsaeCamera;
        delete picture;
        delete process_p;
        delete reader_p;
        return 0;
    }
    /*线程池相关*/
    ThreadPool pool(6);
    std::vector<std::future<PROCESS_state>> results;

    while (true)
    { /// 8ms
        results.clear();
        for (int i = 0; i < 1; ++i)
        {
            results.emplace_back(
                pool.enqueue(
                    [process_p]
                    { return process_p->processing(); }));
        }
        picture->TimeBegin();
        BsaeCamera->camera_software_Trigger();
        for (auto &&result : results)
            auto re = result.get();
        BsaeCamera->camera_read_once(BsaeCamera->iCameraCounts); // 读图像数据
        // process_p->processing();                                 // 处理图像数据
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
