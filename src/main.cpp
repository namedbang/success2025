/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-09-24 13:56:59
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-30 22:36:33
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
#include "./process/predict.hpp"
#include "./hardware/uart/Serial_Port.h"
#include "./utils/CppTimer.h"
#include "./process/predict.hpp"
#include "./utils/KalmanFilter/kalman.hpp"

#include <memory>
#include <unistd.h>
#include <chrono>

// using namespace Kalman;

class TimerForKalman : public CppTimer
{ //

    void timerEvent()
    {
        // std::cout << "1" << std::endl;
    }
};

Serial_Port_infom Uart_inf;
int main(int argc, char *argv[])
{
    /*打开串口*/
    initialize_serial_port_info(&Uart_inf);
    int uart = SerialPortOpen(Uart_inf.Uart, 115200, SERIAL_PORT_PARITY_NONE, &Uart_inf.UID0);

    /*配置文件的读取 */
    bool imfom;
    ConfigurationReader *reader_p = new ConfigurationReader("../config.yaml");
    reader_p->ConfigurationRead();
    /*相机视频初始化 */
    EnemyInform *EnemyInform_p = new EnemyInform();
    Picture *picture = new Picture(reader_p, EnemyInform_p);             // 创建视频管道
    BsaeCamera *BsaeCamera = new MindCamera_software(picture, reader_p); // 将Mind相机接入管道
    // chank = BsaeCamera->camera_chank();                               // debug时用
    /*kalman 初始化*/
    MYKalmanFilter *Filter = new MYKalmanFilter(reader_p, EnemyInform_p);
    Filter->KalmanFilterInit();

    process *process_p = new process_opencv_cuda(picture, reader_p, EnemyInform_p, Filter); // 将视频数据传入处理类

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
    TimerForKalman KalmanTimer;
    KalmanTimer.startms(1000);
    /*线程池相关*/
    ThreadPool pool(1);

    while (true)
    { /// 8ms
        // unsigned char temp = 0x01;
        // SerialPortWriteByte(Uart_inf.UID0, temp);

        auto result = pool.enqueue(
            [process_p]
            { return process_p->processing(); }); // 加入任务列表
        picture->TimeBegin();
        BsaeCamera->camera_software_Trigger();                   // 触发读图像
        auto re = result.get();                                  // 处理图像数据
        BsaeCamera->camera_read_once(BsaeCamera->iCameraCounts); // 读图像数据
        picture->TimeEnd();
        picture->CalculateTime();
        picture->displayImage = picture->preImage.clone();
        picture->CvPutTextOnUI();
        if (true == picture->ImgShow())
            break;
        // usleep(1000);
    }
    KalmanTimer.stop();
    delete BsaeCamera;
    delete picture;
    delete process_p;
    delete reader_p;
    return 0;
}
