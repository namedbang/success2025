/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-09-24 13:56:59
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-12-01 06:47:19
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
MYKalmanFilter *Filter;
EnemyInform *EnemyInform_p;
Serial_Port_infom Uart_inf;
ConfigurationReader *reader_p;
extern std::mutex mtx_k; // 互斥量，用于同步访问共享资源

class TimerForKalman : public CppTimer
{ //
    void timerEvent()
    {
        // if (EnemyInform_p->enemy_exist == 1)
        // {
        std::lock_guard<std::mutex> lock(mtx_k); // 加锁
        Eigen::VectorXd y = Filter->xyzV2Eigen(EnemyInform_p->Xw, EnemyInform_p->Yw, EnemyInform_p->Zw);
        Filter->KalmanUpdate(y);
        Eigen::MatrixXd R_eigen(3, 3);
        std::copy(reader_p->R.ptr<double>(0), reader_p->R.ptr<double>(0) + reader_p->R.total(), R_eigen.data()); // R

        Eigen::Vector3d mindXYZ = R_eigen.inverse() * y;
        // Eigen::Vector3d mindXYZ = R_eigen. * y;
        EnemyInform_p->yaw_kalman = atan2(mindXYZ(0), mindXYZ(2)) / M_PI * 180;
        // std::cout << this->EnemyInform_p->yaw << std::endl;
        EnemyInform_p->pitch_kalman = atan2(-mindXYZ(1), sqrt(pow(mindXYZ(0), 2) + pow(mindXYZ(2), 2))) / M_PI * 180;

        /*debug------------------------------------ */
        char buffer[30];
        memset(buffer, 0, sizeof(buffer));
        // 打印 y 向量的 x, y, z 分量
        sprintf(buffer, " :%.2f, %.2f, %.2f\n", EnemyInform_p->yaw_kalman, EnemyInform_p->pitch_kalman, mindXYZ(2)); // y(0), y(1), y(2) 分别是 x, y, z
        SerialPortWriteBuffer(Uart_inf.UID0, buffer, sizeof(buffer));
        /*debug------------------------------------ */
        // }
    }
};

int main(int argc, char *argv[])
{
    /*打开串口*/
    initialize_serial_port_info(&Uart_inf);
    int uart = SerialPortOpen(Uart_inf.Uart, 115200, SERIAL_PORT_PARITY_NONE, &Uart_inf.UID0);

    /*配置文件的读取 */
    bool imfom;
    reader_p = new ConfigurationReader("../config.yaml");
    reader_p->ConfigurationRead();
    /*相机视频初始化 */
    EnemyInform_p = new EnemyInform();                                   // 敌人信息初始化
    Picture *picture = new Picture(reader_p, EnemyInform_p);             // 创建视频管道
    BsaeCamera *BsaeCamera = new MindCamera_software(picture, reader_p); // 将Mind相机接入管道
    // chank = BsaeCamera->camera_chank();                               // debug时用
    /*kalman 初始化*/
    Filter = new MYKalmanFilter(reader_p, EnemyInform_p);
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
    KalmanTimer.startms(10);
    /*线程池相关*/
    ThreadPool pool(2);

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
