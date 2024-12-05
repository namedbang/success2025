/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-09-24 13:56:59
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-12-06 02:31:30
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
#include "./hardware/can/canbus.hpp"
#include "./hardware/can/can.hpp"

#include <memory>
#include <unistd.h>
#include <chrono>
#include <thread>

// using namespace Kalman;
using namespace std;
using namespace wlp;

MYKalmanFilter *Filter;
EnemyInform *EnemyInform_p;
Serial_Port_infom Uart_inf;
// canbus bus("can0");
ConfigurationReader *reader_p;
GM6020 *motor_control;
Picture *picture;
extern std::mutex mtx_k; // 互斥量，用于同步访问共享资源

class TimerForKalman : public CppTimer
{ //
    void timerEvent()
    {
        std::lock_guard<std::mutex> lock(mtx_k);                                                         // 加锁
        Eigen::VectorXd y = Filter->xyzV2Eigen(EnemyInform_p->Xw, EnemyInform_p->Yw, EnemyInform_p->Zw); // 打包
        if (reader_p->Debug_Kalman == "true" && reader_p->Debug_Kalman_AdvantceTime != 0)
            Filter->KalmanUpdate(y, reader_p->Debug_Kalman_AdvantceTime, 1);
        else
            Filter->KalmanUpdate(y, 0, 1);
        if (EnemyInform_p->T.empty() == 0 && EnemyInform_p->enemy_exist == 1)
        {
            cv::Mat point_camera = (cv::Mat_<double>(4, 1) <<     //
                                        Filter->kf->state()(0),   //
                                    Filter->kf->state()(1),       //
                                    Filter->kf->state()(2), 1.0); //
                                                                  // 应用齐次变换矩阵将点从世界坐标系转换到相机坐标系
            cv::Mat point_world = EnemyInform_p->T * point_camera;
            auto Xw = point_world.at<double>(0, 0);
            auto Yw = point_world.at<double>(1, 0);
            auto Zw = point_world.at<double>(2, 0);
            EnemyInform_p->yaw_kalman = atan2(Xw, Zw) / M_PI * 180;
            EnemyInform_p->pitch_kalman = atan2(-Yw, sqrt(pow(Xw, 2) + pow(Zw, 2))) / M_PI * 180;

            /*debug-------------------------------------------------------------------------------- */
            if (reader_p->Debug_Kalman == "true")
            {
                char buffer[100];
                memset(buffer, 0, sizeof(buffer));
                sprintf(buffer, " :%f,%f,%f,%f\n", EnemyInform_p->yaw_kalman, EnemyInform_p->pitch_kalman, EnemyInform_p->yaw, EnemyInform_p->pitch); // y(0), y(1), y(2) 分别是 x, y, z
                SerialPortWriteBuffer(Uart_inf.UID0, buffer, sizeof(buffer));
            }
            /*debug-------------------------------------------------------------------------------- */
        }
    }
};

class TimerFor6020 : public CppTimer
{ //
    void timerEvent()
    {
        motor_control->GM6020_read();
    }
};

int main(int argc, char *argv[])
{
    /*打开串口----------------------------------------*/
    initialize_serial_port_info(&Uart_inf); // debug串口
    int uart = SerialPortOpen(Uart_inf.Uart, 115200, SERIAL_PORT_PARITY_NONE, &Uart_inf.UID0);
    /*打开can----------------------------------------*/
    // 设置 CAN 接口名称
    char can_interface[] = "can0";
    // 设置电机数量和电机 ID
    int GM6020_number = 1;          // 电机数量
    uint16_t motor_ids[] = {0x205}; // 电机 ID（例如：0x201, 0x202, 0x203）
    motor_control = new GM6020(can_interface, GM6020_number, motor_ids[0]);
    motor_control->GM6020_init();
    /*配置文件的读取 */
    bool imfom;
    reader_p = new ConfigurationReader("../config.yaml");
    reader_p->ConfigurationRead();
    /*相机视频初始化 */
    EnemyInform_p = new EnemyInform();                                   // 敌人信息初始化
    picture = new Picture(reader_p, EnemyInform_p);                      // 创建视频管道
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
        delete motor_control;
        delete BsaeCamera;
        delete picture;
        delete process_p;
        delete reader_p;

        return 0;
    }

    /*线程相关*/
    TimerForKalman KalmanTimer;
    KalmanTimer.startms(Kalman_cycle);
    TimerFor6020 GM6020Timer;
    GM6020Timer.startms(1);
    // std::thread t1(timerEvent6020);
    // t1.detach();
    ThreadPool pool(2);

    while (true)
    { /// 8ms
        /*debug------------------------------------ */
        // unsigned char temp = 0x01;
        // SerialPortWriteByte(Uart_inf.UID0, temp);
        /*debug------------------------------------ */
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
        // if (true == picture->ImgShow())
        //     break;
        // usleep(1000);
    }
    KalmanTimer.stop();
    GM6020Timer.stop();
    delete motor_control;
    delete BsaeCamera;
    delete picture;
    delete process_p;
    delete reader_p;
    return 0;
}
