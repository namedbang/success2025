/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-09-24 13:56:59
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2025-01-15 19:14:29
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
#include "./RTSP/RTSPStreamer.hpp"
#include "./hardware/gpio/GPIO.hpp"
#include "./yolo/yolov8.hpp"
extern "C"
{
#include "./hardware/uart/modbus.h"
}
#include <memory>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <atomic>
#include <jetson-utils/videoSource.h>
#include <jetson-utils/videoOutput.h>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudawarping.hpp>

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
videoOutput *output;
// YOLOv8 *yolov8;
Gpio gpio_h;
cv::Mat point_Kalman_image;
cv::Point KalmanPoint;
extern imu_angle_t imu_angle;
double yaw_last;
uint8_t reset;

// 互斥锁
std::mutex point_Kalman_image_mtx;
std::mutex displayImg_mtx;
extern std::mutex mtx_k;

class TimerForKalman : public CppTimer
{ //
    void timerEvent()
    {
        std::lock_guard<std::mutex> lock(mtx_k); // 加锁
        if (EnemyInform_p->T.empty() == 0 && EnemyInform_p->enemy_exist == 1)
        {
            bool flage = 0;
            if (abs(yaw_last - EnemyInform_p->yaw) >= 6)
            {
                reset = 0; // 低复位
                flage = 1;
            }
            double AdTime = Filter->computeADTime(Filter->v_projectile, EnemyInform_p->Zw);                  // 提前量的计算
            Eigen::VectorXd y = Filter->xyzV2Eigen(EnemyInform_p->Xw, EnemyInform_p->Yw, EnemyInform_p->Zw); // 打包

            // if (reader_p->Debug_Kalman == "true" && reader_p->Debug_Kalman_AdvantceTime != 0)
            // {
            Filter->KalmanUpdate(y, AdTime, reset); // 应用卡尔曼滤波
            // Filter->KalmanUpdate(y, AdTime, reset);
            reset = 1;
            yaw_last = EnemyInform_p->yaw;
            // }
            // else
            //     Filter->KalmanUpdate(y, AdTime, 1);               // 应用卡尔曼滤波
            cv::Mat point_camera = (cv::Mat_<double>(4, 1) <<     //
                                        Filter->kf->state()(0),   //
                                    Filter->kf->state()(1),       //
                                    Filter->kf->state()(2), 1.0); //
            // 应用齐次变换矩阵将点从世界坐标系转换到相机坐标系
            std::vector<cv::Point3f> objectPoints = {cv::Point3f(Filter->kf->state()(0), Filter->kf->state()(1), Filter->kf->state()(2))};
            {
                std::lock_guard<std::mutex> lock(point_Kalman_image_mtx); // 加锁保护对 point_image 的访问
                std::vector<cv::Point2f> imagePoints;
                cv::projectPoints(objectPoints, EnemyInform_p->rvec, EnemyInform_p->tvec, reader_p->camera_matrix, reader_p->distort_coefficient.t(), imagePoints);
                KalmanPoint = cv::Point(imagePoints[0].x, imagePoints[0].y);
            }
            cv::Mat point_world = EnemyInform_p->T * point_camera;
            auto Xc = point_world.at<double>(0, 0);
            auto Yc = point_world.at<double>(1, 0);
            auto Zc = point_world.at<double>(2, 0);
            EnemyInform_p->yaw_kalman = atan2(Xc, Zc) / M_PI * 180;
            EnemyInform_p->pitch_kalman = atan2(-Yc, sqrt(pow(Xc, 2) + pow(Zc, 2))) / M_PI * 180;
            /*debug-------------------------------------------------------------------------------- */
            if (reader_p->Debug_Kalman == "true")
            {
                char buffer[100];
                memset(buffer, 0, sizeof(buffer));
                sprintf(buffer, " :%f,%f,%f,%f,%f,%d,%d\n", EnemyInform_p->yaw_kalman.load(), EnemyInform_p->pitch_kalman.load(), EnemyInform_p->yaw, EnemyInform_p->pitch, AdTime, flage, EnemyInform_p->enemy_exist); // y(0), y(1), y(2) 分别是 x, y, z
                SerialPortWriteBuffer(Uart_inf.UID0, buffer, sizeof(buffer));
            }
            flage = 0;
            /*debug-------------------------------------------------------------------------------- */
        }
        else
        {
            reset = 0;
        }
        // modbus_write_registers_noreply(MB_STM32_YUNTAI_ID, MB_WRITE_AUTOMATIC_AIMING_REGISTERS, static_cast<float>(EnemyInform_p->yaw_kalman.load()), static_cast<float>(EnemyInform_p->pitch_kalman.load()), EnemyInform_p->enemy_exist);
        modbus_write_registers_noreply(MB_STM32_YUNTAI_ID, MB_WRITE_AUTOMATIC_AIMING_REGISTERS, static_cast<float>(EnemyInform_p->yaw), static_cast<float>(EnemyInform_p->pitch), EnemyInform_p->enemy_exist);
    }
};

class TimerFor6020 : public CppTimer
{ //
    void timerEvent()
    {
        motor_control->GM6020_read();
    }
};

class GpioReader : public CppTimer
{ //
    void timerEvent()
    {
        gpio_h.SwitchVal = gpio_h.gpio_read(gpio_h.SwitchPort);
        // std::cout << gpio_h.SwitchVal << std::endl;
    }
};

int main(int argc, char *argv[])
{
    /*打开IO------------------------------------------*/
    gpio_h.GpioInit();
    /*打开串口----------------------------------------*/
    initialize_serial_port_info(&Uart_inf); // debug串口
    int uart = SerialPortOpen(Uart_inf.Uart, 115200, SERIAL_PORT_PARITY_NONE, &Uart_inf.UID0);
    int uart2stm32 = SerialPortOpen(Uart_inf.Uart2STM32, 115200, SERIAL_PORT_PARITY_NONE, &Uart_inf.UID2STM32);
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
    reader_p = new ConfigurationReader("/home/gyxy/Desktop/workspeaseMY/success2025/config.yaml");
    reader_p->ConfigurationRead();
    /*相机视频初始化 */
    EnemyInform_p = new EnemyInform();              // 敌人信息初始化
    picture = new Picture(reader_p, EnemyInform_p); // 创建视频管道
    // BsaeCamera *BsaeCamera = new MindCamera_software(picture, reader_p); // 将Mind相机接入管道
    BsaeCamera *BsaeCamera = new MindCamera(picture, reader_p); // 将Mind相机接入管道
                                                                // chank = BsaeCamera->camera_chank();                               // debug时用
    /*rtsp*/ if (reader_p->Debug_RTSP == "true")
    {
        cv::cuda::setDevice(0);
        videoOptions options;
        options.frameRate = 60;
        // options.bitRate = 700000000;
        // options.latency = 0;
        std::string str = "rtsp://192.168.137.4:8554/live";
        output = videoOutput::Create(str.c_str(), argc, argv, 0, options);
    }

    /*kalman 初始化*/
    Filter = new MYKalmanFilter(reader_p, EnemyInform_p);
    Filter->KalmanFilterInit();

    /*yolo初始化*/
    // const string engine_file_path = "../mode/yolo.trt";
    // cout << "Set CUDA...\n"
    //      << endl;
    // cudaSetDevice(0);
    // cout << "Loading TensorRT model " << engine_file_path << endl;
    // cout << "\nWait a second...." << std::flush;
    // yolov8 = new YOLOv8(engine_file_path);
    // cout << "\rLoading the pipe... " << string(10, ' ') << "\n\r";
    // cout << endl;
    // yolov8->MakePipe(false);

    const std::vector<std::string> engine_files = {
        // "../mode/yolo.trt",
        // "../mode/yolo.trt",
        // "../mode/yolo.trt",
        // "../mode/yolo.trt",
        // "../mode/yoloInt8.trt",
        "../mode/yolo.trt"};
    std::vector<std::shared_ptr<YOLOv8>> yolo_models; // 使用智能指针管理 YOLOv8
    std::cout << "Set CUDA...\n";
    cudaSetDevice(0);
    for (const auto &engine_file : engine_files)
    {
        std::cout << "Loading TensorRT model " << engine_file << std::endl;
        auto yolo = std::make_shared<YOLOv8>(engine_file);
        yolo->MakePipe(false);
        yolo_models.push_back(yolo);
    }

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
    GM6020Timer.startns(1);
    GpioReader GpioReadThead;
    GpioReadThead.startms(500);
    ThreadPool pool(3);
    std::future<PROCESS_state> result; // 8ms ↓
    cv::Mat tempMat;
    std::vector<std::future<PROCESS_state>> results; // 存储任务结果
    while (true)
    {

        // char buffer[50];
        // memset(buffer, 0, sizeof(buffer));
        // sprintf(buffer, ":%f\n", imu_angle.yaw_imu.load());
        // SerialPortWriteBuffer(Uart_inf.UID0, buffer, sizeof(buffer));
        picture->TimeBegin();
        // result = pool.enqueue(
        //     [process_p]
        //     { return process_p->processing(); }); // 加入任务列表

        // BsaeCamera->camera_software_Trigger(); // 触发读图像
        // BsaeCamera->camera_read_once(BsaeCamera->iCameraCounts); // 读图像数据

        for (auto &yolo_p : yolo_models)
        {
            results.push_back(pool.enqueue(
                [&process_p, &yolo_p, &BsaeCamera, tempMat]
                {
                    auto temp_state = process_p->processing(yolo_p.get(), tempMat.clone());

                    return temp_state;
                }));
            tempMat = BsaeCamera->camera_read_once(BsaeCamera->iCameraCounts); // 读图像数据
        }
        for (auto &resul : results)
        {
            bool success = resul.get();
        }
        // results.front().get();
        // results.pop_front();
        // results.front().get();
        results.clear();
        // auto re = result.get(); // 处理图像数据//同步处理（测速时使用）
        picture->TimeEnd();
        picture->CalculateTime();
        picture->displayImage = picture->preImage.clone();
        picture->CvPutTextOnUI(yolo_models[0].get());

        /*rtsp*/ if (reader_p->Debug_RTSP == "true" && picture->displayImage.empty() == false)
        {
            cv::cuda::GpuMat G_displayImage;
            G_displayImage.upload(picture->displayImage);
            if (output != NULL)
            {
                cv::Size target_size(1000, 1024);
                cv::cuda::cvtColor(G_displayImage, G_displayImage, cv::COLOR_BGR2RGB);
                cv::cuda::resize(G_displayImage, G_displayImage, target_size);
                uchar3 *image_data = (uchar3 *)G_displayImage.ptr<uchar3>(0);
                output->Render(image_data, G_displayImage.rows, G_displayImage.cols);
                if (!output->IsStreaming())
                    ;
            }
            G_displayImage.download(picture->displayImage);
        }
        // if (true == picture->ImgShow())
        //     break;
        // usleep(1000);
    }
    SAFE_DELETE(output);
    KalmanTimer.stop();
    GM6020Timer.stop();
    GpioReadThead.stop();
    delete motor_control;
    delete BsaeCamera;
    delete picture;
    delete process_p;
    delete reader_p;
    return 0;
}
