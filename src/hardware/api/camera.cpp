/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-09-25 11:36:52
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2025-01-12 18:02:40
 * @FilePath: /success2025/src/hardware/api/camera.cpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */

#include "camera.hpp"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <stdio.h>

using namespace cv;

// IplImage *iplImage = NULL;
// int channel = 3;
std::mutex perImg_mtx;
unsigned char *g_pRgbBuffer; // 处理后数据缓存区

bool MindCamera::MYCameraInit()
{
    CameraSdkInit(1);

    // 枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
    printf("设备状态 Status = %d\n", iStatus);
    printf("设备数目 count  = %d\n", iCameraCounts);
    // 没有连接设备
    if (iCameraCounts == 0)
    {
        return false;
    }
    // CameraSetSysOption("NumBuffers", "20");

    // 相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);
    iStatusDir = CameraReadParameterFromFile(hCamera, this->Reader_p->stringToCharPointer(this->Reader_p->camera_config_path));
    printf("是否导入cfg  = %d\n", iStatusDir);
    // 初始化失败
    printf("初始化状态 = %d\n", iStatus);
    if (iStatus != CAMERA_STATUS_SUCCESS)
    {
        return false;
    }

    // 获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera, &tCapability);

    g_pRgbBuffer = (unsigned char *)malloc(tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3);

    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */
    CameraPlay(hCamera);

    if (tCapability.sIspCapacity.bMonoSensor)
    {
        // channel = 1;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
    }
    else
    {
        // channel = 3;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
    }

    return true;
}

cv::Mat MindCamera::camera_read_once(unsigned char camera_id)
{
    try // 相机读取帧
    {
        if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS)
        {
            // 最大等到1000
            // {
            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);
            {
                std::lock_guard<std::mutex> lock(perImg_mtx);
                this->Picture_p->preImage = cv::Mat(
                    cv::Size(sFrameInfo.iWidth, sFrameInfo.iHeight),
                    sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                    g_pRgbBuffer);
            }

            // imshow("Opencv Demo", matImage);
            // waitKey(0);
            // 在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
            // 否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
            CameraReleaseImageBuffer(hCamera, pbyBuffer);
            // }
        }
        else
        {
            throw "相机读取帧失败";
        }
    }
    catch (const char *e)
    {
        std::cerr << e << '\n';
    }
    return this->Picture_p->preImage;
}

#include <iostream>

bool MindCamera::camera_chank()
{
    bool inf;
    inf = MYCameraInit();
    if (inf)
        std::cout << "相机自检失败" << std::endl;
    else
        std::cout << "相机自检成功" << std::endl;
    camera_read_once(this->iCameraCounts);
    return inf;
}

MindCamera::~MindCamera()
{
    std::cout << "free Camera" << std::endl;
    CameraUnInit(hCamera);
    // 注意，现反初始化后再free
    free(g_pRgbBuffer);
}

bool MindCamera_software::MYCameraInit()
{
    CameraSdkInit(1);

    // 枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
    printf("设备状态 Status = %d\n", iStatus);
    printf("设备数目 count  = %d\n", iCameraCounts);
    // 没有连接设备
    if (iCameraCounts == 0)
    {
        return false;
    }
    // CameraSetSysOption("NumBuffers", "20");

    // 相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);
    iStatusDir = CameraReadParameterFromFile(hCamera, this->Reader_p->stringToCharPointer(this->Reader_p->camera_config_path));
    printf("是否导入cfg  = %d\n", iStatusDir);
    // 初始化失败
    printf("初始化状态 = %d\n", iStatus);
    if (iStatus != CAMERA_STATUS_SUCCESS)
    {
        return false;
    }

    // 获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera, &tCapability);

    g_pRgbBuffer = (unsigned char *)malloc(tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3);

    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */
    CameraPlay(hCamera);

    if (tCapability.sIspCapacity.bMonoSensor)
    {
        // channel = 1;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
    }
    else
    {
        // channel = 3;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
    }

    return true;
}

cv::Mat MindCamera_software::camera_read_once(unsigned char camera_id)
{
    try // 相机读取帧
    {
        if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS)
        {
            // 最大等到1000
            // {
            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);

            this->Picture_p->preImage = cv::Mat(
                cv::Size(sFrameInfo.iWidth, sFrameInfo.iHeight),
                sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                g_pRgbBuffer);

            // imshow("Opencv Demo", matImage);
            // waitKey(0);
            // 在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
            // 否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
            CameraReleaseImageBuffer(hCamera, pbyBuffer);
            // }
        }
        else
        {
            throw "相机读取帧失败";
        }
    }
    catch (const char *e)
    {
        std::cerr << e << '\n';
    }
    return this->Picture_p->preImage;
}

void MindCamera_software::camera_software_Trigger()
{
    CameraClearBuffer(hCamera);
    CameraSoftTrigger(hCamera);
}

bool MindCamera_software::camera_chank()
{
    bool inf;
    inf = MYCameraInit();
    if (inf)
        std::cout << "相机自检失败" << std::endl;
    else
        std::cout << "相机自检成功" << std::endl;
    camera_read_once(this->iCameraCounts);
    return inf;
}

MindCamera_software::~MindCamera_software()
{
    std::cout << "free Camera" << std::endl;
    CameraUnInit(hCamera);
    // 注意，现反初始化后再free
    free(g_pRgbBuffer);
}