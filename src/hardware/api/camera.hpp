/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-09-25 11:13:03
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-17 22:22:28
 * @FilePath: /success2025/src/hardware/api/camera.hpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "../../app/api/picture.hpp"
#include "../../app/ConfigurationReader.hpp"
#include "CameraApi.h" //相机SDK的API头文件
#include <opencv2/opencv.hpp>
#include <vector>

class BsaeCamera
{
protected:
    Picture *Picture_p;
    ConfigurationReader *Reader_p;

public:
    int iStatus = -1;
    int iStatusDir = -1;
    int iCameraCounts = 1;

    BsaeCamera(Picture *Picture = nullptr, ConfigurationReader *Reader = nullptr)
        : Picture_p(Picture),
          Reader_p(Reader) {}
    virtual ~BsaeCamera() {}

    virtual bool MYCameraInit() = 0;
    virtual void camera_read_once(unsigned char camera_id) = 0;
    virtual bool camera_chank() = 0;
    virtual void camera_software_Trigger() {};
};

class MindCamera : public BsaeCamera
{
private:
    tSdkCameraDevInfo tCameraEnumList;
    int hCamera;
    tSdkCameraCapbility tCapability; // 设备描述信息
    tSdkFrameHead sFrameInfo;
    BYTE *pbyBuffer;

public:
    std::vector<unsigned char> camera_id_list;

    MindCamera(Picture *Picture = nullptr, ConfigurationReader *Reader = nullptr) : BsaeCamera(Picture, Reader) {}
    ~MindCamera();

    bool camera_chank() override;
    bool MYCameraInit() override;
    void camera_read_once(unsigned char camera_id) override;
    // void camera_id_read();
};

class MindCamera_software : public BsaeCamera
{
private:
    tSdkCameraDevInfo tCameraEnumList;
    int hCamera;
    tSdkCameraCapbility tCapability; // 设备描述信息
    tSdkFrameHead sFrameInfo;
    tSdkFrameHead FrameHead;
    BYTE *pbyBuffer;
    BYTE *pRawData;

public:
    std::vector<unsigned char> camera_id_list;

    MindCamera_software(Picture *Picture = nullptr, ConfigurationReader *Reader = nullptr) : BsaeCamera(Picture, Reader) {}
    ~MindCamera_software();

    bool camera_chank() override;
    bool MYCameraInit() override;
    void camera_read_once(unsigned char camera_id) override;
    void camera_software_Trigger();
    // void camera_id_read();
};

#endif