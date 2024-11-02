/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-09-25 11:13:03
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-02 17:56:30
 * @FilePath: /success2025/src/hardware/api/camera.hpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "../../app/api/picture.hpp"
#include <opencv2/opencv.hpp>
#include <vector>

class BsaeCamera
{
protected:
    Picture *Picture_p;
    std::vector<unsigned char> camera_id_list;

public:
    BsaeCamera(Picture *Picture = nullptr) {}
    virtual ~BsaeCamera() {}

    virtual bool MYCameraInit() = 0;
    virtual cv::Mat camera_read_once(unsigned char camera_id) = 0;
    virtual bool camera_chank() = 0;
};

class MindCamera : public BsaeCamera
{
private:
    /* data */
public:
    MindCamera(Picture *Picture = nullptr) : BsaeCamera(Picture) {}
    ~MindCamera();

    bool camera_chank() override;
    bool MYCameraInit() override;
    cv::Mat camera_read_once(unsigned char camera_id) override;
    // void camera_id_read();
};

#endif