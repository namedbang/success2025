/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-08 09:55:47
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-28 20:12:25
 * @FilePath: /success2025/src/process/process_opencv.hpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
#ifndef __PROCESS_OPENCV_CUDA_H__
#define __PROCESS_OPENCV_CUDA_H__

#include "../app/api/picture.hpp"
#include "../app/ConfigurationReader.hpp"
#include "./enemy_Inform.hpp"
/*小装甲板*/
// #define HALF_LENGTH_LENGHT 134 / 2
// #define HALF_LENGTH_WIDTH 60 / 2
/*大装甲板*/
#define HALF_LENGTH_LENGHT 226 / 2
#define HALF_LENGTH_WIDTH 60 / 2

constexpr float Pi = 3.1415;
// 自定义的物体世界坐标，单位为mm

enum PROCESS_state
{
    PROCESUNSUCCESS = 0,
    PROCESSUCCESS
};

class process
{
protected:
    Picture *Picture_p;
    ConfigurationReader *Reader;
    EnemyInform *EnemyInform_p;
    PROCESS_state getEuler();
    cv::Mat CoordinateSystemChange(cv::Mat xyz);

public:
    process(Picture *Picturep = nullptr, ConfigurationReader *Readerp = nullptr, EnemyInform *EnemyInform_P = nullptr)
        : Picture_p(Picturep), Reader(Readerp), EnemyInform_p(EnemyInform_P) {}
    virtual ~process() {}
    virtual PROCESS_state processing() = 0;
};

class process_opencv_cuda : public process
{
private:
    /* data */
    cv::Scalar *lowerFilter;
    cv::Scalar *higherFilter;

public:
    process_opencv_cuda(Picture *Picturep = nullptr, ConfigurationReader *Readerp = nullptr, EnemyInform *EnemyInform_P = nullptr)
        : process(Picturep, Readerp, EnemyInform_P)
    { // hsv阈值参数读入
        this->lowerFilter = new cv::Scalar(this->Reader->HSV_lowerb_red[0], this->Reader->HSV_lowerb_red[1], this->Reader->HSV_lowerb_red[2]);
        this->higherFilter = new cv::Scalar(this->Reader->HSV_upperb_red[0], this->Reader->HSV_upperb_red[1], this->Reader->HSV_upperb_red[2]);
    }
    ~process_opencv_cuda() {};
    PROCESS_state processing() override;
};

#endif /* __PROCESS_OPENCV_CUDA_H__ */
