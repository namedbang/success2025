/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-08 09:55:47
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2025-01-05 19:57:36
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
#include "./predict.hpp"
#include "Eigen/Dense"

/*小装甲板*/
// #define HALF_LENGTH_LENGHT 134 / 2
// #define HALF_LENGTH_WIDTH 60 / 2
/*大装甲板*/
#define HALF_LENGTH_LENGHT 221 / 2
#define HALF_LENGTH_WIDTH 56 / 2

constexpr int max_size = 2;
constexpr float Pi = 3.1415;
// 自定义的物体世界坐标，单位为mm
using namespace cv;

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
    MYKalmanFilter *Filter_p;

    PROCESS_state getEuler();
    cv::Mat CoordinateSystemChange(cv::Mat xyz);

public:
    list<TimestampNode> timestamps;

    bool checkAngle(const RotatedRect &rrect);
    bool checkArea(double area);
    bool checkAspectRatio(double ratio);
    bool isValidLightBarBlob(const RotatedRect &rrect);
    void MeasureSpeed(const double x, const double y, const double z);

    process(Picture *Picturep = nullptr, ConfigurationReader *Readerp = nullptr, EnemyInform *EnemyInform_P = nullptr, MYKalmanFilter *Filterp = nullptr)
        : Picture_p(Picturep), Reader(Readerp), EnemyInform_p(EnemyInform_P), Filter_p(Filterp) {}
    virtual ~process() {}
    virtual PROCESS_state processing() = 0;
};

class process_opencv_cuda : public process
{
private:
    /* data */
    cv::Scalar *lowerFilter;
    cv::Scalar *higherFilter;
    cv::Scalar *lowerFilter_blue;
    cv::Scalar *higherFilter_blue;

public:
    process_opencv_cuda(Picture *Picturep = nullptr, ConfigurationReader *Readerp = nullptr, EnemyInform *EnemyInform_P = nullptr, MYKalmanFilter *Filterp = nullptr)
        : process(Picturep, Readerp, EnemyInform_P, Filterp)
    { // hsv阈值参数读入
        this->lowerFilter = new cv::Scalar(this->Reader->HSV_lowerb_red[0], this->Reader->HSV_lowerb_red[1], this->Reader->HSV_lowerb_red[2]);
        this->higherFilter = new cv::Scalar(this->Reader->HSV_upperb_red[0], this->Reader->HSV_upperb_red[1], this->Reader->HSV_upperb_red[2]);

        this->lowerFilter_blue = new cv::Scalar(this->Reader->HSV_lowerb_blue[0], this->Reader->HSV_lowerb_blue[1], this->Reader->HSV_lowerb_blue[2]);
        this->higherFilter_blue = new cv::Scalar(this->Reader->HSV_upperb_blue[0], this->Reader->HSV_upperb_blue[1], this->Reader->HSV_upperb_blue[2]);
    }
    ~process_opencv_cuda()
    {
        delete this->lowerFilter;
        delete this->higherFilter;
        delete this->lowerFilter_blue;
        delete this->higherFilter_blue;
    };
    PROCESS_state processing() override;
};

#endif /* __PROCESS_OPENCV_CUDA_H__ */
