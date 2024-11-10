/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-08 09:55:47
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-10 17:38:00
 * @FilePath: /success2025/src/process/process_opencv.hpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
#ifndef __PROCESS_OPENCV_CUDA_H__
#define __PROCESS_OPENCV_CUDA_H__

#include "../app/api/picture.hpp"
#include "../app/ConfigurationReader.hpp"

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

public:
    process(Picture *Picturep = nullptr, ConfigurationReader *Readerp = nullptr) : Picture_p(Picturep), Reader(Readerp) {}
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
    process_opencv_cuda(Picture *Picturep = nullptr, ConfigurationReader *Readerp = nullptr) : process(Picturep, Readerp)
    {
        this->lowerFilter = new cv::Scalar(this->Reader->HSV_lowerb[0], this->Reader->HSV_lowerb[1], this->Reader->HSV_lowerb[2]);
        this->higherFilter = new cv::Scalar(this->Reader->HSV_upperb[0], this->Reader->HSV_upperb[1], this->Reader->HSV_upperb[2]);
    }
    ~process_opencv_cuda() {};
    PROCESS_state processing() override;
};

#endif /* __PROCESS_OPENCV_CUDA_H__ */
