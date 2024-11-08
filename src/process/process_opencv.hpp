/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-08 09:55:47
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-08 17:56:25
 * @FilePath: /success2025/src/process/process_opencv.hpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
#ifndef __PROCESS_OPENCV_CUDA_H__
#define __PROCESS_OPENCV_CUDA_H__

#include "../app/api/picture.hpp"

enum PROCESS_state
{
    PROCESUNSUCCESS = 0,
    PROCESSUCCESS
};

class process
{
protected:
    Picture *Picture_p;

public:
    process(Picture *Picturep = nullptr) : Picture_p(Picturep) {}
    virtual ~process() {}
    virtual PROCESS_state processing() = 0;
};

class process_opencv_cuda : public process
{
private:
    /* data */
public:
    process_opencv_cuda(Picture *Picturep = nullptr) : process(Picturep) {}
    ~process_opencv_cuda() {};
    PROCESS_state processing() override;
};

#endif /* __PROCESS_OPENCV_CUDA_H__ */
