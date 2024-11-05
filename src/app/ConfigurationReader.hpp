/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-05 14:46:32
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-05 15:54:54
 * @FilePath: /success2025/src/app/ConfigurationReader.hpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */

#ifndef __CONFIGURATIONREADER_H__
#define __CONFIGURATIONREADER_H__

#include <opencv2/opencv.hpp>
#include "../../utils/cout.h"
#include <vector>
#include <string>

class ConfigurationReader
{
public:
    std::string YamlPth;
    std::string verison;
    std::string ros_enable;
    std::string FPS_show;

    ConfigurationReader(std::string Pth)
        : YamlPth(Pth) {} // 读取配置文件地址
    ~ConfigurationReader();

    void ConfigurationRead()
    {
        cv::FileStorage fs_read(this->YamlPth, cv::FileStorage::READ);
        this->verison = static_cast<std::string>(fs_read["verison"]);
        this->ros_enable = static_cast<std::string>(fs_read["ros_enable"]);
        this->FPS_show = static_cast<std::string>(fs_read["FPS_show"]);

        std::cout << GREEN << "verison :" << this->verison << "\n"
                  << "ros_enable :" << this->ros_enable << "\n"
                  << "FPS_show :" << this->FPS_show
                  << WHITE << std::endl;
    }
};

#endif /* __CONFIGURATIONREADER_H__ */