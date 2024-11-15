/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-05 14:46:32
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-15 20:27:34
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
    std::string enable_show; // 显示需要开启
    std::string FPS_show;
    std::string time_show;
    std::string camera_config_path;
    std::vector<int> HSV_lowerb_red;
    std::vector<int> HSV_upperb_red;
    cv::Mat_<double> camera_matrix, distort_coefficient;
    char *charArray;

    ConfigurationReader(std::string Pth)
        : YamlPth(Pth) {} // 读取配置文件地址
    ~ConfigurationReader()
    {
        delete[] charArray;
    }

    void ConfigurationRead()
    {
        cv::FileStorage fs_read(this->YamlPth, cv::FileStorage::READ);
        this->verison = static_cast<std::string>(fs_read["verison"]);
        this->ros_enable = static_cast<std::string>(fs_read["ros_enable"]);
        this->FPS_show = static_cast<std::string>(fs_read["FPS_show"]);
        this->enable_show = static_cast<std::string>(fs_read["enable_show"]);
        this->time_show = static_cast<std::string>(fs_read["time_show"]);
        this->camera_config_path = static_cast<std::string>(fs_read["camera_config_path"]);
        fs_read["HSV_lowerb_red"] >> this->HSV_lowerb_red;
        fs_read["HSV_upperb_red"] >> this->HSV_upperb_red;
        fs_read["cameraMatrix"] >> this->camera_matrix;
        fs_read["distCoeffs"] >> this->distort_coefficient;

        std::cout << GREEN << "verison :" << this->verison << "\n"
                  << "ros_enable :" << this->ros_enable << "\n"
                  << "enable_show :" << this->enable_show << "\n"
                  << "time_show :" << this->time_show << "\n"
                  << "time_show :" << this->camera_config_path << "\n"
                  << "FPS_show :" << this->FPS_show
                  << WHITE << std::endl;
    }
    char *stringToCharPointer(const std::string &str)
    {
        // 分配足够的内存来存储字符串和 null 终止符
        this->charArray = new char[str.length() + 1];
        // 使用 strcpy 复制内容
        std::strcpy(this->charArray, str.c_str());
        // 返回 char*
        return this->charArray;
    }
};

#endif /* __CONFIGURATIONREADER_H__ */