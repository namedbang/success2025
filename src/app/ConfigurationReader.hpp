/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-05 14:46:32
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-12-06 01:48:34
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
#include <thread> // For sleep
#include <chrono> // For duration

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
    std::string Debug_FPS;
    std::string Debug_Kalman;
    std::string Debug_Can;
    int Debug_Kalman_AdvantceTime;
    std::vector<int> HSV_lowerb_red;
    std::vector<int> HSV_upperb_red;
    cv::Mat_<double> camera_matrix, distort_coefficient;
    cv::Mat_<double> R;
    cv::Mat_<double> Q;
    cv::Mat_<double> P;
    cv::Mat_<double> TVEC;

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
        this->Debug_FPS = static_cast<std::string>(fs_read["Debug_FPS"]);
        this->Debug_Kalman = static_cast<std::string>(fs_read["Debug_Kalman"]);
        this->Debug_Can = static_cast<std::string>(fs_read["Debug_Can"]);
        fs_read["HSV_lowerb_red"] >> this->HSV_lowerb_red;
        fs_read["HSV_upperb_red"] >> this->HSV_upperb_red;
        fs_read["cameraMatrix"] >> this->camera_matrix;
        fs_read["distCoeffs"] >> this->distort_coefficient;
        fs_read["R"] >> this->R;
        fs_read["Q"] >> this->Q;
        fs_read["P"] >> this->P;
        fs_read["TVEC"] >> this->TVEC;
        fs_read["Debug_Kalman_AdvantceTime"] >> this->Debug_Kalman_AdvantceTime;
        std::cout << GREEN << "verison :" << this->verison << "\n"
                  << "ros_enable :" << this->ros_enable << "\n"
                  << "enable_show :" << this->enable_show << "\n"
                  << "time_show :" << this->time_show << "\n"
                  << "camera_config_path :" << this->camera_config_path << "\n"
                  << "FPS_show :" << this->FPS_show
                  << YELLOW << "\n"
                  << "Debug_FPS :" << this->Debug_FPS << "\n"
                  << "Debug_Kalman :" << this->Debug_Kalman << "\n"
                  << "Debug_Kalman_AdvantceTime :" << this->Debug_Kalman_AdvantceTime << "\n"
                  << "Debug_Can :" << this->Debug_Can << "\n"
                  << WHITE << std::endl;
        // Sleep for 1 second
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
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