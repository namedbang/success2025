/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-05 14:46:32
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-05 15:15:20
 * @FilePath: /success2025/src/app/ConfigurationReader.hpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */

#ifndef __CONFIGURATIONREADER_H__
#define __CONFIGURATIONREADER_H__

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

class ConfigurationReader
{
private:
    std::string YamlPth;
    std::string verison;

public:
    ConfigurationReader(std::string Pth)
        : YamlPth(Pth) {}
    ~ConfigurationReader();

    void ConfigurationRead()
    {
        cv::FileStorage fs_read(this->YamlPth, cv::FileStorage::READ);
        this->verison = static_cast<std::string>(fs_read["verison"]);
        std::cout << this->verison << std::endl;
    }
};

ConfigurationReader::~ConfigurationReader()
{
}

#endif /* __CONFIGURATIONREADER_H__ */