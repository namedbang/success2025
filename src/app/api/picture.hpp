/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-02 12:50:18
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-05 16:22:03
 * @FilePath: /success2025/src/app/api/picture.hpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
#ifndef PICTURE_HPP
#define PICTURE_HPP

#include <opencv2/imgproc/types_c.h>
#include "opencv2/highgui/highgui.hpp"
#include "../ConfigurationReader.hpp"

class Picture
{
private:
    ConfigurationReader *Config;
    double T1; // to FPS
    double T2;
    double spendTime;

public:
    cv::Mat preImage;
    char ImgShow() // in while
    {
        cv::imshow("success2025", preImage);
        return cv::waitKey(1) == 'q';
    }
    void CalculateTime()
    {
        this->spendTime = (this->T2 - this->T1) * 1000 / (cv::getTickFrequency());
    }
    void TimeBegin()
    {
        T1 = cv::getTickCount();
    }
    void TimeEnd()
    {
        T2 = cv::getTickCount();
    }
    void CvShowOnUI()
    {
        if ((Config->enable_show == "true") && (preImage.empty()))
        {
            std::ostringstream oss;
            if (Config->FPS_show == "true")
                oss << "FPS:  " << 1 / spendTime << '\n';
            if (Config->time_show == "true")
                oss << spendTime << "  ms" << '\n';
        }
    }
    Picture(ConfigurationReader *Config_p)
        : Config(Config_p) {}
    ~Picture() = default;
};

#endif // PICTURE_HPP
