/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-02 12:50:18
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-09 16:17:37
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
    cv::Mat endImage;
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
    void CvPutTextOnUI()
    {
        if (static_cast<int>(1000 / spendTime) < 80)
            std::cout << "low_fps_warn  " << static_cast<int>(spendTime) << std::endl;
        if ((Config->enable_show == "true") && !(preImage.empty()))
        {
            std::ostringstream oss;
            if (Config->FPS_show == "true")
                oss << "FPS :  " << static_cast<int>(1000 / spendTime) << "   ";
            if (Config->time_show == "true")
                oss << "Time :  " << static_cast<int>(spendTime) << " ms" << "   ";
            std::string result = oss.str();
            // 定义文本的位置（图像左下角的坐标）
            cv::Point org(50, 50);
            // 定义字体（例如，HERSHEY_SIMPLEX）
            int fontFace = cv::FONT_HERSHEY_SIMPLEX;
            // 定义字体比例
            double fontScale = 1;
            // 定义文本的线宽（如果为负数或FONT_THICKNESS，则填充文本）
            int thickness = 3;
            // 定义文本颜色（例如，白色）
            cv::Scalar color(0, 255, 255);
            cv::putText(preImage, result, org, fontFace, fontScale, color, thickness, cv::LINE_AA);
        }
    }
    Picture(ConfigurationReader *Config_p)
        : Config(Config_p) {}
    ~Picture() = default;
};

#endif // PICTURE_HPP
