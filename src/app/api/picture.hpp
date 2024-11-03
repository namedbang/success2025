/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-11-02 12:50:18
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-11-03 14:20:08
 * @FilePath: /success2025/src/app/api/picture.hpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
#ifndef PICTURE_HPP
#define PICTURE_HPP

#include <opencv2/imgproc/types_c.h>
#include "opencv2/highgui/highgui.hpp"

class Picture
{
private:
    /* data */
public:
    cv::Mat preImage;
    char ImgShow()
    { // in while
        cv::imshow("success2025", preImage);
        return cv::waitKey(1) == 'q';
    }
    Picture(/* args */) = default;
    ~Picture() = default;
};

#endif // PICTURE_HPP
