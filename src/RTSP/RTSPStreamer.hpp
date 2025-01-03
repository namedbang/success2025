/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-12-15 21:04:07
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2025-01-03 13:54:36
 * @FilePath: /success2025/src/RTSP/RTSPStreamer.hpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
#ifndef __RTSPSTREAMER_H__
#define __RTSPSTREAMER_H__

#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h> // 确保包含此头文件
#include <thread>
#include <deque>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

#define MAX_QUEUE_SIZE 3 // 队列最大大小，避免过多堆积帧

class RTSPStreamer
{
public:
    RTSPStreamer(const std::string &rtsp_url);
    ~RTSPStreamer();

    // 启动推流和帧生成线程
    void start_stream();
    void start_frame_generation();

    // 停止推流和帧生成线程
    void stop_stream();
    void push_frame(const cv::Mat &frame);

private:
    void push_stream_thread();
    void frame_generation_thread();

private:
    std::string rtsp_url;
    bool done;
    std::deque<cv::Mat> frame_queue; // 存放待推送帧的队列
    std::mutex mtx;                  // 用于队列同步
    std::condition_variable cv;      // 用于线程间同步
    std::thread stream_thread;       // 推流线程
    std::thread frame_gen_thread;    // 帧生成线程
};

#endif /* __RTSPSTREAMER_H__ */