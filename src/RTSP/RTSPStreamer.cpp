/*
 * @Author: bangbang 1789228622@qq.com
 * @Date: 2024-12-15 21:03:53
 * @LastEditors: bangbang 1789228622@qq.com
 * @LastEditTime: 2024-12-16 00:16:21
 * @FilePath: /success2025/src/RTSP/RTSPStreamer.cpp
 * @Description:
 *
 * Copyright (c) 2024 by CDTU-Success, All Rights Reserved.
 */
#include "./RTSPStreamer.hpp"
#include <iostream>
#include <unistd.h>

RTSPStreamer::RTSPStreamer(const std::string &rtsp_url) : rtsp_url(rtsp_url), done(false)
{
    gst_init(nullptr, nullptr); // 初始化 GStreamer
}

RTSPStreamer::~RTSPStreamer()
{
    stop_stream();
}

void RTSPStreamer::push_frame(const cv::Mat &frame)
{
    std::lock_guard<std::mutex> lock(mtx);

    if (frame_queue.size() >= MAX_QUEUE_SIZE)
    {
        // std::cout << "Dropping frame due to queue full" << std::endl;
        frame_queue.pop();
        return; // 丢弃当前帧
    }
    // else
    // {
    frame_queue.push(frame); // 队列未满时才加入新帧
    cv.notify_one();         // 通知推流线程有新帧
    // }
}

void RTSPStreamer::start_stream()
{
    stream_thread = std::thread(&RTSPStreamer::push_stream_thread, this);
}

void RTSPStreamer::start_frame_generation()
{
    frame_gen_thread = std::thread(&RTSPStreamer::frame_generation_thread, this);
}

void RTSPStreamer::stop_stream()
{
    done = true;
    cv.notify_all(); // 通知推流线程退出
    if (stream_thread.joinable())
    {
        stream_thread.join();
    }
    if (frame_gen_thread.joinable())
    {
        frame_gen_thread.join();
    }
}

void RTSPStreamer::push_stream_thread()
{
    // 设置 GStreamer 管道
    std::string pipeline_str = "appsrc ! videoconvert ! x264enc ! rtph264pay ! udpsink host=" + rtsp_url;
    GstElement *pipeline = gst_parse_launch(pipeline_str.c_str(), nullptr);
    if (!pipeline)
    {
        std::cerr << "Failed to create GStreamer pipeline" << std::endl;
        return;
    }

    GstElement *appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "appsrc0");
    if (!appsrc)
    {
        std::cerr << "Failed to find appsrc element" << std::endl;
        return;
    }

    // 配置 appsrc
    GstCaps *caps = gst_caps_new_simple("video/x-raw",
                                        "format", G_TYPE_STRING, "BGR",
                                        "width", G_TYPE_INT, 640,
                                        "height", G_TYPE_INT, 480,
                                        "framerate", GST_TYPE_FRACTION, 30, 1,
                                        NULL); // 这里添加 NULL 作为参数列表的结束标识符
    gst_app_src_set_caps(GST_APP_SRC(appsrc), caps);
    gst_caps_unref(caps);

    // 主推流循环
    while (!done)
    {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this]()
                { return !frame_queue.empty() || done; });

        if (done && frame_queue.empty())
        {
            break;
        }

        // 获取队列中的帧
        cv::Mat frame = frame_queue.front();
        frame_queue.pop();
        lock.unlock();

        // 创建 GstBuffer
        GstBuffer *buffer = gst_buffer_new_allocate(nullptr, frame.total() * frame.elemSize(), nullptr);
        if (buffer == nullptr)
        {
            std::cerr << "Failed to allocate GstBuffer" << std::endl;
            break;
        }

        // 使用 gst_buffer_map() 来访问缓冲区数据
        GstMapInfo map;
        if (gst_buffer_map(buffer, &map, GST_MAP_WRITE))
        {
            // 将 OpenCV 图像数据复制到 GStreamer 缓冲区
            memcpy(map.data, frame.data, frame.total() * frame.elemSize());
            gst_buffer_unmap(buffer, &map);
        }
        else
        {
            std::cerr << "Failed to map GstBuffer" << std::endl;
            gst_buffer_unref(buffer);
            break;
        }

        // 推送数据到 RTSP
        GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer);
        if (ret != GST_FLOW_OK)
        {
            std::cerr << "Error pushing buffer to appsrc" << std::endl;
            break;
        }

        // 控制帧率（如 30fps）
        usleep(33333); // 33ms，即 30fps
    }

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
}

void RTSPStreamer::frame_generation_thread()
{
    // 模拟每隔 33 毫秒生成一帧，并将其推入队列
    while (!done)
    {
        // 生成一帧黑色图像（你可以根据需要修改生成的图像内容）
        cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3); // 生成一帧黑色图像

        push_frame(frame); // 将帧推入队列

        // 每隔 33 毫秒产生一帧，即 30fps
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
}