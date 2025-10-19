/******************************************************************
IP stream camera device under ROS 2

Features:
- IP stream
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_qrcode_pose/whi_stream.h"

#include <rclcpp/rclcpp.hpp>

namespace ip_stream
{
    bool StreamDevice::open()
    {
        std::string pipeline = "rtspsrc location=" + url_ + " latency=0 ! "
            "rtph264depay ! h264parse ! nvv4l2decoder ! nvvidconv ! video/x-raw,format=BGRx ! "
            "videoconvert ! appsink";
        try
        {
            capture_ = std::make_unique<cv::VideoCapture>(pipeline, cv::CAP_GSTREAMER);
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("ip_stream"), "\033[1;33m" <<
                "failed to create the capture with error: " << e.what() << "\033[0m");
        }
        
        if (!(is_opened_ = capture_->isOpened()))
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("ip_stream"), "\033[1;31m" <<
                "failed to open stream: " << url_<< "\033[0m");
        }
        return is_opened_;
    }

    bool StreamDevice::start()
    {
        RCLCPP_INFO(rclcpp::get_logger("ip_stream"), "Starting camera");

        return true;
    }

    bool StreamDevice::stop()
    {
        capture_->release();
        return true;
    }

    std::shared_ptr<cv::Mat> StreamDevice::capture()
    {
        auto frame = std::make_shared<cv::Mat>();
        if (capture_->read(*frame))
        {
            if (frame->channels() == 4)
            {
                cv::cvtColor(*frame, *frame, cv::COLOR_RGBA2RGB);
            }

            return frame;
        }
        else
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("ip_stream"), "\033[1;33m" <<
                "failed to read frame from stream" << "\033[0m");
            return nullptr;
        }
    }

    std::string StreamDevice::getCameraName() const
    {
        return url_;
    }
}  // namespace ip_stream
