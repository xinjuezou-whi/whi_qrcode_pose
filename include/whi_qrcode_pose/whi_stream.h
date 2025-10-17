/******************************************************************
IP stream camera device under ROS 2

Features:
- IP stream
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2025-10-17: Initial version
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_base_camera.h"

#include <opencv2/opencv.hpp>

namespace ip_stream
{
    class StreamDevice : public WhiCamera
    {
    public:
        explicit StreamDevice(const std::string& Url)
            : WhiCamera(), url_(Url) {};
        virtual ~StreamDevice() = default;

        bool open() override;
        bool start() override;
        bool stop() override;
        std::shared_ptr<cv::Mat> capture() override;
        std::string getCameraName() const override;

    private:
        std::string url_;
        std::unique_ptr<cv::VideoCapture> capture_{ nullptr };
    };
}  // namespace ip_stream
