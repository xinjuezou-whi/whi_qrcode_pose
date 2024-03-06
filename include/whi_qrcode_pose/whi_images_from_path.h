/******************************************************************
images from path under ROS 1

Features:
- images in folder
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-03-04: Initial version
2024-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_base_camera.h"

#include <string>
#include <vector>
#include <map>

namespace images_from_path
{
    class ImagePathDevice : public WhiCamera
    {
    public:
        explicit ImagePathDevice(const std::string& Path)
            : WhiCamera(), path_(Path) {};
        virtual ~ImagePathDevice() = default;

        bool open() override;
        bool start() override;
        bool stop() override;
        std::shared_ptr<cv::Mat> capture() override;
        std::string getCameraName() const override;

    private:
        std::string path_;
        std::vector<std::string> images_;
    };
}  // namespace images_from_path
