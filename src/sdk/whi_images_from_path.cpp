/******************************************************************
images from path under ROS 1

Features:
- images in folder
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_qrcode_pose/whi_images_from_path.h"

#include <ros/ros.h>

#include <filesystem>

namespace images_from_path
{
    bool ImagePathDevice::open()
    {
        for (const auto& entry : std::filesystem::directory_iterator(std::filesystem::path(path_)))
        {
#ifdef DEBUG
            std::cout << entry.path() << std::endl;
#endif
            images_.push_back(entry.path().string());
        }

        return (is_opened_ = !images_.empty());
    }

    bool ImagePathDevice::start()
    {
        ROS_INFO("Starting camera");

        return true;
    }

    bool ImagePathDevice::stop()
    {
        return true;
    }

    std::shared_ptr<cv::Mat> ImagePathDevice::capture()
    {
        static int index = 0;
        if (index < images_.size())
        {
            auto img = std::make_shared<cv::Mat>(cv::imread(images_[index++]));
            if (img->empty())
            {
                return nullptr;
            }

            return img;
        }
        else
        {
            return nullptr;
        }
    }

    std::string ImagePathDevice::getCameraName() const
    {
        return path_;
    }
}  // namespace images_from_path
