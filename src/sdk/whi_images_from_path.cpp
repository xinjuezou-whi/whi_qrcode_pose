/******************************************************************
images from path under ROS 1

Features:
- images in folder
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_qrcode_pose/whi_images_from_path.h"

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

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

    sensor_msgs::Image::Ptr ImagePathDevice::capture()
    {
	    cv::Mat src = cv::imread(images_.front());
	    if (src.empty())
        {
		    return nullptr;
	    }
	    cv::imshow("src image", src);
        cv::waitKey(1);

        // Create image object
        auto img = std::make_unique<sensor_msgs::Image>();

        return nullptr;
    }

    std::string ImagePathDevice::getCameraName() const
    {
        return path_;
    }
}  // namespace images_from_path
