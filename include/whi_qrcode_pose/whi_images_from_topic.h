/******************************************************************
images from subcribed topic under ROS 2

Features:
- images subscribed
- transplant toCvCopy of cv_bridge
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-03-06: Initial version
2025-10-16: Migrate to ROS 2
2024-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_base_camera.h"
#include "event_queue.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <string>
#include <vector>
#include <map>

namespace images_from_topic
{
    class ImageTopicDevice : public WhiCamera
    {
    public:
        explicit ImageTopicDevice(std::shared_ptr<rclcpp::Node>& NodeHandle, const std::string& Topic)
            : WhiCamera(), node_handle_(NodeHandle), topic_(Topic) {};
        virtual ~ImageTopicDevice() = default;

        bool open() override;
        bool start() override;
        bool stop() override;
        std::shared_ptr<cv::Mat> capture() override;
        std::string getCameraName() const override;

    private:
        void callbackImage(const sensor_msgs::msg::Image::SharedPtr Msg);
        std::shared_ptr<cv::Mat> toCvMat(const sensor_msgs::msg::Image& Source, const std::string& Encoding);

    private:
        std::shared_ptr<rclcpp::Node> node_handle_{ nullptr };
        std::string topic_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_images_{ nullptr };
        EventQueue<cv::Mat>::UniquePtr queue_images_{ nullptr };
    };
}  // namespace images_from_topic
