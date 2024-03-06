/******************************************************************
QR code pose detection interface under ROS 1

Features:
- instance image source according to configure
- transplant toCvCopy of cv_bridge
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-03-04: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_base_camera.h"
#include "whi_interfaces/WhiSrvQrOffset.h"

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <memory>
#include <thread>
#include <mutex>

namespace whi_qrcode_pose
{
	class QrcodePose
	{
    public:
        QrcodePose(std::shared_ptr<ros::NodeHandle>& NodeHandle);
        ~QrcodePose();

    protected:
        void init();
        void update(const ros::TimerEvent & Event);
        void streaming(std::shared_ptr<WhiCamera> Camera);
        bool onServiceOffset(whi_interfaces::WhiSrvQrOffset::Request& Request,
            whi_interfaces::WhiSrvQrOffset::Response& Response);
        std::shared_ptr<cv::Mat> toCvMat(const sensor_msgs::Image& Source, const std::string& Encoding);

    protected:
        std::shared_ptr<ros::NodeHandle> node_handle_{ nullptr };
        std::unique_ptr<ros::Timer> non_realtime_loop_{ nullptr };
        ros::Duration elapsed_time_;
        double loop_hz_{ 10.0 };
        bool show_source_image_{ false };
        bool show_detected_image_{ false };
        std::thread th_streaming_;
        std::atomic<bool> terminated_{ false };
        std::unique_ptr<ros::ServiceServer> service_{ nullptr };
        cv::Mat rotation_vec_;
        cv::Mat translation_vec_;
        std::mutex mtx_;
	};
} // namespace whi_qrcode_pose
