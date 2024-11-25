/******************************************************************
QR code pose detection interface under ROS 1

Features:
- instance image source according to configure
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
#include "whi_interfaces/WhiSrvQrcode.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/SetBool.h>

#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <opencv2/objdetect/aruco_detector.hpp>

namespace whi_qrcode_pose
{
	class QrcodePose
	{
	public:
		enum CodeType { TYPE_QR = 0, TYPE_ARUCO, TYPE_SUM };
		static constexpr const char* codeType[TYPE_SUM] = { "qr", "aruco" };

    public:
        QrcodePose(std::shared_ptr<ros::NodeHandle>& NodeHandle);
        ~QrcodePose();

    protected:
        void init();
        void update(const ros::TimerEvent & Event);
        void streaming(std::shared_ptr<WhiCamera> Camera);
        bool onServiceQrcode(whi_interfaces::WhiSrvQrcode::Request& Request,
            whi_interfaces::WhiSrvQrcode::Response& Response);
        bool onServiceActivate(std_srvs::SetBool::Request& Request,
            std_srvs::SetBool::Response& Response);

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
        std::unique_ptr<ros::ServiceServer> service_activate_{ nullptr };
        double intrinsic_unit_unit_scale_{ 1.0 };
        std::vector<cv::Mat> rotations_;
        std::vector<cv::Mat> translations_;
        std::string codes_;
        std::mutex mtx_;
        std::condition_variable cv_;
        bool activated_{ false };
        int request_count_{ 0 };
        std::string code_type_{ codeType[TYPE_QR] };
        double marker_side_length_qr_{ 0.165 }; // in meter
        double marker_side_length_aruco_{ 0.165 }; // in meter
        int dictionary_{ cv::aruco::DICT_4X4_50 };
        int min_marker_perimeter_{ 50 }; // in pixel
	};
} // namespace whi_qrcode_pose
