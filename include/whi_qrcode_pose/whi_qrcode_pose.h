/******************************************************************
QR code pose detection interface under ROS 2

Features:
- instance image source according to configure
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-03-04: Initial version
2025-10-16: Migrate to ros 2
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_base_camera.h"
#include "whi_interfaces/srv/whi_srv_qrcode.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>

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
        QrcodePose(std::shared_ptr<rclcpp::Node>& NodeHandle);
        ~QrcodePose();

    protected:
        void init();
        void update();
        void streaming(std::shared_ptr<WhiCamera> Camera);
        void estimate();
        bool onServiceQrcode(const std::shared_ptr<whi_interfaces::srv::WhiSrvQrcode::Request> Request,
	        std::shared_ptr<whi_interfaces::srv::WhiSrvQrcode::Response> Response);
        bool onServiceActivate(const std::shared_ptr<std_srvs::srv::SetBool::Request> Request,
	        std::shared_ptr<std_srvs::srv::SetBool::Response> Response);

    protected:
        std::shared_ptr<rclcpp::Node> node_handle_{ nullptr };
        rclcpp::TimerBase::SharedPtr non_realtime_loop_{ nullptr };
        bool show_source_image_{ false };
        bool show_detected_image_{ false };
        std::thread th_streaming_;
        std::atomic<bool> terminated_{ false };
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_{ nullptr };
        rclcpp::Service<whi_interfaces::srv::WhiSrvQrcode>::SharedPtr service_{ nullptr };
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_activate_{ nullptr };
        double intrinsic_unit_unit_scale_{ 0.1 };
        std::vector<cv::Mat> rotations_;
        std::vector<cv::Mat> translations_;
        std::string codes_;
        geometry_msgs::msg::PoseStamped estimated_pose_;
        std::mutex mtx_;
        std::condition_variable cv_;
        bool activated_{ false };
        int publish_count_{ 5 };
        int estimate_count_{ 0 };
        std::string code_type_{ codeType[TYPE_QR] };
        double marker_side_length_qr_{ 0.165 }; // in meter
        double marker_side_length_aruco_{ 0.165 }; // in meter
        int dictionary_{ cv::aruco::DICT_4X4_50 };
        int min_marker_perimeter_{ 50 }; // in pixel
	};
} // namespace whi_qrcode_pose
