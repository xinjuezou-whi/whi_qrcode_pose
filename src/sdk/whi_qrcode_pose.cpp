/******************************************************************
QR code pose detection interface under ROS 1

Features:
- instance image source according to configure
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_qrcode_pose/whi_qrcode_pose.h"
#include "whi_qrcode_pose/whi_v4l_device.h"
#include "whi_qrcode_pose/whi_images_from_path.h"

namespace whi_qrcode_pose
{
    QrcodePose::QrcodePose(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    QrcodePose::~QrcodePose()
    {
        terminated_.store(true);
        if (th_streaming_.joinable())
        {
            th_streaming_.join();
        }
    }

    void QrcodePose::init()
    {
        // params
        node_handle_->param("frame_id", frame_id_, std::string("camera"));
        std::string imgTopic, camDevice, imgPath;
        node_handle_->param("image_topic", imgTopic, std::string("image"));
        node_handle_->param("camera_device", camDevice, std::string(""));
        node_handle_->param("image_path", imgPath, std::string(""));
        std::string imgSouce;
        node_handle_->param("source", imgSouce, std::string("topic")); // topic, device, path

        /// camera
        std::shared_ptr<WhiCamera> camera;
        if (imgSouce == "topic")
        {
            // topic source
        }
        else if (imgSouce == "device")
        {
            camera = std::make_shared<v4l2_camera::V4l2CameraDevice>(camDevice);
        }
        else if (imgSouce == "path")
        {
            camera = std::make_shared<images_from_path::ImagePathDevice>(imgPath);
        }

        streaming(camera);

        // spinner
        node_handle_->param("loop_hz", loop_hz_, 10.0);
        ros::Duration updateFreq = ros::Duration(1.0 / loop_hz_);
        non_realtime_loop_ = std::make_unique<ros::Timer>(node_handle_->createTimer(updateFreq,
            std::bind(&QrcodePose::update, this, std::placeholders::_1)));
    }

    void QrcodePose::update(const ros::TimerEvent& Event)
    {
        elapsed_time_ = ros::Duration(Event.current_real - Event.last_real);
        // TODO
#ifdef DEBUG
        std::cout << "elapsed " << elapsed_time_.toSec() << std::endl;
#endif
    }

    void QrcodePose::streaming(std::shared_ptr<WhiCamera> Camera)
    {
        if (!Camera->open())
        {
            return;
        }
        Camera->start();

        cam_info_ = std::make_unique<camera_info_manager::CameraInfoManager>(*node_handle_, Camera->getCameraName());

        th_streaming_ = std::thread
        {
            [this, Camera]() -> void
            {
                while (!terminated_.load())
                {
                    auto img = Camera->capture();
                    if (img == nullptr)
                    {
                        // Failed capturing image, assume it is temporarily and continue a bit later
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        continue;
                    }

                    // if (img->encoding != output_encoding_) // TODO
                    // {
                    //     ROS_WARN_STREAM_ONCE(
                    //         "Image encoding not the same as requested output, performing possibly slow conversion: " <<
                    //         img->encoding << " => " << output_encoding_);
                    //     img = convert(*img);
                    // }
                    // img = convert(img);
                    // img->header.stamp = ros::Time::now();
                    // img->header.frame_id = frame_id_;
                    // pub_image_->publish(img);
                    // auto camInfo = std::make_unique<sensor_msgs::CameraInfo>(cam_info_->getCameraInfo());
                    // pub_info_->publish(*img, *camInfo, img->header.stamp);
                }
            }
        };
    }

    // sensor_msgs::Image::Ptr QrcodePose::convert(const sensor_msgs::ImageConstPtr Img) const
    // {
    //     auto cvImg = cv_bridge::toCvShare(Img);
    //     auto outImg = std::make_unique<sensor_msgs::Image>();
    //     auto cvConvertedImg = cv_bridge::cvtColor(cvImg, "bgr8");
    //     cvConvertedImg->toImageMsg(*outImg);

    //     return outImg;
    // }
} // namespace whi_qrcode_pose
