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
#include "whi_qrcode_pose/whi_images_from_topic.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/opencv.hpp>

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
        std::string imgTopic, camDevice, imgPath;
        node_handle_->param("image_topic", imgTopic, std::string("image"));
        node_handle_->param("camera_device", camDevice, std::string(""));
        node_handle_->param("image_path", imgPath, std::string(""));
        std::string imgSouce;
        node_handle_->param("source", imgSouce, std::string("topic")); // topic, device, path
        node_handle_->param("show_source_image", show_source_image_, false);
        node_handle_->param("show_detected_image", show_detected_image_, false);

        /// camera
        std::shared_ptr<WhiCamera> camera;
        if (imgSouce == "topic")
        {
            camera = std::make_shared<images_from_topic::ImageTopicDevice>(node_handle_, imgTopic);
        }
        else if (imgSouce == "device")
        {
            camera = std::make_shared<v4l2_camera::V4l2CameraDevice>(camDevice);
        }
        else if (imgSouce == "path")
        {
            camera = std::make_shared<images_from_path::ImagePathDevice>(imgPath);
        }
        if (camera)
        {
            std::vector<double> intrinsicProjection, intrinsicDistortion;
            node_handle_->getParam("intrinsic_projection", intrinsicProjection);
            node_handle_->getParam("intrinsic_distortion", intrinsicDistortion);

            camera->setIntrinsicProjection(intrinsicProjection);
            camera->setIntrinsicDistortion(intrinsicDistortion);
        }

        streaming(camera);

        // service
        std::string service;
        node_handle_->param("service", service, std::string("offset_request"));
        service_ = std::make_unique<ros::ServiceServer>(
            node_handle_->advertiseService(service, &QrcodePose::onServiceQrcode, this));

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

        th_streaming_ = std::thread
        {
            [this, Camera]() -> void
            {
                while (!terminated_.load())
                {
                    auto img = Camera->capture();
                    if (!img)
                    {
                        // Failed capturing image, assume it is temporarily and continue a bit later
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        continue;
                    }
                    if (show_source_image_)
                    {
                        cv::imshow("source image", *img);
                        cv::waitKey(1);
                    }

                    cv::Mat src;
                    cv::QRCodeDetector detecter;
                                            cv::imshow("source image", *img);
                        cv::waitKey(1);

	                cv::Mat codeCorners;
	                if (detecter.detect(*img, codeCorners))
                    {
#ifndef DEBUG
                        std::cout << "code corners " << codeCorners << std::endl;
#endif
                        float objectPoints[12] = { // follow the order of detected corners of QR code
                            0.0, 0.0, 0.0, // top-left
                            0.0, 1.0, 0.0, // top-right
                            1.0, 1.0, 0.0, // bottom-right
                            1.0, 0.0, 0.0  // bottom-left
                        };
                        cv::Mat objectVec(4, 3, CV_32F, objectPoints);

                        auto projection = Camera->getIntrinsicProjection();
                        float camVec[9] = { projection[0], 0.0 , projection[2],
                            0.0, projection[1], projection[3],
                            0.0, 0.0, 1.0 };
                        cv::Mat cameraMatrix(3, 3, CV_32F, camVec);
                        auto distortion = Camera->getIntrinsicDistortion();
                        cv::Mat distortionCoeffs(4, 1, CV_32F, distortion.data());

                        bool res = false;
                        {
                            const std::lock_guard<std::mutex> lock(mtx_);
                            codes_ = detecter.decode(*img, codeCorners);
                            res = cv::solvePnP(objectVec, codeCorners, cameraMatrix, distortionCoeffs,
                                rotation_vec_, translation_vec_);
                        }

                        if (res && show_detected_image_)
                        {
#ifdef DEBUG
                            std::cout << "translation " << translation_vec_ << " and type " << 
                                cv::typeToString(translation_vec_.type()) << std::endl;
                            std::cout << "rotation " << rotation_vec_ << " and type " <<
                                cv::typeToString(rotation_vec_.type()) << std::endl;
#endif
                            // project to 2D frame
                            float wrtPoints[12] = {
                                0.0, 0.0, 0.0, // origin
                                1.0, 0.0, 0.0, // x
                                0.0, 1.0, 0.0, // y
                                0.0, 0.0, 0.1  // z
                            };
                            cv::Mat wrtVec(4, 3, CV_32F, wrtPoints);
                            cv::Mat imgPoints, jacob;
                            cv::projectPoints(wrtVec, rotation_vec_, translation_vec_, cameraMatrix, distortionCoeffs,
                                imgPoints, jacob);

                            // draw coordinate in 2D frame
                            cv::Scalar color[3] = { cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0) };
                            cv::Point2i origin(int(imgPoints.at<float>(0, 0)), int(imgPoints.at<float>(0, 1)));
                            for (int i = 1; i < 4; ++i)
                            {
                                cv::line(*img, origin,
                                    cv::Point2i(int(imgPoints.at<float>(i, 0)), int(imgPoints.at<float>(i, 1))),
                                    color[i - 1], 8);
                            }

                            cv::imshow("with coordinate", *img);
                            images_from_path::ImagePathDevice* dev =
                                dynamic_cast<images_from_path::ImagePathDevice*>(Camera.get());
                            cv::waitKey(dev == nullptr ? 1 : 0);
                        }
                    }
                }
            }
        };
    }

    bool QrcodePose::onServiceQrcode(whi_interfaces::WhiSrvQrcode::Request& Request,
        whi_interfaces::WhiSrvQrcode::Response& Response)
    {
        if (!translation_vec_.empty() && !rotation_vec_.empty())
        {
            cv::Mat rotation;
            {
                const std::lock_guard<std::mutex> lock(mtx_);
                Response.code = codes_;
                Response.offset_pose.pose.position.x = translation_vec_.at<double>(0, 0);
                Response.offset_pose.pose.position.y = translation_vec_.at<double>(0, 1);
                Response.offset_pose.pose.position.z = translation_vec_.at<double>(0, 2);
                cv::Rodrigues(rotation_vec_, rotation);
            }

            // convert to tf2::Matrix3x3
            tf2::Matrix3x3 tf2Rotation(rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2),
                                rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2),
                                rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2));
            tf2::Transform tf2Transform(tf2Rotation);
            geometry_msgs::Pose poseMsg;
            tf2::toMsg(tf2Transform, poseMsg);

            Response.offset_pose.pose.orientation = poseMsg.orientation;

            return true;
        }
        else
        {
            return false;
        }
    }
} // namespace whi_qrcode_pose
