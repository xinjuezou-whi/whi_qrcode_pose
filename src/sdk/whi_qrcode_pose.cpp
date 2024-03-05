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
            std::vector<double> intrinsicProjection, intrinsicDistortion;
            node_handle_->getParam("intrinsic_projection", intrinsicProjection);
            node_handle_->getParam("intrinsic_distortion", intrinsicDistortion);
            camera = std::make_shared<images_from_path::ImagePathDevice>(imgPath);
            camera->setIntrinsicProjection(intrinsicProjection);
            camera->setIntrinsicDistortion(intrinsicDistortion);
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
                    if (!img)
                    {
                        // Failed capturing image, assume it is temporarily and continue a bit later
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        continue;
                    }

                    cv::Mat src;
                    cv::QRCodeDetector detecter;

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

                        cv::Mat rVec, tVec;
                        if (cv::solvePnP(objectVec, codeCorners, cameraMatrix, distortionCoeffs, rVec, tVec))
                        {
#ifndef DEBUG
                            std::cout << "translation " << tVec << std::endl;
                            std::cout << "rotation " << rVec << std::endl;
#endif
                            float wrtPoints[12] = {
                                0.0, 0.0, 0.0, // origin
                                1.0, 0.0, 0.0, // x
                                0.0, 1.0, 0.0, // y
                                0.0, 0.0, 0.1  // z
                            };
                            cv::Mat wrtVec(4, 3, CV_32F, wrtPoints);
                            cv::Mat imgPoints, jacob;
                            cv::projectPoints(wrtVec, rVec, tVec, cameraMatrix, distortionCoeffs, imgPoints, jacob);

                            // draw coordinate
                            cv::Scalar color[3] = { cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0) };
                            cv::Point2i origin(int(imgPoints.at<float>(0, 0)), int(imgPoints.at<float>(0, 1)));
                            for (int i = 1; i < 4; ++i)
                            {
                                cv::line(*img, origin,
                                    cv::Point2i(int(imgPoints.at<float>(i, 0)), int(imgPoints.at<float>(i, 1))),
                                    color[i - 1], 8);
                            }
#ifndef DEBUG
                            cv::imshow("with coordinate", *img);
                            cv::waitKey(0);
#endif
                        }
                    }
                }
            }
        };
    }
} // namespace whi_qrcode_pose
