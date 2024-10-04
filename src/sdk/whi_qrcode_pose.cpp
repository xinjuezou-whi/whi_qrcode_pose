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
        node_handle_->param("activated_default", activated_, false);
        std::string frameUnit;
        node_handle_->param("frame_unit", frameUnit, std::string("meter")); // meter and millimeter
        frame_unit_scale_ = frameUnit == "millimeter" ? 1.0 : 0.001;

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
        // code type
        if (node_handle_->param("type", code_type_, std::string(codeType[TYPE_QR])))
        {
            std::transform(code_type_.begin(), code_type_.end(), code_type_.begin(),
                [](unsigned char c) { return std::tolower(c); });
        }
        if (code_type_ == codeType[TYPE_ARUCO])
        {
            std::string dict;
            node_handle_->param("aruco/dictionary", dict, std::string("DICT_4X4_50"));
            std::map<std::string, int> mapDict
            {
				{"DICT_4X4_50", cv::aruco::DICT_4X4_50},
                {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
                {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
                {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
				{"DICT_5X5_50", cv::aruco::DICT_5X5_50},
                {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
                {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
                {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
				{"DICT_6X6_50", cv::aruco::DICT_6X6_50},
                {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
                {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
                {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
				{"DICT_7X7_50", cv::aruco::DICT_7X7_50},
                {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
                {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
                {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000}
			};
            dictionary_ = cv::aruco::DICT_4X4_50;
            if (auto search = mapDict.find(dict); search != mapDict.end())
            {
                dictionary_ = mapDict[dict];
            }

            node_handle_->param("aruco/marker_side_length", marker_side_length_, 0.165);
        }

        streaming(camera);

        // service
        service_ = std::make_unique<ros::ServiceServer>(
            node_handle_->advertiseService("qrcode_pose", &QrcodePose::onServiceQrcode, this));
        service_activate_ = std::make_unique<ros::ServiceServer>(
            node_handle_->advertiseService("qrcode_activate", &QrcodePose::onServiceActivate, this));

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

                    if (activated_)
                    {
                        auto projection = Camera->getIntrinsicProjection();
                        float camVec[9] = { projection[0], 0.0 , projection[2],
                            0.0, projection[1], projection[3],
                            0.0, 0.0, 1.0 };
                        cv::Mat cameraMatrix(3, 3, CV_32F, camVec);
                        auto distortion = Camera->getIntrinsicDistortion();
                        cv::Mat distortionCoeffs(4, 1, CV_32F, distortion.data());

                        if (code_type_ == codeType[TYPE_QR])
                        {
                            cv::QRCodeDetector detecter;
                            cv::Mat codeCorners;
                            if (detecter.detect(*img, codeCorners))
                            {
#ifdef DEBUG
                                std::cout << "code corners " << codeCorners << std::endl;
                                cv::circle(*img, cv::Point2i(int(codeCorners.at<float>(0, 0)), int(codeCorners.at<float>(0, 1))), 5, cv::Scalar(0, 0, 255), 10);
                                cv::circle(*img, cv::Point2i(int(codeCorners.at<float>(0, 2)), int(codeCorners.at<float>(0, 3))), 5, cv::Scalar(0, 255, 0), 10);
                                cv::circle(*img, cv::Point2i(int(codeCorners.at<float>(0, 4)), int(codeCorners.at<float>(0, 5))), 5, cv::Scalar(255, 0, 0), 10);
                                cv::circle(*img, cv::Point2i(int(codeCorners.at<float>(0, 6)), int(codeCorners.at<float>(0, 7))), 5, cv::Scalar(255, 255, 0), 10);
#endif
                                float objectPoints[12] = { // follow the order of detected corners of QR code
                                    1.0, 0.0, 0.0, // top-left
                                    0.0, 0.0, 0.0, // top-right
                                    0.0, 1.0, 0.0, // bottom-right
                                    1.0, 1.0, 0.0  // bottom-left
                                };
                                cv::Mat objectVec(4, 3, CV_32F, objectPoints);

                                // bool res = false;
                                cv::Mat rvec, tvec;
                                try
                                {
                                    codes_ = detecter.decode(*img, codeCorners);
                                    if (cv::solvePnP(objectVec, codeCorners, cameraMatrix, distortionCoeffs,
                                        rvec, tvec))
                                    {
                                        cv::solvePnPRefineLM(objectVec, codeCorners, cameraMatrix, distortionCoeffs,
                                            rvec, tvec);

                                        // project to 2D frame
                                        float wrtPoints[12] = {
                                            0.0, 0.0, 0.0, // origin
                                            1.0, 0.0, 0.0, // x
                                            0.0, 1.0, 0.0, // y
                                            0.0, 0.0, 1.0  // z
                                        };
                                        cv::Mat wrtVec(4, 3, CV_32F, wrtPoints);
                                        cv::Mat imgPoints, jacob;
                                        cv::projectPoints(wrtVec, rvec, tvec, cameraMatrix, distortionCoeffs,
                                            imgPoints, jacob);
                                        
                                        cv::Point topL(imgPoints.at<float>(1, 0), imgPoints.at<float>(1, 1));
                                        cv::Point topR(imgPoints.at<float>(0, 0), imgPoints.at<float>(0, 1));
                                        cv::Point bottomR(imgPoints.at<float>(2, 0), imgPoints.at<float>(2, 1));
                                        cv::Point cornerTopL(codeCorners.at<float>(0, 0), codeCorners.at<float>(0, 1));
                                        cv::Point cornerTopR(codeCorners.at<float>(0, 2), codeCorners.at<float>(0, 3));
                                        cv::Point cornerBottomR(codeCorners.at<float>(0, 4), codeCorners.at<float>(0, 5));
                                        if (fabs(cv::norm(topL - cornerTopL)) < 3.0 &&
                                            fabs(cv::norm(topR - cornerTopR)) < 3.0 &&
                                            fabs(cv::norm(bottomR - cornerBottomR)) < 3.0)
                                        {
#ifdef DEBUG
                                            std::cout << "translation " << tvec << " and type " << 
                                                cv::typeToString(tvec.type()) << std::endl;
                                            std::cout << "rotation " << rvec << " and type " <<
                                                cv::typeToString(rvec.type()) << std::endl;
#endif

                                            if (request_count_ > 0)
                                            {
                                                rotations_.push_back(rvec);
                                                translations_.push_back(tvec);
                                                if (rotations_.size() >= request_count_)
                                                {
                                                    request_count_ = 0;
                                                    cv_.notify_all();
                                                }
                                            }

                                            if (show_detected_image_)
                                            {
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
                                catch (const std::exception& e)
                                {
                                    ROS_WARN_STREAM("failed to solvePnp problem with: " << e.what());
                                }
                            }
                        }
                        else if (code_type_ == codeType[TYPE_ARUCO])
                        {
                            cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
                            cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(dictionary_);
                            cv::aruco::ArucoDetector detector(dictionary, detectorParams);
                            std::vector<int> markerIds;
                            std::vector<std::vector<cv::Point2f>> corners, rejectedCandidates;
                            detector.detectMarkers(*img, corners, markerIds, rejectedCandidates);

                            if (!markerIds.empty())
                            {
                                // set coordinate system
                                cv::Mat objPoints(4, 1, CV_32FC3);
                                objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-marker_side_length_/2.f, marker_side_length_/2.f, 0);
                                objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(marker_side_length_/2.f, marker_side_length_/2.f, 0);
                                objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(marker_side_length_/2.f, -marker_side_length_/2.f, 0);
                                objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-marker_side_length_/2.f, -marker_side_length_/2.f, 0);

                                size_t markers = markerIds.size();
                                std::vector<cv::Vec3d> rvecs(markers), tvecs(markers);
                                // calculate pose for each marker
                                for (size_t i = 0; i < markers; ++i)
                                {
                                    cv::solvePnP(objPoints, corners.at(i), cameraMatrix, distortionCoeffs,
                                        rvecs.at(i), tvecs.at(i));
                                }

                                if (show_detected_image_)
                                {
                                    for (size_t i = 0; i < markerIds.size(); ++i)
                                    {
                                        cv::drawFrameAxes(*img, cameraMatrix, distortionCoeffs, rvecs[i], tvecs[i],
                                            marker_side_length_ * 1.5f, 8);
                                    }

                                    cv::imshow("with coordinate", *img);
                                    images_from_path::ImagePathDevice* dev =
                                        dynamic_cast<images_from_path::ImagePathDevice*>(Camera.get());
                                    cv::waitKey(dev == nullptr ? 1 : 0);
                                }
                            }
                        }
                    }
                }
            }
        };
    }

    bool QrcodePose::onServiceQrcode(whi_interfaces::WhiSrvQrcode::Request& Request,
        whi_interfaces::WhiSrvQrcode::Response& Response)
    {
        if (!activated_)
        {
            return false;
        }
        else
        {
            request_count_ = Request.count;

            {
                std::unique_lock<std::mutex> lock(mtx_);
                if (cv_.wait_for(lock, std::chrono::seconds(std::max(5, int(1.5 * request_count_)))) == std::cv_status::timeout)
                {
                    request_count_ = 0;
                    translations_.clear();
                    rotations_.clear();
                    return false;
                }
            }

            Response.code = codes_;

            for (const auto& it : translations_)
            {
                Response.offset_pose.pose.position.x += it.at<double>(0, 0) * frame_unit_scale_;
                Response.offset_pose.pose.position.y += it.at<double>(0, 1) * frame_unit_scale_;
                Response.offset_pose.pose.position.z += it.at<double>(0, 2) * frame_unit_scale_;
            }
            Response.offset_pose.pose.position.x /= translations_.size();
            Response.offset_pose.pose.position.y /= translations_.size();
            Response.offset_pose.pose.position.z /= translations_.size();
            translations_.clear();

            std::vector<double> vecAvg;
            vecAvg.resize(3);
            for (const auto& it : rotations_)
            {
                vecAvg[0] += it.at<double>(0, 0);
                vecAvg[1] += it.at<double>(0, 1);
                vecAvg[2] += it.at<double>(0, 2);
            }
            vecAvg[0] /= rotations_.size();
            vecAvg[1] /= rotations_.size();
            vecAvg[2] /= rotations_.size();
            rotations_.clear();
            cv::Mat rotationAvg(vecAvg);
#ifndef DEBUG
            std::cout << "QR average rotation:" << rotationAvg << std::endl;
#endif

            cv::Mat rotation;
            cv::Rodrigues(rotationAvg, rotation);
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
    }

    bool QrcodePose::onServiceActivate(std_srvs::SetBool::Request& Request,
            std_srvs::SetBool::Response& Response)
    {
        activated_ = Request.data;
        Response.success = true;

        return Response.success;
    }
} // namespace whi_qrcode_pose
