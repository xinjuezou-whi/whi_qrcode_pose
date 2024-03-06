/******************************************************************
QR code pose detection interface under ROS 1

Features:
- instance image source according to configure
- transplant toCvCopy of cv_bridge
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_qrcode_pose/whi_qrcode_pose.h"
#include "whi_qrcode_pose/whi_v4l_device.h"
#include "whi_qrcode_pose/whi_images_from_path.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/opencv.hpp>

#include "boost/endian/conversion.hpp"
#include <regex>

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
            // topic source
        }
        else if (imgSouce == "device")
        {
            camera = std::make_shared<v4l2_camera::V4l2CameraDevice>(camDevice);

            std::vector<double> intrinsicProjection, intrinsicDistortion;
            node_handle_->getParam("intrinsic_projection", intrinsicProjection);
            node_handle_->getParam("intrinsic_distortion", intrinsicDistortion);
            camera->setIntrinsicProjection(intrinsicProjection);
            camera->setIntrinsicDistortion(intrinsicDistortion);
        }
        else if (imgSouce == "path")
        {
            camera = std::make_shared<images_from_path::ImagePathDevice>(imgPath);

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
            node_handle_->advertiseService(service, &QrcodePose::onServiceOffset, this));

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
                            res = cv::solvePnP(objectVec, codeCorners, cameraMatrix, distortionCoeffs,
                                rotation_vec_, translation_vec_);
                        }

                        if (res)
                        {
#ifdef DEBUG
                            std::cout << "translation " << translation_vec_ << " and type " << 
                                cv::typeToString(translation_vec_.type()) << std::endl;
                            std::cout << "rotation " << rotation_vec_ << " and type " <<
                                cv::typeToString(rotation_vec_.type()) << std::endl;
#endif
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

                            if (show_detected_image_)
                            {
                                // draw coordinate
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
            }
        };
    }

    bool QrcodePose::onServiceOffset(whi_interfaces::WhiSrvQrOffset::Request& Request,
        whi_interfaces::WhiSrvQrOffset::Response& Response)
    {
        if (!translation_vec_.empty() && !rotation_vec_.empty())
        {
            cv::Mat rotation;
            {
                const std::lock_guard<std::mutex> lock(mtx_);
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

    static int depthStrToInt(const std::string Depth)
    {
        if (Depth == "8U")
        {
            return 0;
        }
        else if (Depth == "8S")
        {
            return 1;
        }
        else if (Depth == "16U")
        {
            return 2;
        }
        else if (Depth == "16S")
        {
            return 3;
        }
        else if (Depth == "32S")
        {
            return 4;
        }
        else if (Depth == "32F")
        {
            return 5;
        }
        return 6;
    }

    static int getCvType(const std::string& Encoding)
    {
        // Check for the most common encodings first
        if (Encoding == sensor_msgs::image_encodings::BGR8)   return CV_8UC3;
        if (Encoding == sensor_msgs::image_encodings::MONO8)  return CV_8UC1;
        if (Encoding == sensor_msgs::image_encodings::RGB8)   return CV_8UC3;
        if (Encoding == sensor_msgs::image_encodings::MONO16) return CV_16UC1;
        if (Encoding == sensor_msgs::image_encodings::BGR16)  return CV_16UC3;
        if (Encoding == sensor_msgs::image_encodings::RGB16)  return CV_16UC3;
        if (Encoding == sensor_msgs::image_encodings::BGRA8)  return CV_8UC4;
        if (Encoding == sensor_msgs::image_encodings::RGBA8)  return CV_8UC4;
        if (Encoding == sensor_msgs::image_encodings::BGRA16) return CV_16UC4;
        if (Encoding == sensor_msgs::image_encodings::RGBA16) return CV_16UC4;

        // For bayer, return one-channel
        if (Encoding == sensor_msgs::image_encodings::BAYER_RGGB8) return CV_8UC1;
        if (Encoding == sensor_msgs::image_encodings::BAYER_BGGR8) return CV_8UC1;
        if (Encoding == sensor_msgs::image_encodings::BAYER_GBRG8) return CV_8UC1;
        if (Encoding == sensor_msgs::image_encodings::BAYER_GRBG8) return CV_8UC1;
        if (Encoding == sensor_msgs::image_encodings::BAYER_RGGB16) return CV_16UC1;
        if (Encoding == sensor_msgs::image_encodings::BAYER_BGGR16) return CV_16UC1;
        if (Encoding == sensor_msgs::image_encodings::BAYER_GBRG16) return CV_16UC1;
        if (Encoding == sensor_msgs::image_encodings::BAYER_GRBG16) return CV_16UC1;

        // Miscellaneous
        if (Encoding == sensor_msgs::image_encodings::YUV422) return CV_8UC2;

        // Check all the generic content encodings
        std::cmatch m;

        if (std::regex_match(Encoding.c_str(), m,
            std::regex("(8U|8S|16U|16S|32S|32F|64F)C([0-9]+)")))
        {
            return CV_MAKETYPE(depthStrToInt(m[1].str()), atoi(m[2].str().c_str()));
        }

        if (std::regex_match(Encoding.c_str(), m,
            std::regex("(8U|8S|16U|16S|32S|32F|64F)")))
        {
            return CV_MAKETYPE(depthStrToInt(m[1].str()), 1);
        }

        throw std::runtime_error("Unrecognized image encoding [" + Encoding + "]");
    }

    static cv::Mat matFromImageMsg(const sensor_msgs::Image& Source)
    {
        int sourceType = getCvType(Source.encoding);
        int byteDepth = sensor_msgs::image_encodings::bitDepth(Source.encoding) / 8;
        int numChannels = sensor_msgs::image_encodings::numChannels(Source.encoding);

        if (Source.step < Source.width * byteDepth * numChannels)
        {
            std::stringstream ss;
            ss << "Image is wrongly formed: step < width * byteDepth * numChannels  or  " << Source.step << " != " <<
                Source.width << " * " << byteDepth << " * " << numChannels;
            throw std::runtime_error(ss.str());
        }
        if (Source.height * Source.step != Source.data.size())
        {
            std::stringstream ss;
            ss << "Image is wrongly formed: height * step != size  or  " << Source.height << " * " <<
                        Source.step << " != " << Source.data.size();
            throw std::runtime_error(ss.str());
        }

        // If the endianness is the same as locally, share the data
        cv::Mat mat(Source.height, Source.width, sourceType, const_cast<uchar*>(&Source.data[0]), Source.step);
        if ((boost::endian::order::native == boost::endian::order::big && Source.is_bigendian) ||
            (boost::endian::order::native == boost::endian::order::little && !Source.is_bigendian) ||
            byteDepth == 1)
        {
            return mat;
        }
        // Otherwise, reinterpret the data as bytes and switch the channels accordingly
        mat = cv::Mat(Source.height, Source.width, CV_MAKETYPE(CV_8U, numChannels * byteDepth),
            const_cast<uchar*>(&Source.data[0]), Source.step);
        cv::Mat matSwap(Source.height, Source.width, mat.type());

        std::vector<int> fromTo;
        fromTo.reserve(numChannels * byteDepth);
        for (int i = 0; i < numChannels; ++i)
        {
            for (int j = 0; j < byteDepth; ++j)
            {
                fromTo.push_back(byteDepth * i + j);
                fromTo.push_back(byteDepth * i + byteDepth - 1 - j);
            }
        }
        cv::mixChannels(std::vector<cv::Mat>(1, mat), std::vector<cv::Mat>(1, matSwap), fromTo);

        // Interpret matSwap back as the proper type
        matSwap.reshape(numChannels);

        return matSwap;
    }

    enum Encoding { INVALID = -1, GRAY = 0, RGB, BGR, RGBA, BGRA, YUV422, BAYER_RGGB, BAYER_BGGR, BAYER_GBRG, BAYER_GRBG};
    static Encoding getEncoding(const std::string& Encoding)
    {
        if ((Encoding == sensor_msgs::image_encodings::MONO8) || (Encoding == sensor_msgs::image_encodings::MONO16))
        {
            return GRAY;
        }
        if ((Encoding == sensor_msgs::image_encodings::BGR8) || (Encoding == sensor_msgs::image_encodings::BGR16))
        {
            return BGR;
        }
        if ((Encoding == sensor_msgs::image_encodings::RGB8) || (Encoding == sensor_msgs::image_encodings::RGB16))
        {
            return RGB;
        }
        if ((Encoding == sensor_msgs::image_encodings::BGRA8) || (Encoding == sensor_msgs::image_encodings::BGRA16))
        {
            return BGRA;
        }
        if ((Encoding == sensor_msgs::image_encodings::RGBA8) || (Encoding == sensor_msgs::image_encodings::RGBA16))
        {
            return RGBA;
        }
        if (Encoding == sensor_msgs::image_encodings::YUV422)
        {
            return YUV422;
        }
        if ((Encoding == sensor_msgs::image_encodings::BAYER_RGGB8) ||
            (Encoding == sensor_msgs::image_encodings::BAYER_RGGB16))
        {
            return BAYER_RGGB;
        }
        if ((Encoding == sensor_msgs::image_encodings::BAYER_BGGR8) ||
            (Encoding == sensor_msgs::image_encodings::BAYER_BGGR16))
        {
            return BAYER_BGGR;
        }
        if ((Encoding == sensor_msgs::image_encodings::BAYER_GBRG8) ||
            (Encoding == sensor_msgs::image_encodings::BAYER_GBRG16))
        {
            return BAYER_GBRG;
        }
        if ((Encoding == sensor_msgs::image_encodings::BAYER_GRBG8) ||
            (Encoding == sensor_msgs::image_encodings::BAYER_GRBG16))
        {
            return BAYER_GRBG;
        }

        // We don't support conversions to/from other types
        return INVALID;
    }

    static const int SAME_FORMAT = -1;

    std::map<std::pair<Encoding, Encoding>, std::vector<int>> getConversionCodes()
    {
        std::map<std::pair<Encoding, Encoding>, std::vector<int>> res;
        for(int i = 0; i <= 5; ++i)
        {
            res[std::pair<Encoding, Encoding>(Encoding(i),Encoding(i))].push_back(SAME_FORMAT);
        }

        res[std::make_pair(GRAY, RGB)].push_back(cv::COLOR_GRAY2RGB);
        res[std::make_pair(GRAY, BGR)].push_back(cv::COLOR_GRAY2BGR);
        res[std::make_pair(GRAY, RGBA)].push_back(cv::COLOR_GRAY2RGBA);
        res[std::make_pair(GRAY, BGRA)].push_back(cv::COLOR_GRAY2BGRA);

        res[std::make_pair(RGB, GRAY)].push_back(cv::COLOR_RGB2GRAY);
        res[std::make_pair(RGB, BGR)].push_back(cv::COLOR_RGB2BGR);
        res[std::make_pair(RGB, RGBA)].push_back(cv::COLOR_RGB2RGBA);
        res[std::make_pair(RGB, BGRA)].push_back(cv::COLOR_RGB2BGRA);

        res[std::make_pair(BGR, GRAY)].push_back(cv::COLOR_BGR2GRAY);
        res[std::make_pair(BGR, RGB)].push_back(cv::COLOR_BGR2RGB);
        res[std::make_pair(BGR, RGBA)].push_back(cv::COLOR_BGR2RGBA);
        res[std::make_pair(BGR, BGRA)].push_back(cv::COLOR_BGR2BGRA);

        res[std::make_pair(RGBA, GRAY)].push_back(cv::COLOR_RGBA2GRAY);
        res[std::make_pair(RGBA, RGB)].push_back(cv::COLOR_RGBA2RGB);
        res[std::make_pair(RGBA, BGR)].push_back(cv::COLOR_RGBA2BGR);
        res[std::make_pair(RGBA, BGRA)].push_back(cv::COLOR_RGBA2BGRA);

        res[std::make_pair(BGRA, GRAY)].push_back(cv::COLOR_BGRA2GRAY);
        res[std::make_pair(BGRA, RGB)].push_back(cv::COLOR_BGRA2RGB);
        res[std::make_pair(BGRA, BGR)].push_back(cv::COLOR_BGRA2BGR);
        res[std::make_pair(BGRA, RGBA)].push_back(cv::COLOR_BGRA2RGBA);

        res[std::make_pair(YUV422, GRAY)].push_back(cv::COLOR_YUV2GRAY_UYVY);
        res[std::make_pair(YUV422, RGB)].push_back(cv::COLOR_YUV2RGB_UYVY);
        res[std::make_pair(YUV422, BGR)].push_back(cv::COLOR_YUV2BGR_UYVY);
        res[std::make_pair(YUV422, RGBA)].push_back(cv::COLOR_YUV2RGBA_UYVY);
        res[std::make_pair(YUV422, BGRA)].push_back(cv::COLOR_YUV2BGRA_UYVY);

        // Deal with Bayer
        res[std::make_pair(BAYER_RGGB, GRAY)].push_back(cv::COLOR_BayerBG2GRAY);
        res[std::make_pair(BAYER_RGGB, RGB)].push_back(cv::COLOR_BayerBG2RGB);
        res[std::make_pair(BAYER_RGGB, BGR)].push_back(cv::COLOR_BayerBG2BGR);

        res[std::make_pair(BAYER_BGGR, GRAY)].push_back(cv::COLOR_BayerRG2GRAY);
        res[std::make_pair(BAYER_BGGR, RGB)].push_back(cv::COLOR_BayerRG2RGB);
        res[std::make_pair(BAYER_BGGR, BGR)].push_back(cv::COLOR_BayerRG2BGR);

        res[std::make_pair(BAYER_GBRG, GRAY)].push_back(cv::COLOR_BayerGR2GRAY);
        res[std::make_pair(BAYER_GBRG, RGB)].push_back(cv::COLOR_BayerGR2RGB);
        res[std::make_pair(BAYER_GBRG, BGR)].push_back(cv::COLOR_BayerGR2BGR);

        res[std::make_pair(BAYER_GRBG, GRAY)].push_back(cv::COLOR_BayerGB2GRAY);
        res[std::make_pair(BAYER_GRBG, RGB)].push_back(cv::COLOR_BayerGB2RGB);
        res[std::make_pair(BAYER_GRBG, BGR)].push_back(cv::COLOR_BayerGB2BGR);

        return res;
    }

    static const std::vector<int> getConversionCode(const std::string& SrcEncoding, const std::string& DstEncoding)
    {
        Encoding srcEncod = getEncoding(SrcEncoding);
        Encoding dstEncod = getEncoding(DstEncoding);
        bool isSrcColorFormat = sensor_msgs::image_encodings::isColor(SrcEncoding) ||
            sensor_msgs::image_encodings::isMono(SrcEncoding) || sensor_msgs::image_encodings::isBayer(SrcEncoding) ||
            (SrcEncoding == sensor_msgs::image_encodings::YUV422);
        bool isDstColorFormat = sensor_msgs::image_encodings::isColor(DstEncoding) ||
            sensor_msgs::image_encodings::isMono(DstEncoding) || sensor_msgs::image_encodings::isBayer(DstEncoding) ||
            (DstEncoding == sensor_msgs::image_encodings::YUV422);
        bool isTheSameNumChannels = 
            (sensor_msgs::image_encodings::numChannels(SrcEncoding) == sensor_msgs::image_encodings::numChannels(DstEncoding));

        // If we have no color info in the source, we can only convert to the same format which
        // was resolved in the previous condition. Otherwise, fail
        if (!isSrcColorFormat)
        {
            if (isDstColorFormat)
            {
                throw std::runtime_error("[" + SrcEncoding + "] is not a color format. but [" + DstEncoding +
                            "] is. The conversion does not make sense");
            }
            if (!isTheSameNumChannels)
            {
                throw std::runtime_error("[" + SrcEncoding + "] and [" + DstEncoding +
                    "] do not have the same number of channel");
            }
            return std::vector<int>(1, SAME_FORMAT);
        }

        // If we are converting from a color type to a non color type, we can only do so if we stick
        // to the number of channels
        if (!isDstColorFormat)
        {
            if (!isTheSameNumChannels)
            {
                throw std::runtime_error("[" + SrcEncoding + "] is a color format but [" + DstEncoding + "] " +
                    "is not so they must have the same OpenCV type, CV_8UC3, CV16UC1 ....");
            }

            return std::vector<int>(1, SAME_FORMAT);
        }

        // If we are converting from a color type to another type, then everything is fine
        static const std::map<std::pair<Encoding, Encoding>, std::vector<int> > CONVERSION_CODES = getConversionCodes();

        std::pair<Encoding, Encoding> key(srcEncod, dstEncod);
        std::map<std::pair<Encoding, Encoding>, std::vector<int>>::const_iterator val = CONVERSION_CODES.find(key);
        if (val == CONVERSION_CODES.end())
        {
            throw std::runtime_error("Unsupported conversion from [" + SrcEncoding + "] to [" + DstEncoding + "]");
        }

        // And deal with depth differences if the colors are different
        std::vector<int> res = val->second;
        if ((sensor_msgs::image_encodings::bitDepth(SrcEncoding) != sensor_msgs::image_encodings::bitDepth(DstEncoding)) &&
            (getEncoding(SrcEncoding) != getEncoding(DstEncoding)))
        {
            res.push_back(SAME_FORMAT);
        }

        return res;
    }

    std::shared_ptr<cv::Mat> QrcodePose::toCvMat(const sensor_msgs::Image& Source, const std::string& Encoding)
    {
        auto src = matFromImageMsg(Source);

        // Copy metadata
        auto img = std::make_shared<cv::Mat>();

        if (Encoding.empty() || Encoding == Source.encoding)
        {
            // Copy to new buffer if same encoding requested
            src.copyTo(*img);
        }
        else
        {
            // Convert the source data to the desired encoding
            const std::vector<int> conversionCodes = getConversionCode(Source.encoding, Encoding);
            for (size_t i = 0; i < conversionCodes.size(); ++i)
            {
                int conversionDode = conversionCodes[i];
                if (conversionDode == SAME_FORMAT)
                {
                    // Same number of channels, but different bit depth
                    int srcDepth = sensor_msgs::image_encodings::bitDepth(Source.encoding);
                    int dstDepth = sensor_msgs::image_encodings::bitDepth(Encoding);
                    // Keep the number of channels for now but changed to the final depth
                    int type = CV_MAKETYPE(CV_MAT_DEPTH(getCvType(Encoding)), src.channels());

                    // Do scaling between CV_8U [0,255] and CV_16U [0,65535] images.
                    if (srcDepth == 8 && dstDepth == 16)
                    {
                        src.convertTo(*img, type, 65535. / 255.);
                    }
                    else if (srcDepth == 16 && dstDepth == 8)
                    {
                        src.convertTo(*img, type, 255. / 65535.);
                    }
                    else
                    {
                        src.convertTo(*img, type);
                    }
                }
                else
                {
                    // Perform color conversion
                    cv::cvtColor(src, *img, conversionDode);
                }
            }
        }

        return img;
    }
} // namespace whi_qrcode_pose
