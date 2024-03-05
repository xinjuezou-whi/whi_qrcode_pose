/******************************************************************
images from path under ROS 1

Features:
- images in folder
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-07-15: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_base_camera.h"

#include <string>
#include <vector>
#include <map>

namespace images_from_path
{
    class ImagePathDevice : public WhiCamera
    {
    public:
        explicit ImagePathDevice(const std::string& Path)
            : WhiCamera(), path_(Path) {};
        virtual ~ImagePathDevice() = default;

        bool open() override;
        bool start() override;
        bool stop() override;
        std::shared_ptr<cv::Mat> capture() override;
        std::string getCameraName() const override;
        std::vector<double> getIntrinsicProjection() const override;
        std::vector<double> getIntrinsicDistortion() const override;
        void setIntrinsicProjection(const std::vector<double>& Projection) override;
        void setIntrinsicDistortion(const std::vector<double>& Distortion) override;

    private:
        std::string path_;
        std::vector<std::string> images_;
        std::vector<double> intrinsic_projection_;
        std::vector<double> intrinsic_distortion_;
    };
}  // namespace images_from_path
