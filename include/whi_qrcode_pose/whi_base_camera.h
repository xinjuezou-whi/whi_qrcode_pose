/******************************************************************
abstract camera interface under ROS 1

Features:
- abstract interfaces
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-07-26: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <opencv2/opencv.hpp>

class WhiCamera
{
public:
    WhiCamera() = default;
    virtual ~WhiCamera() = default;

public:
    virtual bool open() = 0;
    virtual bool start() = 0;
    virtual bool stop() = 0;
    virtual std::shared_ptr<cv::Mat> capture() = 0;
    virtual std::string getCameraName() const = 0;
    std::vector<double> getIntrinsicProjection() const
    {
        return intrinsic_projection_;
    };
    std::vector<double> getIntrinsicDistortion() const
    {
        return intrinsic_distortion_;
    };
    void setIntrinsicProjection(const std::vector<double>& Projection)
    {
        intrinsic_projection_ = Projection;
    };
    void setIntrinsicDistortion(const std::vector<double>& Distortion)
    {
        intrinsic_distortion_ = Distortion;
    };

protected:
    bool is_opened_{ false };
    std::vector<double> intrinsic_projection_;
    std::vector<double> intrinsic_distortion_;
};
