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
    virtual std::vector<double> getIntrinsicProjection() const = 0;
    virtual std::vector<double> getIntrinsicDistortion() const = 0;
    virtual void setIntrinsicProjection(const std::vector<double>& Projection) = 0;
    virtual void setIntrinsicDistortion(const std::vector<double>& Distortion) = 0;

protected:
    bool is_opened_{ false };
};
