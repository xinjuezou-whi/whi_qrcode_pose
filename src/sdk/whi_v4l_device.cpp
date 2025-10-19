/******************************************************************
v4l camera device under ROS 1

Features:
- v2l camera
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_qrcode_pose/whi_v4l_device.h"

#include <rclcpp/rclcpp.hpp>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

namespace v4l2_camera
{
    static int xioctl(int Handle, int Request, void* Arg)
    {
        int r;

        do
        {
            r = ioctl(Handle, Request, Arg);
        } while (-1 == r && EINTR == errno);

        return r;
    }

    bool V4l2CameraDevice::open()
    {
        fd_ = ::open(device_.c_str(), O_RDWR);
        if (fd_ < 0)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("V4l2CameraDevice"), "\033[1;31m" <<
                "Failed opening device " << device_ << ": " << strerror(errno) << " (" << errno << ")" << "\033[0m");
            return false;
        }

        // List capabilities
        xioctl(fd_, VIDIOC_QUERYCAP, &capabilities_);

        auto canRead = capabilities_.capabilities & V4L2_CAP_READWRITE;
        auto canStream = capabilities_.capabilities & V4L2_CAP_STREAMING;

        RCLCPP_INFO_STREAM(rclcpp::get_logger("V4l2CameraDevice"), "Driver: " << capabilities_.driver);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("V4l2CameraDevice"), "Version: " << capabilities_.version);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("V4l2CameraDevice"), "Device: " << capabilities_.card);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("V4l2CameraDevice"), "Location: " << capabilities_.bus_info);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("V4l2CameraDevice"), "Capabilities:\n" << "  Read/write: " << (canRead ? "YES\n" : "NO\n")
            << "  Streaming: " << (canStream ? "YES" : "NO"));

        // Get current data (pixel) format
        v4l2_format formatReq;
        formatReq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(fd_, VIDIOC_G_FMT, &formatReq);
        cur_data_format_ = PixelFormat(formatReq.fmt.pix);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("V4l2CameraDevice"), "Current pixel format: " << v4l2_fourcc::toString(cur_data_format_.format_) <<
            " @ " << cur_data_format_.width_ << "x" << cur_data_format_.height_);

        // List all available image formats and controls
        listImageFormats();
        listImageSizes();
        listControls();

        RCLCPP_INFO(rclcpp::get_logger("V4l2CameraDevice"), "Available pixel formats:");
        for (auto const & format : image_formats_)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("V4l2CameraDevice"), "  " << v4l2_fourcc::toString(format.format_) << " - " << format.description_);
        }

        if (controls_.empty())
        {
            RCLCPP_INFO(rclcpp::get_logger("V4l2CameraDevice"), "Available controls: none");
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("V4l2CameraDevice"), "Available controls:");
            for (auto const & control : controls_)
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("V4l2CameraDevice"), "  " << control.name_ << " (" << static_cast<unsigned>(control.type_) <<
                    ") = " << getControlValue(control.id_) << (control.inactive_ ? " [inactive]" : ""));
            }
        }

        is_opened_ = true;
        return is_opened_;
    }

    bool V4l2CameraDevice::start()
    {
        RCLCPP_INFO(rclcpp::get_logger("V4l2CameraDevice"), "Starting camera");

        // set to YUYV if it is supported by camera
        auto isYuyv = [](ImageFormat Format) { return Format.format_ == V4L2_PIX_FMT_YUYV; };
        if (auto it = std::find_if(image_formats_.begin(), image_formats_.end(), isYuyv); it != image_formats_.end())
        {
            cur_data_format_.format_ = V4L2_PIX_FMT_YUYV;
            v4l2_format fmtSet;
            fmtSet.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            fmtSet.fmt.pix.width = cur_data_format_.width_;
            fmtSet.fmt.pix.height = cur_data_format_.height_;
            fmtSet.fmt.pix.pixelformat = cur_data_format_.format_;
            fmtSet.fmt.pix.field = V4L2_FIELD_ANY;
            xioctl(fd_, VIDIOC_S_FMT, &fmtSet);
        }

        if (!initMemoryMapping())
        {
            return false;
        }

        // Queue the buffers
        for (auto const& buffer : buffers_)
        {
            v4l2_buffer buf;
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = buffer.index_;
            if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf))
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("V4l2CameraDevice"), "\033[1;31m" <<
                    "Buffer failure on capture start: " << strerror(errno) << " (" << errno << ")" << "\033[0m");
                return false;
            }
        }

        // Start stream
        unsigned type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl(fd_, VIDIOC_STREAMON, &type))
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("V4l2CameraDevice"), "\033[1;31m" <<
                "Failed stream start: " << strerror(errno) << " (" << errno << ")" << "\033[0m");
            return false;
        }
        return true;
    }

    bool V4l2CameraDevice::stop()
    {
        return true;
    }

    std::shared_ptr<cv::Mat> V4l2CameraDevice::capture()
    {
        v4l2_buffer buf;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        // Dequeue buffer with new image
        if (-1 == xioctl(fd_, VIDIOC_DQBUF, &buf))
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("V4l2CameraDevice"), "\033[1;31m" <<
                "Error dequeueing buffer: " << strerror(errno) << " (" << errno << ")" << "\033[0m");
            return nullptr;
        }

        // Requeue buffer to be reused for new captures
        if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf))
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("V4l2CameraDevice"), "\033[1;31m" <<
                "Error re-queueing buffer: " << strerror(errno) << " (" << errno << "\033[0m");
            return nullptr;
        }

        // Fill in remaining image information
        const auto& buffer = buffers_[buf.index];
        if (cur_data_format_.format_ == V4L2_PIX_FMT_YUYV)
        {
            /// output is YUY2
            cv::Mat yuyv(cur_data_format_.height_, cur_data_format_.width_, CV_8UC2, buffer.start_);
            auto bgr = std::make_shared<cv::Mat>(cur_data_format_.height_, cur_data_format_.width_, CV_8UC3);
	        cv::cvtColor(yuyv, *bgr, cv::COLOR_YUV2BGR_YUYV);

            return bgr;
        }
        else if (cur_data_format_.format_ == V4L2_PIX_FMT_UYVY)
        {
            cv::Mat uyvy(cur_data_format_.height_, cur_data_format_.width_, CV_8UC2, buffer.start_);
            auto bgr = std::make_shared<cv::Mat>(cur_data_format_.height_, cur_data_format_.width_, CV_8UC3);
	        cv::cvtColor(uyvy, *bgr, cv::COLOR_YUV2BGR_UYVY);

            return bgr;
        }
        else if (cur_data_format_.format_ == V4L2_PIX_FMT_GREY)
        {

        }
        else if (cur_data_format_.format_ == V4L2_PIX_FMT_MJPEG)
        {

        }
        else
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("V4l2CameraDevice"), "\033[1;33m" <<
                "Current pixel format is not supported yet: " <<
                v4l2_fourcc::toString(cur_data_format_.format_) << " " << cur_data_format_.format_ << "\033[0m");
        }

        return nullptr;
    }

    std::string V4l2CameraDevice::getCameraName() const
    {
        auto name = std::string((char*)(capabilities_.card));
        std::transform(name.begin(), name.end(), name.begin(), ::tolower);
        std::replace(name.begin(), name.end(), ' ', '_');

        return name;
    }

    v4l2_camera::Control V4l2CameraDevice::queryControl(uint32_t Id, bool Silent/* = false*/)
    {
        v4l2_queryctrl queryctrl;
        queryctrl.id = Id;
        if (xioctl(fd_, VIDIOC_QUERYCTRL, &queryctrl) != 0)
        {
            if (!Silent)
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("V4l2CameraDevice"), "\033[1;31m" <<
                    "Failed querying control with ID: " << queryctrl.id << " - " <<
                    strerror(errno) << " (" << errno << ")" << "\033[0m");
            }
            return {};
        }

        std::map<int, std::string> menuItems;
        if (queryctrl.type == (unsigned)ControlType::MENU)
        {
            v4l2_querymenu querymenu;
            querymenu.id = queryctrl.id;

            // Query all enum values
            for (auto i = queryctrl.minimum; i <= queryctrl.maximum; i++)
            {
                querymenu.index = i;
                if (xioctl(fd_, VIDIOC_QUERYMENU, &querymenu) == 0)
                {
                    menuItems[i] = (const char *)querymenu.name;
                }
            }
        }

        Control control;
        control.id_ = queryctrl.id;
        control.name_ = std::string{reinterpret_cast<char *>(queryctrl.name)};
        control.type_ = static_cast<ControlType>(queryctrl.type);
        control.minimum_ = queryctrl.minimum;
        control.maximum_ = queryctrl.maximum;
        control.default_value_ = queryctrl.default_value;
        control.menu_items_map_ = std::move(menuItems);
        control.disabled_ = (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) != 0;
        control.inactive_ = (queryctrl.flags & V4L2_CTRL_FLAG_INACTIVE) != 0;

        return control;
    }

    int32_t V4l2CameraDevice::getControlValue(uint32_t Id) const
    {
        v4l2_control ctrl;
        ctrl.id = Id;
        if (-1 == xioctl(fd_, VIDIOC_G_CTRL, &ctrl))
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("V4l2CameraDevice"), "\033[1;31m" <<
                "Failed getting value for control " << ctrl.id << ": " <<
                strerror(errno) << " (" << errno << "); returning 0!" << "\033[0m");

            return 0;
        }
        return ctrl.value;
    }

    void V4l2CameraDevice::listImageFormats()
    {
        image_formats_.clear();

        struct v4l2_fmtdesc fmtDesc;
        fmtDesc.index = 0;
        fmtDesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        while (xioctl(fd_, VIDIOC_ENUM_FMT, &fmtDesc) == 0)
        {
            image_formats_.emplace_back(fmtDesc);
            fmtDesc.index++;
        }
    }

    void V4l2CameraDevice::listImageSizes()
    {
        image_sizes_.clear();
        struct v4l2_frmsizeenum frameSizeEnum;
        // Supported sizes can be different per format
        for (auto const& format : image_formats_)
        {
            frameSizeEnum.index = 0;
            frameSizeEnum.pixel_format = format.format_;
            if (-1 == xioctl(fd_, VIDIOC_ENUM_FRAMESIZES, &frameSizeEnum))
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("V4l2CameraDevice"), "\033[1;31m" <<
                    "Failed listing frame size " << strerror(errno) << " (" << errno << ")" << "\033[0m");
                continue;
            }

            switch (frameSizeEnum.type)
            {
            case V4L2_FRMSIZE_TYPE_DISCRETE:
                image_sizes_[format.format_] = listDiscreteImageSizes(frameSizeEnum);
                break;
            case V4L2_FRMSIZE_TYPE_STEPWISE:
                image_sizes_[format.format_] = listStepwiseImageSizes(frameSizeEnum);
                break;
            case V4L2_FRMSIZE_TYPE_CONTINUOUS:
                image_sizes_[format.format_] = listContinuousImageSizes(frameSizeEnum);
                break;
            default:
                RCLCPP_WARN_STREAM(rclcpp::get_logger("V4l2CameraDevice"), "\033[1;33m" <<
                    "Frame size type not supported: " << frameSizeEnum.type << "\033[0m");
                continue;
            }
        }
    }

    void V4l2CameraDevice::listControls()
    {
        controls_.clear();

        auto query_id = V4L2_CID_BASE;
        while (true)
        {
            auto control = queryControl(query_id, true);
            if (control.id_ == 0)
            {
                break;
            }

            if (control.disabled_)
            {
                query_id = control.id_ |= V4L2_CTRL_FLAG_NEXT_CTRL;
                continue;
            }

            controls_.push_back(control);

            // Get ready to query next item
            query_id = control.id_ |= V4L2_CTRL_FLAG_NEXT_CTRL;
        }
    }

    V4l2CameraDevice::ImageSizesDescription V4l2CameraDevice::listDiscreteImageSizes(v4l2_frmsizeenum FrameSizeEnum)
    {
        ImageSizesVector sizes;
        do
        {
            sizes.emplace_back(std::make_pair(FrameSizeEnum.discrete.width, FrameSizeEnum.discrete.height));
            FrameSizeEnum.index++;
        } while (xioctl(fd_, VIDIOC_ENUM_FRAMESIZES, &FrameSizeEnum) == 0);

        return std::make_pair(ImageSizeType::DISCRETE, std::move(sizes));
    }

    V4l2CameraDevice::ImageSizesDescription V4l2CameraDevice::listStepwiseImageSizes(v4l2_frmsizeenum FrameSizeEnum)
    {
        // Three entries: min size, max size and stepsize
        auto sizes = ImageSizesVector(3);
        sizes[0] = std::make_pair(FrameSizeEnum.stepwise.min_width, FrameSizeEnum.stepwise.min_height);
        sizes[1] = std::make_pair(FrameSizeEnum.stepwise.max_width, FrameSizeEnum.stepwise.max_height);
        sizes[2] = std::make_pair(FrameSizeEnum.stepwise.step_width, FrameSizeEnum.stepwise.step_height);

        return std::make_pair(ImageSizeType::STEPWISE, std::move(sizes));
    }

    V4l2CameraDevice::ImageSizesDescription V4l2CameraDevice::listContinuousImageSizes(v4l2_frmsizeenum FrameSizeEnum)
    {
        // Two entries: min size and max size, stepsize is implicitly 1
        auto sizes = ImageSizesVector(2);
        sizes[0] = std::make_pair(FrameSizeEnum.stepwise.min_width, FrameSizeEnum.stepwise.min_height);
        sizes[1] = std::make_pair(FrameSizeEnum.stepwise.max_width, FrameSizeEnum.stepwise.max_height);

        return std::make_pair(ImageSizeType::CONTINUOUS, std::move(sizes));
    }

    bool V4l2CameraDevice::initMemoryMapping()
    {
        v4l2_requestbuffers req;
        req.count = 4; // request 4 buffers
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        xioctl(fd_, VIDIOC_REQBUFS, &req);
        if (req.count < 2)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("V4l2CameraDevice"), "\033[1;31m" <<
                "insufficient buffer memory" << "\033[0m");
            return false;
        }

        buffers_.resize(req.count);
        for (auto i = 0u; i < req.count; ++i)
        {
            v4l2_buffer buf;
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            xioctl(fd_, VIDIOC_QUERYBUF, &buf);

            buffers_[i].index_ = buf.index;
            buffers_[i].length_ = buf.length;
            buffers_[i].start_ = (uint8_t*)mmap(NULL /* start anywhere */,
                                                buf.length,
                                                PROT_READ | PROT_WRITE /* required */,
                                                MAP_SHARED /* recommended */,
                                                fd_, buf.m.offset);

            if (MAP_FAILED == buffers_[i].start_)
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("V4l2CameraDevice"), "\033[1;31m" <<
                    "Failed mapping device memory" << "\033[0m");
                return false;
            }
        }

        return true;
    }
}  // namespace v4l2_camera
