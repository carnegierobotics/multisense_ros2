/**
 * @file camera.cpp
 *
 * Copyright 2020
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Robotics, LLC nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CARNEGIE ROBOTICS, LLC BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

#include <arpa/inet.h>
#include <chrono>
#include <fstream>
#include <sstream>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include <Eigen/Geometry>
#include <sensor_msgs/image_encodings.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <multisense_ros/multisense.h>

#include <MultiSense/MultiSenseSerialization.hh>
#include <MultiSense/MultiSenseUtilities.hh>

namespace lms = multisense;

using namespace std::chrono_literals;

namespace multisense_ros {

namespace { // anonymous

struct ParameterStep
{
    double offset = 0.0;
    double delta = 0.0;
};

rclcpp::Time create_time(const lms::ImuSample &sample,
                         const TimestampSource &time_source,
                         const std::optional<std::chrono::nanoseconds> &camera_host_offset)
{
    switch (time_source)
    {
        case TimestampSource::CAMERA:
        {
            return rclcpp::Time(sample.sample_time.time_since_epoch().count());
        }
        case TimestampSource::SYSTEM:
        {
            return (camera_host_offset && (camera_host_offset.value() + sample.sample_time.time_since_epoch()) > 0s) ?
                rclcpp::Time(camera_host_offset->count() + sample.sample_time.time_since_epoch().count()) :
                rclcpp::Time(sample.sample_time.time_since_epoch().count());

        }
        case TimestampSource::PTP:
        {
            return rclcpp::Time(sample.ptp_sample_time.time_since_epoch().count());
        }
        default:
        {
            throw std::runtime_error("Unknown time source");
        }
    }

    return rclcpp::Time(0);
}

rclcpp::Time create_time(const lms::ImageFrame &frame,
                         const TimestampSource &time_source,
                         const std::optional<std::chrono::nanoseconds> &camera_host_offset)
{
    switch (time_source)
    {
        case TimestampSource::CAMERA:
        {
            return rclcpp::Time(frame.frame_time.time_since_epoch().count());
        }
        case TimestampSource::SYSTEM:
        {
            return (camera_host_offset && (camera_host_offset.value() + frame.frame_time.time_since_epoch()) > 0s) ?
                rclcpp::Time(camera_host_offset->count() + frame.frame_time.time_since_epoch().count()) :
                rclcpp::Time(frame.frame_time.time_since_epoch().count());

        }
        case TimestampSource::PTP:
        {
            return rclcpp::Time(frame.ptp_frame_time.time_since_epoch().count());
        }
        default:
        {
            throw std::runtime_error("Unkown time source");
        }
    }

    return rclcpp::Time(0);
}

bool is_time_valid(const lms::ImuSample &sample, const TimestampSource &time_source)
{
    return time_source == TimestampSource::PTP ? sample.ptp_sample_time.time_since_epoch().count() >= 0 :
                          sample.sample_time.time_since_epoch().count() > 0;
}

bool is_time_valid(const lms::ImageFrame &frame, const TimestampSource &time_source)
{
    return time_source == TimestampSource::PTP ? frame.ptp_frame_time.time_since_epoch().count() >= 0 :
                          frame.frame_time.time_since_epoch().count() > 0;
}

bool is_gen2_camera(const lms::MultiSenseInfo::DeviceInfo::HardwareRevision &camera_type)
{
    return (camera_type == lms::MultiSenseInfo::DeviceInfo::HardwareRevision::S27 ||
            camera_type == lms::MultiSenseInfo::DeviceInfo::HardwareRevision::S30 ||
            camera_type == lms::MultiSenseInfo::DeviceInfo::HardwareRevision::KS21 ||
            camera_type == lms::MultiSenseInfo::DeviceInfo::HardwareRevision::KS21_SILVER ||
            camera_type == lms::MultiSenseInfo::DeviceInfo::HardwareRevision::KS21i);
}

std::chrono::microseconds get_closest_exposure(uint32_t exposure_us, uint32_t min, double step)
{
    if (step == 0.0)
    {
        std::cerr << "Exposure step size is invalid. Cannot find closest exposure" << std::endl;
        return std::chrono::microseconds(exposure_us);

    }

    return std::chrono::microseconds{static_cast<uint32_t>(std::floor((std::floor(static_cast<double>(exposure_us - min) / step) * step) + min))};
}

float get_closet_aux_gain(float gain)
{

    constexpr std::array<float, 86> gains = {1.00 ,1.03 ,1.07 ,1.10 ,1.14 ,1.19 ,1.23 ,1.28 ,1.33 ,1.39 ,1.45 ,1.52,
                                             1.60 ,1.68 ,1.78 ,1.88 ,2.00 ,2.13 ,2.29 ,2.46 ,2.67 ,2.91 ,3.00 ,3.10,
                                             3.20 ,3.31 ,3.43 ,3.56 ,3.69 ,3.84 ,4.00 ,4.17 ,4.36 ,4.57 ,4.80 ,5.05,
                                             5.33 ,5.65 ,6.00 ,6.40 ,6.86 ,7.38 ,8.00 ,8.73 ,9.60 ,10.67,12.00,12.39,
                                             12.80,13.24,13.71,14.22,14.77,15.36,16.00,16.70,17.45,18.29,19.20,20.21,
                                             21.33,22.59,24.00,24.77,25.60,26.48,27.43,28.44,29.54,30.72,32.00,33.39,
                                             34.91,36.57,38.40,40.42,42.67,45.18,48.00,51.20,54.86,59.08,64.00,69.82,
                                             76.80,85.33};

    const auto val = std::lower_bound(std::begin(gains), std::end(gains), gain);

    if (val == std::begin(gains))
    {
        return *val;
    }

    if (val == std::end(gains))
    {
        return gains.back();
    }

    const auto before = std::prev(val);
    return (std::abs(*val - gain) < std::abs(*before - gain)) ? *val : *before;
}

float get_closest_stereo_gain(float gain)
{
    constexpr std::array<float, 36> gains{1.68421,  1.77778,  1.88235, 2.00000,  2.13333,  2.28572,  2.46154,
                                          2.66667,  2.90909,  3.20000,  3.55556, 4.00000,  4.12903,  4.26667,
                                          4.41379,  4.57144,  4.74074,  4.92308,  5.12000, 5.33333,  5.56522,
                                          5.81818,  6.09524,  6.40000,  6.73684,  7.11111,  7.52941, 8.00000,
                                          8.53333,  9.14286,  9.84616, 10.66667, 11.63636, 12.90000, 14.22222,
                                          16.00000};

    const auto val = std::lower_bound(std::begin(gains), std::end(gains), gain);

    if (val == std::begin(gains))
    {
        return *val;
    }

    if (val == std::end(gains))
    {
        return gains.back();
    }

    const auto before = std::prev(val);
    return (std::abs(*val - gain) < std::abs(*before - gain)) ? *val : *before;
}

tf2::Matrix3x3 to_rotation(const std::array<std::array<float, 3>, 3> & R)
{
    return tf2::Matrix3x3{R[0][0], R[0][1], R[0][2],
                          R[1][0], R[1][1], R[1][2],
                          R[2][0], R[2][1], R[2][2]};
}

lms::CameraCalibration scale_calibration(lms::CameraCalibration calibration, double x_scale, double y_scale)
{
    calibration.K[0][0] *= x_scale;
    calibration.K[0][2] *= x_scale;
    calibration.K[1][1] *= y_scale;
    calibration.K[1][2] *= y_scale;

    calibration.P[0][0] *= x_scale;
    calibration.P[0][2] *= x_scale;
    calibration.P[0][3] *= x_scale;
    calibration.P[1][1] *= y_scale;
    calibration.P[1][2] *= y_scale;
    calibration.P[1][3] *= y_scale;

    return calibration;
}

lms::StereoCalibration scale_calibration(lms::StereoCalibration calibration, double x_scale, double y_scale)
{
    calibration.left = scale_calibration(std::move(calibration.left), x_scale, y_scale);
    calibration.right = scale_calibration(std::move(calibration.right), x_scale, y_scale);

    if (calibration.aux)
    {
        calibration.aux = scale_calibration(std::move(calibration.aux.value()), x_scale, y_scale);
    }
    return calibration;
}

std_msgs::msg::Header create_header(const rclcpp::Time &stamp, const std::string &frame_id)
{
    std_msgs::msg::Header header;

    header.stamp = stamp;
    header.frame_id = frame_id;

    return header;
}

sensor_msgs::msg::CameraInfo create_camera_info(const lms::CameraCalibration &cal,
                                                const std_msgs::msg::Header &header,
                                                uint32_t width,
                                                uint32_t height)
{
    sensor_msgs::msg::CameraInfo info;

    info.width = width;
    info.height = height;

    info.header = header;

    switch (cal.distortion_type)
    {
        case lms::CameraCalibration::DistortionType::PLUMBBOB:
        {
            info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
            break;
        }
        case lms::CameraCalibration::DistortionType::RATIONAL_POLYNOMIAL:
        {
            info.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
            break;
        }
        default: {break;}
    }

    //
    // We need to convert our floating point calibrations to doubles

    for (const auto &e: cal.D)
    {
        info.d.push_back(e);
    }

    for (size_t i = 0 ; i < info.k.size() ; ++i)
    {
        info.k[i] = *(reinterpret_cast<const float*>(cal.K.data()) + i);
    }

    for (size_t i = 0 ; i < info.r.size() ; ++i)
    {
        info.r[i] = *(reinterpret_cast<const float*>(cal.R.data()) + i);
    }

    for (size_t i = 0 ; i < info.p.size() ; ++i)
    {
        info.p[i] = *(reinterpret_cast<const float*>(cal.P.data()) + i);
    }

    return info;
}

void populate_image(const lms::Image &image,
                    sensor_msgs::msg::Image &ros_image,
                    const std::string &frame_id,
                    const rclcpp::Time &ros_time)
{
    ros_image.data.resize(image.image_data_length);
    memcpy(&ros_image.data[0], image.raw_data->data() + image.image_data_offset, image.image_data_length);

    ros_image.header.frame_id = frame_id;
    ros_image.header.stamp = ros_time;
    ros_image.height = image.height;
    ros_image.width = image.width;

    switch(image.format)
    {
        case lms::Image::PixelFormat::MONO8:
        {
            ros_image.encoding = sensor_msgs::image_encodings::MONO8;
            ros_image.step = image.width;
            break;
        }
        case lms::Image::PixelFormat::BGR8:
        {
            ros_image.encoding = sensor_msgs::image_encodings::BGR8;
            ros_image.step = image.width * 3;
            break;
        }
        case lms::Image::PixelFormat::MONO16:
        {
            ros_image.encoding = sensor_msgs::image_encodings::MONO16;
            ros_image.step = image.width * 2;
            break;
        }
        case lms::Image::PixelFormat::FLOAT32:
        {
            ros_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            ros_image.step = image.width * 4;
            break;
        }
        default: {throw std::runtime_error("Invalid image format");}
    };

    ros_image.is_bigendian = (htonl(1) == 1);
}

void publish_image(const lms::Image &image,
                   std::shared_ptr<ImagePublisher> publisher,
                   sensor_msgs::msg::Image &ros_image,
                   const std::string &frame_id,
                   const rclcpp::Time &ros_time)
{
    populate_image(image, ros_image, frame_id, ros_time);

    auto camera_info = create_camera_info(image.calibration, ros_image.header, image.width, image.height);
    publisher->publish(std::make_unique<sensor_msgs::msg::Image>(ros_image),
                       std::make_unique<sensor_msgs::msg::CameraInfo>(std::move(camera_info)));
}

template <typename Color>
void publish_point_cloud(const lms::PointCloud<Color> &point_cloud,
                         const rclcpp::Time &ros_time,
                         rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
                         sensor_msgs::msg::PointCloud2 &ros_point_cloud,
                         const std::string &frame_id)
{
    ros_point_cloud.is_bigendian = (htonl(1) == 1);
    ros_point_cloud.is_dense = true;
    ros_point_cloud.header.frame_id = frame_id;
    ros_point_cloud.fields.reserve(4);
    ros_point_cloud.fields.resize(3);
    ros_point_cloud.point_step = 0;
    ros_point_cloud.fields[0].name     = "x";
    ros_point_cloud.fields[0].offset   = 0;
    ros_point_cloud.fields[0].count    = 1;
    ros_point_cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    ros_point_cloud.point_step += sizeof(float);
    ros_point_cloud.fields[1].name     = "y";
    ros_point_cloud.fields[1].offset   = sizeof(float);
    ros_point_cloud.fields[1].count    = 1;
    ros_point_cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    ros_point_cloud.point_step += sizeof(float);
    ros_point_cloud.fields[2].name     = "z";
    ros_point_cloud.fields[2].offset   = 2 * sizeof(float);
    ros_point_cloud.fields[2].count    = 1;
    ros_point_cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    ros_point_cloud.point_step += sizeof(float);

    if constexpr (std::is_same_v<Color, std::array<uint8_t, 3>>)
    {
        ros_point_cloud.fields.resize(4);
        ros_point_cloud.fields[3].name = "rgb";
        ros_point_cloud.fields[3].offset = 3 * sizeof(float);
        ros_point_cloud.fields[3].count = 1;
        ros_point_cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        ros_point_cloud.point_step += sizeof(float);

        const size_t num_fields = ros_point_cloud.fields.size();

        //
        // Unfortunately we need to iterate and write our points to the output buffer since we need to
        // pad color pixels with an extra 8 bits to align with the FLOAT32 boundary
        ros_point_cloud.data.resize(point_cloud.cloud.size() * (ros_point_cloud.point_step));
        auto point_cloud_p = reinterpret_cast<float*>(ros_point_cloud.data.data());
        for (size_t i = 0; i < point_cloud.cloud.size(); ++i)
        {
            point_cloud_p[i * num_fields + 0] = point_cloud.cloud[i].x;
            point_cloud_p[i * num_fields + 1] = point_cloud.cloud[i].y;
            point_cloud_p[i * num_fields + 2] = point_cloud.cloud[i].z;

            const uint32_t rgb = static_cast<uint32_t>(0) |
                                 (static_cast<uint32_t>(point_cloud.cloud[i].color[2]) << 16) |
                                 (static_cast<uint32_t>(point_cloud.cloud[i].color[1]) << 8) |
                                 (static_cast<uint32_t>(point_cloud.cloud[i].color[0]));

            auto color_p = reinterpret_cast<uint32_t*>(&point_cloud_p[i * num_fields + 3]);
            color_p[0] = rgb;
        }
    }
    else if constexpr (std::is_same_v<Color, uint8_t>)
    {
        ros_point_cloud.fields.resize(4);
        ros_point_cloud.fields[3].name = "intensity";
        ros_point_cloud.fields[3].offset = 3 * sizeof(float);
        ros_point_cloud.fields[3].count = 1;
        ros_point_cloud.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
        ros_point_cloud.point_step += sizeof(uint8_t);

        ros_point_cloud.data.resize(point_cloud.cloud.size() * sizeof(lms::Point<Color>));
        memcpy(&ros_point_cloud.data[0], reinterpret_cast<const uint8_t*>(point_cloud.cloud.data()), ros_point_cloud.data.size());
    }
    else if constexpr (std::is_same_v<Color, uint16_t>)
    {
        ros_point_cloud.fields.resize(4);
        ros_point_cloud.fields[3].name = "intensity";
        ros_point_cloud.fields[3].offset = 3 * sizeof(float);
        ros_point_cloud.fields[3].count = 1;
        ros_point_cloud.fields[3].datatype = sensor_msgs::msg::PointField::UINT16;
        ros_point_cloud.point_step += sizeof(uint16_t);

        ros_point_cloud.data.resize(point_cloud.cloud.size() * sizeof(lms::Point<Color>));
        memcpy(&ros_point_cloud.data[0], reinterpret_cast<const uint8_t*>(point_cloud.cloud.data()), ros_point_cloud.data.size());
    }
    else if constexpr (std::is_same_v<Color, void>)
    {
        ros_point_cloud.data.resize(point_cloud.cloud.size() * sizeof(lms::Point<Color>));
        memcpy(&ros_point_cloud.data[0], reinterpret_cast<const uint8_t*>(point_cloud.cloud.data()), ros_point_cloud.data.size());
    }

    ros_point_cloud.width = point_cloud.cloud.size();
    ros_point_cloud.height = 1;
    ros_point_cloud.row_step = point_cloud.cloud.size() * ros_point_cloud.point_step;

    ros_point_cloud.header.frame_id = frame_id;
    ros_point_cloud.header.stamp = ros_time;

    publisher->publish(std::make_unique<sensor_msgs::msg::PointCloud2>(ros_point_cloud));
}

void publish_imu_sample(const lms::ImuSample &sample,
                        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher,
                        const std::string &frame_id,
                        const rclcpp::Time &ros_time)
{
    constexpr double GRAVITY = 9.80665;

    sensor_msgs::msg::Imu message{};

    message.header.frame_id = frame_id;
    message.header.stamp = ros_time;

    if (sample.accelerometer)
    {
        message.linear_acceleration.x = sample.accelerometer->x * GRAVITY;
        message.linear_acceleration.y = sample.accelerometer->y * GRAVITY;
        message.linear_acceleration.z = sample.accelerometer->z * GRAVITY;
    }

    if (sample.gyroscope)
    {
        message.angular_velocity.x = sample.gyroscope->x * M_PI/180.0;
        message.angular_velocity.y = sample.gyroscope->y * M_PI/180.0;
        message.angular_velocity.z = sample.gyroscope->z * M_PI/180.0;
    }

    publisher->publish(message);
}

template <typename ImageParamsT>
multisense::MultiSenseConfig::ImageConfig update_param_config(multisense::MultiSenseConfig::ImageConfig config,
                                                              const ImageParamsT &params,
                                                              const std::optional<ParameterStep> &exposure_step,
                                                              bool aux_camera)
{
    config.gamma = params.gamma;
    config.auto_exposure_enabled = params.auto_exposure_enabled;

    if (config.auto_exposure_enabled && config.auto_exposure)
    {
        config.auto_exposure->max_exposure_time = std::chrono::microseconds{params.auto_exposure.max_exposure_time};
        config.auto_exposure->decay = params.auto_exposure.decay;
        config.auto_exposure->target_intensity = params.auto_exposure.target_intensity;
        config.auto_exposure->target_threshold = params.auto_exposure.target_threshold;
        config.auto_exposure->max_gain = params.auto_exposure.max_gain;

        config.auto_exposure->roi.top_left_x_position = params.auto_exposure.roi[0];
        config.auto_exposure->roi.top_left_y_position = params.auto_exposure.roi[1];
        config.auto_exposure->roi.width = params.auto_exposure.roi[2];
        config.auto_exposure->roi.height = params.auto_exposure.roi[3];
    }
    else if (config.manual_exposure)
    {
        config.manual_exposure->exposure_time = exposure_step ?
                                                get_closest_exposure(params.manual_exposure.exposure_time,
                                                                     exposure_step->offset,
                                                                     exposure_step->delta) :
                                                std::chrono::microseconds{params.manual_exposure.exposure_time};

        config.manual_exposure->gain = aux_camera ?
                                       get_closet_aux_gain(params.manual_exposure.gain) :
                                       get_closest_stereo_gain(params.manual_exposure.gain);
    }

    config.auto_white_balance_enabled = params.auto_white_balance_enabled;

    if (!config.auto_white_balance_enabled && config.manual_white_balance)
    {
        config.manual_white_balance->red = params.manual_white_balance.red;
        config.manual_white_balance->blue = params.manual_white_balance.blue;
    }

    if (config.auto_white_balance_enabled && config.auto_white_balance)
    {
        config.auto_white_balance->decay = params.auto_white_balance.decay;
        config.auto_white_balance->threshold = params.auto_white_balance.threshold;
    }

    return config;
}

multisense::MultiSenseConfig update_param_config(multisense::MultiSenseConfig config,
                                                 const multisense::Params &params,
                                                 const lms::MultiSenseInfo::DeviceInfo::HardwareRevision &camera_type)
{
    config.frames_per_second = params.fps;

    config.stereo_config.postfilter_strength = params.stereo.postfilter_strength;

    config.image_config = update_param_config(std::move(config.image_config),
                                              params.image,
                                              is_gen2_camera(camera_type) ?
                                                std::make_optional(ParameterStep{20.0, 10.0125}) : std::nullopt,
                                              false);

    if (config.aux_config)
    {
        config.aux_config->image_config = update_param_config(std::move(config.aux_config->image_config),
                                                              params.aux.image,
                                                              is_gen2_camera(camera_type) ?
                                                                std::make_optional(ParameterStep{20.0, 10.096875}) :
                                                                std::nullopt,
                                                              true);
    }

    if (config.time_config)
    {
        config.time_config->ptp_enabled = params.time.ptp_enabled;
    }

    if (config.network_config)
    {
        config.network_config->packet_delay_enabled = params.network.packet_delay_enabled;
    }

    if (config.imu_config)
    {
        config.imu_config->samples_per_frame = params.imu.samples_per_frame;
    }

    return config;
}

uint32_t disparity_pixels(const multisense::MultiSenseConfig::MaxDisparities &disparity)
{
    switch (disparity)
    {
        case multisense::MultiSenseConfig::MaxDisparities::D64:
        {
            return 64;
        }
        case multisense::MultiSenseConfig::MaxDisparities::D128:
        {
            return 128;
        }
        case multisense::MultiSenseConfig::MaxDisparities::D256:
        {
            return 256;
        }
    }

    throw std::runtime_error("unsupported disparity setting");
}

multisense::MultiSenseConfig::MaxDisparities pixel_to_disparity(const uint32_t disparity)
{
    if (disparity == 64)
    {
        return multisense::MultiSenseConfig::MaxDisparities::D64;
    }
    else if (disparity == 128)
    {
        return multisense::MultiSenseConfig::MaxDisparities::D128;
    }
    else if (disparity == 256)
    {
        return multisense::MultiSenseConfig::MaxDisparities::D256;
    }

    throw std::runtime_error("unsupported disparity setting");
}

} // anonymous

//
// Provide compiler with definition of the static members

constexpr char MultiSense::LEFT[];
constexpr char MultiSense::RIGHT[];
constexpr char MultiSense::AUX[];
constexpr char MultiSense::IMU[];

constexpr char MultiSense::LEFT_CAMERA_FRAME[];
constexpr char MultiSense::RIGHT_CAMERA_FRAME[];
constexpr char MultiSense::LEFT_RECTIFIED_FRAME[];
constexpr char MultiSense::RIGHT_RECTIFIED_FRAME[];
constexpr char MultiSense::AUX_CAMERA_FRAME[];
constexpr char MultiSense::AUX_RECTIFIED_FRAME[];
constexpr char MultiSense::IMU_FRAME[];

constexpr char MultiSense::INFO_TOPIC[];
constexpr char MultiSense::STATUS_TOPIC[];
constexpr char MultiSense::HISTOGRAM_TOPIC[];
constexpr char MultiSense::RAW_CONFIG_TOPIC[];
constexpr char MultiSense::MONO_TOPIC[];
constexpr char MultiSense::RECT_TOPIC[];
constexpr char MultiSense::DISPARITY_TOPIC[];
constexpr char MultiSense::DISPARITY_IMAGE_TOPIC[];
constexpr char MultiSense::DEPTH_TOPIC[];
constexpr char MultiSense::OPENNI_DEPTH_TOPIC[];
constexpr char MultiSense::COST_TOPIC[];
constexpr char MultiSense::COLOR_TOPIC[];
constexpr char MultiSense::RECT_COLOR_TOPIC[];
constexpr char MultiSense::POINTCLOUD_TOPIC[];
constexpr char MultiSense::LUMA_POINTCLOUD_TOPIC[];
constexpr char MultiSense::COLOR_POINTCLOUD_TOPIC[];
constexpr char MultiSense::ORGANIZED_POINTCLOUD_TOPIC[];
constexpr char MultiSense::COLOR_ORGANIZED_POINTCLOUD_TOPIC[];
constexpr char MultiSense::MONO_CAMERA_INFO_TOPIC[];
constexpr char MultiSense::RECT_CAMERA_INFO_TOPIC[];
constexpr char MultiSense::COLOR_CAMERA_INFO_TOPIC[];
constexpr char MultiSense::RECT_COLOR_CAMERA_INFO_TOPIC[];
constexpr char MultiSense::DEPTH_CAMERA_INFO_TOPIC[];
constexpr char MultiSense::DISPARITY_CAMERA_INFO_TOPIC[];
constexpr char MultiSense::COST_CAMERA_INFO_TOPIC[];
constexpr char MultiSense::IMU_TOPIC[];

MultiSense::MultiSense(const std::string& node_name,
               const rclcpp::NodeOptions& options,
               std::unique_ptr<multisense::Channel> channel,
               const std::string& tf_prefix,
               bool use_image_transport,
               bool use_sensor_qos):
    Node(node_name, options),
    channel_(std::move(channel)),
    left_node_(create_sub_node(LEFT)),
    right_node_(create_sub_node(RIGHT)),
    aux_node_(create_sub_node(AUX)),
    imu_node_(create_sub_node(IMU)),
    frame_id_left_(tf_prefix + LEFT_CAMERA_FRAME),
    frame_id_right_(tf_prefix + RIGHT_CAMERA_FRAME),
    frame_id_aux_(tf_prefix + AUX_CAMERA_FRAME),
    frame_id_rectified_left_(tf_prefix + LEFT_RECTIFIED_FRAME),
    frame_id_rectified_right_(tf_prefix + RIGHT_RECTIFIED_FRAME),
    frame_id_rectified_aux_(tf_prefix + AUX_RECTIFIED_FRAME),
    frame_id_imu_(tf_prefix + IMU_FRAME),
    static_tf_broadcaster_(std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this))
{
    if (!channel_)
    {
        throw std::runtime_error("Invalid channel");
    }

    //
    // All streams off

    stop();

    info_ = channel_->get_info();

    has_aux_camera_ = info_.device.has_aux_camera();

    //
    // Topics published for all device types

    const auto latching_qos = rclcpp::QoS(1).transient_local();
    auto default_qos = rclcpp::SystemDefaultsQoS();
    default_qos.keep_last(1);

    histogram_pub_ = create_publisher<multisense_msgs::msg::Histogram>(HISTOGRAM_TOPIC, default_qos);
    info_pub_ = create_publisher<multisense_msgs::msg::Info>(INFO_TOPIC, latching_qos);
    config_pub_ = create_publisher<std_msgs::msg::String>(RAW_CONFIG_TOPIC, latching_qos);
    status_pub_ = create_publisher<multisense_msgs::msg::Status>(STATUS_TOPIC, default_qos);

    const auto now = rclcpp::Clock().now();
    const auto left_header = create_header(now, frame_id_left_);
    const auto left_rect_header = create_header(now, frame_id_rectified_left_);
    const auto right_header = create_header(now, frame_id_right_);
    const auto right_rect_header = create_header( now, frame_id_rectified_right_);

    const auto config = channel_->get_config();
    const double x_scale = static_cast<double>(config.width) / static_cast<double>(info_.device.imager_width);
    const double y_scale = static_cast<double>(config.height) / static_cast<double>(info_.device.imager_height);
    const auto calibration = scale_calibration(channel_->get_calibration(), x_scale, y_scale);
    const auto left_cal = create_camera_info(calibration.left, left_header, config.width, config.height);
    const auto left_rect_cal = create_camera_info(calibration.left, left_rect_header, config.width, config.height);
    const auto right_cal = create_camera_info(calibration.right, right_header, config.width, config.height);
    const auto right_rect_cal = create_camera_info(calibration.right, right_rect_header, config.width, config.height);

    //
    // Image publishers

    const rclcpp::QoS sensor_data_qos = rclcpp::SensorDataQoS();
    const auto qos = use_sensor_qos ? sensor_data_qos : default_qos;

    using ds = lms::DataSource;

    left_mono_cam_pub_ = std::make_shared<ImagePublisher>(left_node_,
                                                          MONO_TOPIC,
                                                          left_cal,
                                                          qos,
                                                          create_publisher_options({ds::LEFT_MONO_RAW},
                                                                                   get_full_topic_name(left_node_, MONO_TOPIC)),
                                                          use_image_transport);

    right_mono_cam_pub_ = std::make_shared<ImagePublisher>(right_node_,
                                                           MONO_TOPIC,
                                                           right_cal,
                                                           qos,
                                                           create_publisher_options({ds::RIGHT_MONO_RAW},
                                                                                   get_full_topic_name(right_node_, MONO_TOPIC)),
                                                           use_image_transport);

    left_rect_cam_pub_ = std::make_shared<ImagePublisher>(left_node_,
                                                          RECT_TOPIC,
                                                          left_rect_cal,
                                                          qos,
                                                          create_publisher_options({ds::LEFT_RECTIFIED_RAW},
                                                                                   get_full_topic_name(left_node_, RECT_TOPIC)),
                                                          use_image_transport);

    right_rect_cam_pub_ = std::make_shared<ImagePublisher>(right_node_,
                                                           RECT_TOPIC,
                                                           right_rect_cal,
                                                           qos,
                                                           create_publisher_options({ds::RIGHT_RECTIFIED_RAW},
                                                                                   get_full_topic_name(right_node_, RECT_TOPIC)),
                                                           use_image_transport);

    depth_cam_pub_ = std::make_shared<ImagePublisher>(left_node_,
                                                      DEPTH_TOPIC,
                                                      left_rect_cal,
                                                      qos,
                                                      create_publisher_options({ds::LEFT_DISPARITY_RAW},
                                                                               get_full_topic_name(left_node_, DEPTH_TOPIC)),
                                                      use_image_transport);

    ni_depth_cam_pub_ = std::make_shared<ImagePublisher>(left_node_,
                                                         OPENNI_DEPTH_TOPIC,
                                                         left_rect_cal,
                                                         qos,
                                                         create_publisher_options({ds::LEFT_DISPARITY_RAW},
                                                                                  get_full_topic_name(left_node_, OPENNI_DEPTH_TOPIC)),
                                                         use_image_transport);

    left_disparity_pub_ = std::make_shared<ImagePublisher>(left_node_,
                                                           DISPARITY_TOPIC,
                                                           left_rect_cal,
                                                           qos,
                                                           create_publisher_options({ds::LEFT_DISPARITY_RAW},
                                                                                    get_full_topic_name(left_node_, DISPARITY_TOPIC)),
                                                           use_image_transport);

    left_disparity_cost_pub_ = std::make_shared<ImagePublisher>(left_node_,
                                                                COST_TOPIC,
                                                                left_rect_cal,
                                                                qos,
                                                                create_publisher_options({ds::COST_RAW},
                                                                                         get_full_topic_name(left_node_, COST_TOPIC)),
                                                                use_image_transport);

    if (has_aux_camera_)
    {
        if (!calibration.aux)
        {
            throw std::runtime_error("Invalid aux calibration");
        }

        const auto aux_header = create_header(now, frame_id_aux_);
        const auto aux_rect_header = create_header(now, frame_id_rectified_aux_);

        const auto aux_cal = create_camera_info(calibration.aux.value(), aux_header, config.width, config.height);
        const auto aux_rect_cal = create_camera_info(calibration.aux.value(), aux_rect_header, config.width, config.height);

        aux_mono_cam_pub_ = std::make_shared<ImagePublisher>(aux_node_,
                                                             MONO_TOPIC,
                                                             aux_cal,
                                                             qos,
                                                             create_publisher_options({ds::AUX_LUMA_RAW},
                                                                                      get_full_topic_name(aux_node_, MONO_TOPIC)),
                                                             use_image_transport);

        aux_rgb_cam_pub_ = std::make_shared<ImagePublisher>(aux_node_,
                                                            COLOR_TOPIC,
                                                            aux_cal,
                                                            qos,
                                                            create_publisher_options({ds::AUX_RAW},
                                                                                     get_full_topic_name(aux_node_, COLOR_TOPIC)),
                                                            use_image_transport);

        aux_rect_cam_pub_ = std::make_shared<ImagePublisher>(aux_node_,
                                                             RECT_TOPIC,
                                                             aux_rect_cal,
                                                             qos,
                                                             create_publisher_options({ds::AUX_LUMA_RECTIFIED_RAW},
                                                                                      get_full_topic_name(aux_node_, RECT_TOPIC)),
                                                             use_image_transport);

        aux_rgb_rect_cam_pub_ = std::make_shared<ImagePublisher>(aux_node_,
                                                                 RECT_COLOR_TOPIC,
                                                                 aux_rect_cal,
                                                                 qos,
                                                                 create_publisher_options({ds::AUX_RECTIFIED_RAW},
                                                                                          get_full_topic_name(aux_node_, RECT_COLOR_TOPIC)),
                                                                 use_image_transport);

        color_point_cloud_pub_= create_publisher<sensor_msgs::msg::PointCloud2>(COLOR_POINTCLOUD_TOPIC,
                                                                                qos,
                                                                                create_publisher_options({ds::LEFT_DISPARITY_RAW,
                                                                                                          ds::AUX_RECTIFIED_RAW},
                                                                                          get_full_topic_name(aux_node_, COLOR_POINTCLOUD_TOPIC)));
    }

    point_cloud_pub_= create_publisher<sensor_msgs::msg::PointCloud2>(POINTCLOUD_TOPIC,
                                                                      qos,
                                                                      create_publisher_options({ds::LEFT_DISPARITY_RAW},
                                                                                     get_full_topic_name(left_node_, POINTCLOUD_TOPIC)));

    luma_point_cloud_pub_= create_publisher<sensor_msgs::msg::PointCloud2>(LUMA_POINTCLOUD_TOPIC,
                                                                           qos,
                                                                           create_publisher_options({ds::LEFT_DISPARITY_RAW,
                                                                                                     ds::LEFT_RECTIFIED_RAW},
                                                                                          get_full_topic_name(left_node_, LUMA_POINTCLOUD_TOPIC)));

    left_stereo_disparity_pub_ =
        left_node_->create_publisher<stereo_msgs::msg::DisparityImage>(DISPARITY_IMAGE_TOPIC,
                                                                       qos,
                                                                       create_publisher_options({ds::LEFT_DISPARITY_RAW},
                                                                                                get_full_topic_name(left_node_, DISPARITY_IMAGE_TOPIC)));

    if (info_.imu)
    {
        imu_pub_ = imu_node_->create_publisher<sensor_msgs::msg::Imu>(IMU_TOPIC,
                                                                      qos,
                                                                      create_publisher_options({ds::IMU},
                                                                                               get_full_topic_name(left_node_, IMU_TOPIC)));
    }


    //
    // Publish device info

    publish_info(info_);

    //
    // Publish the static transforms for our camera extrinsics for the left/right/aux frames. We will
    // use the left camera frame as the reference coordinate frame

    publish_static_tf(channel_->get_calibration());

    procesing_threads_.emplace_back(std::thread(&MultiSense::image_publisher, this));
    procesing_threads_.emplace_back(std::thread(&MultiSense::depth_publisher, this));
    procesing_threads_.emplace_back(std::thread(&MultiSense::point_cloud_publisher, this));
    procesing_threads_.emplace_back(std::thread(&MultiSense::color_publisher, this));
    procesing_threads_.emplace_back(std::thread(&MultiSense::imu_publisher, this));

    //
    // Add our single image frame callback which will notify all the process threads which may be listening
    // to image frames

    channel_->add_image_frame_callback([this](const auto &frame){image_frame_notifier_.set_and_notify(frame);});

    //
    // Add our single imu frame callback which will notify all the process threads which may be listening
    // to imu frames

    channel_->add_imu_frame_callback([this](const auto &frame){imu_frame_notifier_.set_and_notify(frame);});


    //
    // Setup parameters
    param_listener_ = std::make_shared<multisense::ParamListener>(get_node_parameters_interface());
    initialize_parameters(config, info_);

    paramter_handle_ = add_on_set_parameters_callback(std::bind(&MultiSense::parameter_callback, this, std::placeholders::_1));
    auto params = param_listener_->get_params();

    pointcloud_max_range_ = params.pointcloud_max_range;

    if (const auto status = channel_->set_config(update_config(config)); status != multisense::Status::OK)
    {
        RCLCPP_WARN(get_logger(), "Invalid config initialization %s", multisense::to_string(status).c_str());
    }

    current_config_ = channel_->get_config();

    status_timer_ = create_wall_timer(1s,
            [this]()
            {
                if (!shutdown_)
                {
                    if (const auto status = channel_->get_system_status(); status)
                    {
                        publish_status(status.value());
                    }
                }
            });
}

MultiSense::~MultiSense()
{
    stop();

    //
    // Shutdown all our publishing threads

    shutdown_ = true;
    image_frame_notifier_.notify_all();

    for (auto &thread : procesing_threads_)
    {
        thread.join();
    }
}

void MultiSense::image_publisher()
{
    const auto timeout = std::make_optional(std::chrono::milliseconds{500});
    while (!shutdown_)
    {
        if (const auto image_frame = image_frame_notifier_.wait(timeout) ; image_frame)
        {
            if (!is_time_valid(image_frame.value(), timestamp_source_))
            {
                RCLCPP_WARN(get_logger(), "FrameId %ld has a negative or zero time. Skipping image publish", image_frame->frame_id);
                continue;
            }

            const auto ros_time = create_time(image_frame.value(), timestamp_source_, camera_host_time_offset_);

            if (image_frame->stereo_histogram)
            {
                multisense_msgs::msg::Histogram histogram;

                histogram.frame_count = image_frame->frame_id;
                histogram.time_stamp = ros_time;

                histogram.width = current_config_.width;
                histogram.height = current_config_.height;
                histogram.fps = current_config_.frames_per_second;
                histogram.exposure_time = image_frame->capture_exposure_time.count();
                histogram.gain = image_frame->capture_gain;

                histogram.channels = image_frame->stereo_histogram->channels;
                histogram.bins = image_frame->stereo_histogram->bins;
                histogram.data = image_frame->stereo_histogram->data;

                histogram_pub_->publish(std::make_unique<multisense_msgs::msg::Histogram>(std::move(histogram)));
            }

            for (const auto &[source, image] : image_frame->images)
            {
                switch (source)
                {
                    case lms::DataSource::LEFT_MONO_RAW:
                    {
                        if (num_subscribers(left_node_, MONO_TOPIC) == 0) continue;
                        publish_image(image, left_mono_cam_pub_, left_mono_image_, frame_id_left_, ros_time);
                        break;
                    }
                    case lms::DataSource::RIGHT_MONO_RAW:
                    {
                        if (num_subscribers(right_node_, MONO_TOPIC) == 0) continue;
                        publish_image(image, right_mono_cam_pub_, right_mono_image_, frame_id_right_, ros_time);
                        break;
                    }
                    case lms::DataSource::LEFT_RECTIFIED_RAW:
                    {
                        if (num_subscribers(left_node_, RECT_TOPIC) == 0) continue;
                        publish_image(image, left_rect_cam_pub_, left_rect_image_, frame_id_rectified_left_, ros_time);
                        break;
                    }
                    case lms::DataSource::RIGHT_RECTIFIED_RAW:
                    {
                        if (num_subscribers(right_node_, RECT_TOPIC) == 0) continue;
                        publish_image(image, right_rect_cam_pub_, right_rect_image_, frame_id_rectified_right_, ros_time);
                        break;
                    }
                    case lms::DataSource::LEFT_DISPARITY_RAW:
                    {
                        if (num_subscribers(left_node_, DISPARITY_TOPIC) > 0)
                        {
                            publish_image(image, left_disparity_pub_, left_disparity_image_, frame_id_rectified_left_, ros_time);
                        }

                        if (num_subscribers(left_node_, DISPARITY_IMAGE_TOPIC) > 0)
                        {
                            populate_image(image, left_stereo_disparity_.image, frame_id_rectified_left_, ros_time);
                            left_stereo_disparity_.f = image.calibration.P[0][0];
                            left_stereo_disparity_.t = -image_frame->calibration.right.rectified_translation()[0];
                            left_stereo_disparity_.min_disparity = 0.0f;
                            left_stereo_disparity_.max_disparity = 256.0f;
                            left_stereo_disparity_.delta_d = 1.0f/16.0f;

                            left_stereo_disparity_pub_->publish(left_stereo_disparity_);
                        }
                        break;
                    }
                    case lms::DataSource::AUX_LUMA_RAW:
                    {
                        if (num_subscribers(aux_node_, MONO_TOPIC) == 0) continue;
                        publish_image(image, aux_mono_cam_pub_, aux_mono_image_, frame_id_aux_, ros_time);
                        break;
                    }
                    case lms::DataSource::AUX_LUMA_RECTIFIED_RAW:
                    {
                        if (num_subscribers(aux_node_, RECT_TOPIC) == 0) continue;
                        publish_image(image, aux_rect_cam_pub_, aux_rect_image_, frame_id_rectified_aux_, ros_time);
                        break;
                    }
                    case lms::DataSource::AUX_CHROMA_RAW:
                    case lms::DataSource::AUX_CHROMA_RECTIFIED_RAW:
                    {
                        //
                        // These are technically images, but we don't want to publish it raw just continue
                        continue;
                    }
                    case lms::DataSource::AUX_RAW:
                    {
                        if (num_subscribers(aux_node_, COLOR_TOPIC) == 0) continue;
                        publish_image(image, aux_rgb_cam_pub_, aux_rgb_image_, frame_id_aux_, ros_time);
                        break;
                    }
                    case lms::DataSource::AUX_RECTIFIED_RAW:
                    {
                        if (num_subscribers(aux_node_, RECT_COLOR_TOPIC) == 0) continue;
                        publish_image(image, aux_rgb_rect_cam_pub_, aux_rect_rgb_image_, frame_id_rectified_aux_, ros_time);
                        break;
                    }
                    case lms::DataSource::COST_RAW:
                    {
                        if (num_subscribers(left_node_, COST_TOPIC) == 0) continue;
                        publish_image(image, left_disparity_cost_pub_, left_disparity_cost_image_, frame_id_rectified_left_, ros_time);
                        break;
                    }
                    default: { RCLCPP_ERROR(get_logger(), "MultiSense: unknown image source"); continue;}
                }
            }
        }
    }
}

void MultiSense::depth_publisher()
{
    constexpr auto disparity_source = lms::DataSource::LEFT_DISPARITY_RAW;

    const auto timeout = std::make_optional(std::chrono::milliseconds{500});
    while (!shutdown_)
    {
        if (const auto image_frame = image_frame_notifier_.wait(timeout) ; image_frame)
        {
            if (!is_time_valid(image_frame.value(), timestamp_source_))
            {
                RCLCPP_WARN(get_logger(), "FrameId %ld has a negative or zero time. Skipping image publish", image_frame->frame_id);
                continue;
            }

            const auto ros_time = create_time(image_frame.value(), timestamp_source_, camera_host_time_offset_);

            if (image_frame->has_image(disparity_source))
            {
                if (num_subscribers(left_node_, DEPTH_TOPIC) > 0)
                {
                    if (const auto depth_image = lms::create_depth_image(image_frame.value(),
                                                                         lms::Image::PixelFormat::FLOAT32,
                                                                         disparity_source,
                                                                         std::numeric_limits<float>::quiet_NaN());
                                                                         depth_image)
                    {
                        publish_image(depth_image.value(), depth_cam_pub_, depth_image_, frame_id_rectified_left_, ros_time);
                    }
                }

                if (num_subscribers(left_node_, OPENNI_DEPTH_TOPIC) > 0)
                {
                    if (const auto depth_image = lms::create_depth_image(image_frame.value(),
                                                                         lms::Image::PixelFormat::MONO16,
                                                                         disparity_source,
                                                                         0);
                                                                         depth_image)
                    {
                        publish_image(depth_image.value(), ni_depth_cam_pub_, ni_depth_image_, frame_id_rectified_left_, ros_time);
                    }
                }
            }
        }
    }
}

void MultiSense::point_cloud_publisher()
{
    constexpr auto disparity_source = lms::DataSource::LEFT_DISPARITY_RAW;

    const auto timeout = std::make_optional(std::chrono::milliseconds{500});
    while (!shutdown_)
    {
        if (const auto image_frame = image_frame_notifier_.wait(timeout) ; image_frame)
        {
            if (!is_time_valid(image_frame.value(), timestamp_source_))
            {
                RCLCPP_WARN(get_logger(), "FrameId %ld has a negative or zero time. Skipping image publish", image_frame->frame_id);
                continue;
            }

            const auto ros_time = create_time(image_frame.value(), timestamp_source_, camera_host_time_offset_);

            if (image_frame->has_image(disparity_source))
            {
                if (num_subscribers(this, POINTCLOUD_TOPIC) > 0)
                {
                    if (const auto point_cloud = lms::create_pointcloud(image_frame.value(),
                                                                        pointcloud_max_range_,
                                                                        disparity_source);
                                                                        point_cloud)
                    {
                        publish_point_cloud(point_cloud.value(),
                                            ros_time,
                                            point_cloud_pub_,
                                            point_cloud_,
                                            frame_id_rectified_left_);
                    }
                }

                if (num_subscribers(this, LUMA_POINTCLOUD_TOPIC) > 0 && image_frame->has_image(lms::DataSource::LEFT_RECTIFIED_RAW))
                {
                    const auto color_format = image_frame->get_image(lms::DataSource::LEFT_RECTIFIED_RAW).format;

                    if (color_format == lms::Image::PixelFormat::MONO8)
                    {
                        if (const auto point_cloud = lms::create_color_pointcloud<uint8_t>(image_frame.value(),
                                                                                           pointcloud_max_range_,
                                                                                           lms::DataSource::LEFT_RECTIFIED_RAW,
                                                                                           disparity_source);
                                                                                           point_cloud)
                        {
                            publish_point_cloud(point_cloud.value(),
                                                ros_time,
                                                luma_point_cloud_pub_,
                                                luma_point_cloud_,
                                                frame_id_rectified_left_);
                        }
                    }
                    else if (color_format == lms::Image::PixelFormat::MONO16)
                    {
                        if (const auto point_cloud = lms::create_color_pointcloud<uint16_t>(image_frame.value(),
                                                                                            pointcloud_max_range_,
                                                                                            lms::DataSource::LEFT_RECTIFIED_RAW,
                                                                                            disparity_source);
                                                                                            point_cloud)
                        {
                            publish_point_cloud(point_cloud.value(),
                                                ros_time,
                                                luma_point_cloud_pub_,
                                                luma_point_cloud_,
                                                frame_id_rectified_left_);
                        }
                    }
                }

                if (num_subscribers(this, COLOR_POINTCLOUD_TOPIC) > 0 &&
                    image_frame->has_image(lms::DataSource::AUX_LUMA_RECTIFIED_RAW) &&
                    image_frame->has_image(lms::DataSource::AUX_CHROMA_RECTIFIED_RAW))
                {
                    const auto bgr_image = lms::create_bgr_image(image_frame.value(), lms::DataSource::AUX_RECTIFIED_RAW);
                    if (bgr_image)
                    {
                        if (const auto point_cloud = lms::create_color_pointcloud<std::array<uint8_t, 3>>(image_frame->get_image(disparity_source),
                                                                                                          bgr_image,
                                                                                                          pointcloud_max_range_,
                                                                                                          image_frame->calibration);
                                                                                                          point_cloud)
                        {
                            publish_point_cloud(point_cloud.value(),
                                                ros_time,
                                                color_point_cloud_pub_,
                                                color_point_cloud_,
                                                frame_id_rectified_left_);
                        }
                    }
                }
            }
        }
    }
}

void MultiSense::color_publisher()
{
    const auto timeout = std::make_optional(std::chrono::milliseconds{500});
    while (!shutdown_)
    {
        if (const auto image_frame = image_frame_notifier_.wait(timeout) ; image_frame)
        {
            if (!is_time_valid(image_frame.value(), timestamp_source_))
            {
                RCLCPP_WARN(get_logger(), "FrameId %ld has a negative or zero time. Skipping image publish", image_frame->frame_id);
                continue;
            }

            const auto ros_time = create_time(image_frame.value(), timestamp_source_, camera_host_time_offset_);

            if (num_subscribers(aux_node_, COLOR_TOPIC) > 0)
            {
                if (auto bgr_image = lms::create_bgr_image(image_frame.value(), lms::DataSource::AUX_RAW); bgr_image)
                {
                    publish_image(bgr_image.value(), aux_rgb_cam_pub_, aux_rgb_image_, frame_id_aux_, ros_time);
                }
            }

            if (num_subscribers(aux_node_, RECT_COLOR_TOPIC) > 0)
            {
                if (auto bgr_image = lms::create_bgr_image(image_frame.value(), lms::DataSource::AUX_RECTIFIED_RAW); bgr_image)
                {
                    publish_image(bgr_image.value(), aux_rgb_rect_cam_pub_, aux_rect_rgb_image_, frame_id_rectified_aux_, ros_time);
                }
            }
        }
    }
}

void MultiSense::imu_publisher()
{
    const auto timeout = std::make_optional(std::chrono::milliseconds{500});
    while (!shutdown_)
    {
        if (const auto imu_frame = imu_frame_notifier_.wait(timeout) ; imu_frame)
        {
            for (const auto &sample : imu_frame->samples)
            {
                if (!is_time_valid(sample, timestamp_source_))
                {
                    RCLCPP_WARN(get_logger(), "IMU smaple has a negative or zero time. Skipping sample publish");
                    continue;
                }

                const auto ros_time = create_time(sample, timestamp_source_, camera_host_time_offset_);

                publish_imu_sample(sample, imu_pub_, frame_id_imu_, ros_time);
            }
        }
    }
}

void MultiSense::stop()
{
    if (const auto status = channel_->stop_streams({lms::DataSource::ALL}); status != lms::Status::OK)
    {
        RCLCPP_ERROR(get_logger(), "MultiSense: failed to stop all streams: %s", lms::to_string(status).c_str());
    }

    std::lock_guard<std::mutex> lock{stream_mutex_};
    active_streams_.clear();
    active_topics_.clear();
}

rclcpp::PublisherOptions MultiSense::create_publisher_options(const std::vector<multisense::DataSource> &sources,
                                                              const std::string &topic)
{
    rclcpp::PublisherOptions options;
    options.event_callbacks.matched_callback =
        [this, sources, topic](const auto &info)
        {
            std::lock_guard<std::mutex> lock{this->stream_mutex_};

            if (info.current_count >= 1 && (active_topics_.count(topic) == 0 || active_topics_.at(topic) == 0))
            {
                for (const auto &source : sources)
                {
                    this->active_streams_[source]++;
                }
            }
            else if (info.current_count <= 0)
            {
                for (const auto &source : sources)
                {
                    this->active_streams_[source]--;
                }
            }

            active_topics_[topic] = info.current_count;

            std::vector<multisense::DataSource> streams_to_stop{};
            std::vector<multisense::DataSource> start_streams{};
            for (const auto &[stream, count] : this->active_streams_)
            {
                if (count > 0)
                {
                    start_streams.push_back(stream);
                }
                else
                {
                    streams_to_stop.push_back(stream);
                }
            }

            if (const auto status = this->channel_->stop_streams(streams_to_stop) ; status != lms::Status::OK)
            {
                RCLCPP_ERROR(get_logger(), "Unable to stop streams");
            }


            if (const auto status = this->channel_->start_streams(start_streams) ; status != lms::Status::OK)
            {
                RCLCPP_ERROR(get_logger(), "Unable to modify active streams");
            }
        };

    return options;
}

void MultiSense::publish_static_tf(const multisense::StereoCalibration &stereo_calibration)
{
    std::vector<geometry_msgs::msg::TransformStamped> stamped_transforms(3 + (stereo_calibration.aux ? 2 : 0));

    tf2::Transform rectified_left_T_left{to_rotation(stereo_calibration.left.R), tf2::Vector3{0., 0., 0.}};
    stamped_transforms[0].header.stamp = rclcpp::Clock().now();
    stamped_transforms[0].header.frame_id = frame_id_rectified_left_;
    stamped_transforms[0].child_frame_id = frame_id_left_;
    stamped_transforms[0].transform = tf2::toMsg(rectified_left_T_left);

    tf2::Transform rectified_right_T_rectified_left{tf2::Matrix3x3::getIdentity(),
                                                    tf2::Vector3{stereo_calibration.right.rectified_translation()[0],
                                                                 stereo_calibration.right.rectified_translation()[1],
                                                                 stereo_calibration.right.rectified_translation()[2]}};
    stamped_transforms[1].header.stamp = rclcpp::Clock().now();
    stamped_transforms[1].header.frame_id = frame_id_rectified_left_;
    stamped_transforms[1].child_frame_id = frame_id_rectified_right_;
    stamped_transforms[1].transform = tf2::toMsg(rectified_right_T_rectified_left.inverse());

    tf2::Transform rectified_right_T_right{to_rotation(stereo_calibration.right.R), tf2::Vector3{0., 0., 0.}};
    stamped_transforms[2].header.stamp = rclcpp::Clock().now();
    stamped_transforms[2].header.frame_id = frame_id_rectified_right_;
    stamped_transforms[2].child_frame_id = frame_id_right_;
    stamped_transforms[2].transform = tf2::toMsg(rectified_right_T_right);

    if (stereo_calibration.aux)
    {
        tf2::Transform rectified_aux_T_rectified_left{tf2::Matrix3x3::getIdentity(),
                                                      tf2::Vector3{stereo_calibration.aux->rectified_translation()[0],
                                                                   stereo_calibration.aux->rectified_translation()[1],
                                                                   stereo_calibration.aux->rectified_translation()[2]}};
        stamped_transforms[3].header.stamp = rclcpp::Clock().now();
        stamped_transforms[3].header.frame_id = frame_id_rectified_left_;
        stamped_transforms[3].child_frame_id = frame_id_rectified_aux_;
        stamped_transforms[3].transform = tf2::toMsg(rectified_aux_T_rectified_left.inverse());

        tf2::Transform rectified_aux_T_aux{to_rotation(stereo_calibration.aux->R), tf2::Vector3{0., 0., 0.}};
        stamped_transforms[4].header.stamp = rclcpp::Clock().now();
        stamped_transforms[4].header.frame_id = frame_id_rectified_aux_;
        stamped_transforms[4].child_frame_id = frame_id_aux_;
        stamped_transforms[4].transform = tf2::toMsg(rectified_aux_T_aux);
    }

    static_tf_broadcaster_->sendTransform(stamped_transforms);
}

void MultiSense::publish_info(const lms::MultiSenseInfo &info)
{
    multisense_msgs::msg::Info info_msg;

    info_msg.device_name     = info.device.camera_name;
    info_msg.build_date      = info.device.build_date;
    info_msg.serial_number   = info.device.serial_number;
    info_msg.device_revision = static_cast<int>(info.device.hardware_revision);

    for (const auto &pcb: info.device.pcb_info)
    {
        info_msg.pcb_serial_numbers.push_back(pcb.revision);
        info_msg.pcb_names.push_back(pcb.name);
    }

    info_msg.imager_name = info.device.imager_name;
    info_msg.imager_type = static_cast<int>(info.device.imager_type);
    info_msg.imager_width = info.device.imager_width;
    info_msg.imager_height = info.device.imager_height;

    info_msg.lens_name = info.device.lens_name;
    info_msg.lens_type = static_cast<int>(info.device.lens_type);
    info_msg.nominal_baseline = info.device.nominal_stereo_baseline;
    info_msg.nominal_focal_length = info.device.nominal_focal_length;
    info_msg.nominal_relative_aperture = info.device.nominal_relative_aperture;

    info_msg.lighting_type = static_cast<int>(info.device.lighting_type);

    info_msg.firmware_build_date = info.version.firmware_build_date;
    info_msg.firmware_version = static_cast<uint16_t>(info.version.firmware_version.major << 8) |
                                       static_cast<uint16_t>(info.version.firmware_version.minor);
    info_msg.bitstream_version = info.version.hardware_version;

    info_msg.ip_address = info.network.ip_address;
    info_msg.gateway = info.network.gateway;
    info_msg.netmask = info.network.netmask;

    info_pub_->publish(info_msg);
}

void MultiSense::publish_config(const multisense::MultiSenseConfig &config)
{
    std_msgs::msg::String output;

    output.data = nlohmann::to_string(nlohmann::json{config});

    config_pub_->publish(std::make_unique<std_msgs::msg::String>(std::move(output)));
}

void MultiSense::publish_status(const multisense::MultiSenseStatus &status)
{
    multisense_msgs::msg::Status output;

    if (status.time)
    {
        output.host_time = rclcpp::Time(status.time->client_host_time.count());
        output.camera_time = rclcpp::Time(status.time->camera_time.count());
        output.network_delay = rclcpp::Time(status.time->network_delay.count());

        //
        // Ignore outliers for delays in status response

        if (status.time->network_delay < 5ms)
        {
            if (!camera_host_time_offset_)
            {
                camera_host_time_offset_ = status.time->offset_to_host();
            }
            else
            {
                //
                // Run a decayed average on our offset. Use doubles to handle interger overflow

                const auto previous_camera_host_time_offset = std::chrono::duration<double>(camera_host_time_offset_.value());
                const auto current_camera_host_time_offset = std::chrono::duration<double>(status.time->offset_to_host());

                const auto offset = std::chrono::duration<double>(((time_offset_buffer_size_ - 1) * previous_camera_host_time_offset.count() +
                                     current_camera_host_time_offset.count()) / time_offset_buffer_size_);

                camera_host_time_offset_ = std::chrono::duration_cast<std::chrono::nanoseconds>(offset);
            }
        }
    }

    output.system_ok = status.system_ok;
    output.cameras_ok = status.camera.cameras_ok;
    output.processing_pipeline_ok = status.camera.processing_pipeline_ok;

    if (status.temperature)
    {
        output.cpu_temp = status.temperature->cpu_temperature;
        output.fpga_temp = status.temperature->fpga_temperature;
        output.left_imager_temp = status.temperature->left_imager_temperature;
        output.right_imager_temp = status.temperature->right_imager_temperature;
    }

    if (status.power)
    {
        output.input_voltage = status.power->input_voltage;
        output.input_current = status.power->input_current;
        output.fpga_power = status.power->fpga_power;
    }

    output.ptp_valid = static_cast<bool>(status.ptp);
    if (status.ptp)
    {
        output.ptp_grandmaster_present = status.ptp->grandmaster_present;
        output.ptp_grandmaster_id = status.ptp->grandmaster_id;
        output.ptp_grandmaster_offset = status.ptp->grandmaster_offset.count();
        output.ptp_path_delay = status.ptp->path_delay.count();
        output.ptp_steps_removed = status.ptp->steps_from_local_to_grandmaster;
    }

    output.received_messages = status.client_network.received_messages;
    output.dropped_messages = status.client_network.dropped_messages;
    output.invalid_packets = status.client_network.invalid_packets;

    status_pub_->publish(std::make_unique<multisense_msgs::msg::Status>(std::move(output)));
}

size_t MultiSense::num_subscribers(const rclcpp::Node::SharedPtr &node, const std::string &topic) const
{
    return num_subscribers(node.get(), topic);
}

size_t MultiSense::num_subscribers(const rclcpp::Node* node, const std::string &topic) const
{
    const std::string full_topic = node->get_sub_namespace().empty() ? topic : node->get_sub_namespace()  + "/" + topic;

    if (!rclcpp::ok())
    {
        return 0;
    }

    return node->count_subscribers(full_topic) ;
}

void MultiSense::initialize_parameters(const multisense::MultiSenseConfig &config, const multisense::MultiSenseInfo& info)
{
    //
    // Sensor resolution

    std::string valid_resolutions = "valid_resolutions [width, height, max disparity value]\n";
    for (const auto& device_mode : info.operating_modes)
    {
        valid_resolutions += "[";
        valid_resolutions += std::to_string(device_mode.width) + ", ";
        valid_resolutions += std::to_string(device_mode.height) + ", ";
        valid_resolutions += std::to_string(disparity_pixels(device_mode.disparities)) + "]\n";
    }

    rcl_interfaces::msg::ParameterDescriptor sensor_resolution_desc;
    sensor_resolution_desc.set__name("sensor_resolution")
                          .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY)
                          .set__description(valid_resolutions);

    declare_parameter("sensor_resolution",
                      std::vector<int64_t>{config.width, config.height, disparity_pixels(config.disparities)},
                      sensor_resolution_desc);

    //
    // Imu rates and ranges

    if (info.imu)
    {
        if (info.imu->accelerometer && config.imu_config && config.imu_config->accelerometer)
        {
            create_imu_parameters(info.imu->accelerometer.value(),
                                  config.imu_config->accelerometer.value(),
                                  "imu.accelerometer");
        }
        if (info.imu->gyroscope && config.imu_config && config.imu_config->gyroscope)
        {
            create_imu_parameters(info.imu->gyroscope.value(),
                                  config.imu_config->gyroscope.value(),
                                  "imu.gyroscope");
        }
        if (info.imu->magnetometer && config.imu_config && config.imu_config->magnetometer)
        {
            create_imu_parameters(info.imu->magnetometer.value(),
                                  config.imu_config->magnetometer.value(),
                                  "imu.magnetometer");
        }
    }

    //
    // If our main image pair does not support color, make sure we explicitly disable white balance. If
    // we could undeclare the parameter, we would, but that interface does not seem to work

    if (!info.device.has_main_stereo_color())
    {
        set_parameter(rclcpp::Parameter{"image.auto_white_balance_enabled", false});
    }

    //
    // If we don't have a aux camera disable controls

    if (!info.device.has_aux_camera())
    {
        set_parameters(std::vector<rclcpp::Parameter>{rclcpp::Parameter{"aux.image.auto_exposure_enabled", false},
                                                      rclcpp::Parameter{"aux.image.auto_white_balance_enabled", false}});
    }
}

void MultiSense::create_imu_parameters(const multisense::MultiSenseInfo::ImuInfo::Source &imu_source,
                                       const multisense::MultiSenseConfig::ImuConfig::OperatingMode &operating_mode,
                                       const std::string &parameter_base)
{
        //
        // Rate

        std::stringstream rate_description;
        rate_description << std::fixed << std::setprecision(2);
        rate_description << "valid rates\n";
        int current_rate = 0;
        size_t rate_i = 0;
        for (const auto &rate : imu_source.rates)
        {
            rate_description << rate_i << ": " <<
                                 rate.sample_rate << "Hz " <<
                                 rate.bandwith_cutoff << "Hz cuttoff\n";

            if (rate == operating_mode.rate)
            {
                current_rate = rate_i;
            }

            ++rate_i;
        }

        rcl_interfaces::msg::IntegerRange rate_range;
        rate_range.set__from_value(0)
                        .set__to_value(rate_i-1)
                        .set__step(1);

        rcl_interfaces::msg::ParameterDescriptor rate_desc;
        rate_desc.set__name(parameter_base + ".rate")
                          .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                          .set__description(rate_description.str())
                          .set__integer_range({rate_range});

        declare_parameter(parameter_base + ".rate", current_rate, rate_desc);

        //
        // Range

        std::stringstream range_description;
        range_description << std::fixed << std::setprecision(4);
        range_description << "valid ranges\n";
        int current_range = 0;
        size_t range_i = 0;
        for (const auto &range : imu_source.ranges)
        {
            range_description << range_i << ": range " <<
                                 range.range << " resolution " <<
                                 range.resolution << "\n";

            if (range == operating_mode.range)
            {
                current_range = range_i;
            }

            ++range_i;
        }

        rcl_interfaces::msg::IntegerRange range_range;
        range_range.set__from_value(0)
                        .set__to_value(range_i-1)
                        .set__step(1);

        rcl_interfaces::msg::ParameterDescriptor range_desc;
        range_desc.set__name(parameter_base + "range")
                          .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                          .set__description(range_description.str())
                          .set__integer_range({range_range});

        declare_parameter(parameter_base + ".range", current_range, range_desc);
}

multisense::MultiSenseConfig MultiSense::update_config(multisense::MultiSenseConfig config)
{
    config = update_param_config(std::move(config), param_listener_->get_params(), info_.device.hardware_revision);

    if (config.time_config)
    {
        //
        // Force a PTP timesource if PTP is enabled

        if (config.time_config->ptp_enabled)
        {
            timestamp_source_ = TimestampSource::PTP;
        }
    }

    const auto res = get_parameter("sensor_resolution").as_integer_array();
    config.width = res[0];
    config.height = res[1];
    config.disparities = pixel_to_disparity(res[2]);

    if (config.imu_config)
    {
        if (config.imu_config->accelerometer)
        {
            config.imu_config->accelerometer->rate = info_.imu->accelerometer->rates[get_parameter("imu.accelerometer.rate").as_int()];
            config.imu_config->accelerometer->range = info_.imu->accelerometer->ranges[get_parameter("imu.accelerometer.range").as_int()];
        }

        if (config.imu_config->gyroscope)
        {
            config.imu_config->gyroscope->rate = info_.imu->gyroscope->rates[get_parameter("imu.gyroscope.rate").as_int()];
            config.imu_config->gyroscope->range = info_.imu->gyroscope->ranges[get_parameter("imu.gyroscope.range").as_int()];
        }

        if (config.imu_config->magnetometer)
        {
            config.imu_config->magnetometer->rate = info_.imu->magnetometer->rates[get_parameter("imu.magnetometer.rate").as_int()];
            config.imu_config->magnetometer->range = info_.imu->magnetometer->ranges[get_parameter("imu.magnetometer.range").as_int()];
        }
    }

    return config;
}

rcl_interfaces::msg::SetParametersResult MultiSense::parameter_callback(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.set__successful(true);

    param_listener_->update(parameters);

    //
    // PTP subsumes network time sync, which subsumes camera time

    if (param_listener_->get_params().time.ptp_enabled)
    {
        timestamp_source_ = TimestampSource::PTP;
    }
    else if (param_listener_->get_params().time.network_time_sync_enabled)
    {
        timestamp_source_ = TimestampSource::SYSTEM;
    }
    else
    {
        timestamp_source_ = TimestampSource::CAMERA;
    }

    pointcloud_max_range_ = param_listener_->get_params().pointcloud_max_range;

    for (const auto &parameter : parameters)
    {
        const auto type = parameter.get_type();
        if (type == rclcpp::ParameterType::PARAMETER_NOT_SET)
        {
            continue;
        }

        const auto name = parameter.get_name();

        //
        // Validate our resolution

        if (name == "sensor_resolution")
        {
            if (type != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
            {
                return result.set__successful(false).set__reason("invalid sensor resolution type");
            }

            const auto value = parameter.as_integer_array();

            if (value.size() != 3)
            {
                return result.set__successful(false)
                             .set__reason("MultiSense: Invalid sensor resolution. Must be [width, height, max disparity]");
            }
        }
    }

    const auto config = update_config(channel_->get_config());

    if (const auto status = channel_->set_config(config); status != multisense::Status::OK)
    {
        //
        // Use nlohmann json to figure out which configs did not apply
        RCLCPP_WARN(get_logger(), "set configuration not successful: %s", lms::to_string(status).c_str());

        if (status == multisense::Status::INCOMPLETE_APPLICATION)
        {
            const nlohmann::json current = channel_->get_config();

            std::stringstream ss;
            ss << "configured:\n";
            ss << std::setw(4) << nlohmann::json::diff(current, config);
            ss << "current:\n";
            ss << std::setw(4) << nlohmann::json::diff(config, current);

            RCLCPP_WARN(get_logger(), "configs which did not apply:\n %s", ss.str().c_str());
        }

        publish_config(config);
        current_config_ = config;

        return result.set__successful(false).set__reason(multisense::to_string(status));
    }

    const auto current_config = channel_->get_config();
    publish_config(current_config);
    current_config_ = current_config;

    return result;
}

} // namespace
