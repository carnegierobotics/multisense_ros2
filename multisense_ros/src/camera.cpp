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
#include <opencv2/opencv.hpp>

#include <Eigen/Geometry>
#include <sensor_msgs/image_encodings.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <multisense_msgs/msg/device_info.hpp>
#include <multisense_ros/camera.h>
#include <multisense_ros/parameter_utilities.h>

#include <MultiSense/MultiSenseUtilities.hh>

namespace lms = multisense;

using namespace std::chrono_literals;

namespace multisense_ros {

namespace { // anonymous

tf2::Matrix3x3 to_rotation(const std::array<std::array<float, 3>, 3> & R)
{
    return tf2::Matrix3x3{R[0][0],
                          R[0][1],
                          R[0][2],
                          R[1][0],
                          R[1][1],
                          R[1][2],
                          R[2][0],
                          R[2][1],
                          R[2][2]};
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
                    const std::string &frame_id)
{
    ros_image.data.resize(image.image_data_length);
    memcpy(&ros_image.data[0], image.raw_data->data() + image.image_data_offset, image.image_data_length);

    ros_image.header.frame_id = frame_id;
    ros_image.header.stamp = rclcpp::Time(image.camera_timestamp.time_since_epoch().count());
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
                   const std::string &frame_id)
{
    populate_image(image, ros_image, frame_id);

    auto camera_info = create_camera_info(image.calibration, ros_image.header, image.width, image.height);
    publisher->publish(std::make_unique<sensor_msgs::msg::Image>(ros_image),
                       std::make_unique<sensor_msgs::msg::CameraInfo>(std::move(camera_info)));
}

template <typename Color>
void publish_point_cloud(const lms::PointCloud<Color> &point_cloud,
                         const lms::TimeT &time,
                         rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
                         sensor_msgs::msg::PointCloud2 &ros_point_cloud,
                         const std::string &frame_id)
{
    ros_point_cloud.is_bigendian = (htonl(1) == 1);
    ros_point_cloud.is_dense = true;
    ros_point_cloud.point_step = sizeof(lms::Point<Color>);
    ros_point_cloud.header.frame_id = frame_id;
    ros_point_cloud.fields.reserve(4);
    ros_point_cloud.fields.resize(3);
    ros_point_cloud.fields[0].name     = "x";
    ros_point_cloud.fields[0].offset   = 0;
    ros_point_cloud.fields[0].count    = 1;
    ros_point_cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    ros_point_cloud.fields[1].name     = "y";
    ros_point_cloud.fields[1].offset   = sizeof(float);
    ros_point_cloud.fields[1].count    = 1;
    ros_point_cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    ros_point_cloud.fields[2].name     = "z";
    ros_point_cloud.fields[2].offset   = 2 * sizeof(float);
    ros_point_cloud.fields[2].count    = 1;
    ros_point_cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;

    if constexpr (std::is_same_v<Color, std::array<uint8_t, 3>>)
    {
        ros_point_cloud.fields.resize(4);
        ros_point_cloud.fields[3].name = "bgr";
        ros_point_cloud.fields[3].offset = 3 * sizeof(float);
        ros_point_cloud.fields[3].count = 1;
        ros_point_cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;

        //
        // Unfortunately we need to iterate and write our points to the output buffer since we need to convert
        // bgr pixels to rgb pixels, and pad them with an extra 8 bits to align with the FLOAT32 boundary

        ros_point_cloud.data.resize(point_cloud.cloud.size() * (sizeof(lms::Point<Color>) + 1));
        auto point_cloud_p = reinterpret_cast<float*>(ros_point_cloud.data.data());
        for (size_t i = 0; i < point_cloud.cloud.size(); ++i)
        {
            point_cloud_p[i + 0] = point_cloud.cloud[i].x;
            point_cloud_p[i + 1] = point_cloud.cloud[i].y;
            point_cloud_p[i + 2] = point_cloud.cloud[i].z;

            const uint32_t rgb = static_cast<uint32_t>(0) |
                                 (static_cast<uint32_t>(point_cloud.cloud[i].color[2]) << 16) |
                                 (static_cast<uint32_t>(point_cloud.cloud[i].color[1]) << 8) |
                                 (static_cast<uint32_t>(point_cloud.cloud[i].color[0]));

            auto color_p = reinterpret_cast<uint32_t*>(&point_cloud_p[i + 3]);
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

        ros_point_cloud.data.resize(point_cloud.cloud.size() * sizeof(lms::Point<Color>));
        memcpy(&ros_point_cloud.data[0], reinterpret_cast<const uint8_t*>(point_cloud.cloud.data()), ros_point_cloud.data.size());
    }

    ros_point_cloud.width = point_cloud.cloud.size();
    ros_point_cloud.height = 1;
    ros_point_cloud.row_step = point_cloud.cloud.size() * ros_point_cloud.point_step;

    ros_point_cloud.header.frame_id = frame_id;
    ros_point_cloud.header.stamp = rclcpp::Time(time.time_since_epoch().count());

    publisher->publish(std::make_unique<sensor_msgs::msg::PointCloud2>(ros_point_cloud));
}

} // anonymous

//
// Provide compiler with definition of the static members

constexpr char Camera::LEFT[];
constexpr char Camera::RIGHT[];
constexpr char Camera::AUX[];
constexpr char Camera::CALIBRATION[];

constexpr char Camera::LEFT_CAMERA_FRAME[];
constexpr char Camera::RIGHT_CAMERA_FRAME[];
constexpr char Camera::LEFT_RECTIFIED_FRAME[];
constexpr char Camera::RIGHT_RECTIFIED_FRAME[];
constexpr char Camera::AUX_CAMERA_FRAME[];
constexpr char Camera::AUX_RECTIFIED_FRAME[];

constexpr char Camera::DEVICE_INFO_TOPIC[];
constexpr char Camera::RAW_CAM_CAL_TOPIC[];
constexpr char Camera::RAW_CAM_CONFIG_TOPIC[];
constexpr char Camera::RAW_CAM_DATA_TOPIC[];
constexpr char Camera::HISTOGRAM_TOPIC[];
constexpr char Camera::MONO_TOPIC[];
constexpr char Camera::RECT_TOPIC[];
constexpr char Camera::DISPARITY_TOPIC[];
constexpr char Camera::DISPARITY_IMAGE_TOPIC[];
constexpr char Camera::DEPTH_TOPIC[];
constexpr char Camera::OPENNI_DEPTH_TOPIC[];
constexpr char Camera::COST_TOPIC[];
constexpr char Camera::COLOR_TOPIC[];
constexpr char Camera::RECT_COLOR_TOPIC[];
constexpr char Camera::POINTCLOUD_TOPIC[];
constexpr char Camera::COLOR_POINTCLOUD_TOPIC[];
constexpr char Camera::ORGANIZED_POINTCLOUD_TOPIC[];
constexpr char Camera::COLOR_ORGANIZED_POINTCLOUD_TOPIC[];
constexpr char Camera::MONO_CAMERA_INFO_TOPIC[];
constexpr char Camera::RECT_CAMERA_INFO_TOPIC[];
constexpr char Camera::COLOR_CAMERA_INFO_TOPIC[];
constexpr char Camera::RECT_COLOR_CAMERA_INFO_TOPIC[];
constexpr char Camera::DEPTH_CAMERA_INFO_TOPIC[];
constexpr char Camera::DISPARITY_CAMERA_INFO_TOPIC[];
constexpr char Camera::COST_CAMERA_INFO_TOPIC[];

Camera::Camera(const std::string& node_name,
               const rclcpp::NodeOptions& options,
               std::unique_ptr<multisense::Channel> channel,
               const std::string& tf_prefix):
    Node(node_name, options),
    channel_(std::move(channel)),
    left_node_(create_sub_node(LEFT)),
    right_node_(create_sub_node(RIGHT)),
    aux_node_(create_sub_node(AUX)),
    calibration_node_(create_sub_node(CALIBRATION)),
    frame_id_left_(tf_prefix + LEFT_CAMERA_FRAME),
    frame_id_right_(tf_prefix + RIGHT_CAMERA_FRAME),
    frame_id_aux_(tf_prefix + AUX_CAMERA_FRAME),
    frame_id_rectified_left_(tf_prefix + LEFT_RECTIFIED_FRAME),
    frame_id_rectified_right_(tf_prefix + RIGHT_RECTIFIED_FRAME),
    frame_id_rectified_aux_(tf_prefix + AUX_RECTIFIED_FRAME),
    static_tf_broadcaster_(std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this)),
    pointcloud_max_range_(15.0)
{
    info_ = channel_->get_info();

    //
    // S27/S30 cameras have a 3rd aux color camera and no left color camera

    has_aux_camera_ = info_.device.hardware_revision == lms::MultiSenseInfo::DeviceInfo::HardwareRevision::S27 ||
                      info_.device.hardware_revision == lms::MultiSenseInfo::DeviceInfo::HardwareRevision::S30 ||
                      info_.device.hardware_revision == lms::MultiSenseInfo::DeviceInfo::HardwareRevision::KS21i;

    //
    // Topics published for all device types

    const auto latching_qos = rclcpp::QoS(1).transient_local();
    device_info_pub_ = calibration_node_->create_publisher<multisense_msgs::msg::DeviceInfo>(DEVICE_INFO_TOPIC, latching_qos);

    const auto now = rclcpp::Clock().now();
    const auto left_header = create_header(now, frame_id_left_);
    const auto left_rect_header = create_header(now, frame_id_rectified_left_);
    const auto right_header = create_header(now, frame_id_right_);
    const auto right_rect_header = create_header( now, frame_id_rectified_right_);

    const auto configuration = channel_->get_configuration();
    const auto calibration = channel_->get_calibration();
    const auto left_cal = create_camera_info(calibration.left, left_header, configuration.width, configuration.height);
    const auto left_rect_cal = create_camera_info(calibration.left, left_rect_header, configuration.width, configuration.height);
    const auto right_cal = create_camera_info(calibration.left, right_header, configuration.width, configuration.height);
    const auto right_rect_cal = create_camera_info(calibration.left, right_rect_header, configuration.width, configuration.height);

    //
    // Image publishers

    bool use_image_transport = true;
    const auto qos = rclcpp::SensorDataQoS();

    using ds = lms::DataSource;

    left_mono_cam_pub_ = add_image_publisher(left_node_, MONO_TOPIC, left_cal, qos, ds::LEFT_MONO_RAW, use_image_transport);
    right_mono_cam_pub_ = add_image_publisher(right_node_, MONO_TOPIC, right_cal, qos, ds::RIGHT_MONO_RAW, use_image_transport);
    left_rect_cam_pub_ = add_image_publisher(left_node_, RECT_TOPIC, left_rect_cal, qos, ds::LEFT_RECTIFIED_RAW, use_image_transport);
    right_rect_cam_pub_ = add_image_publisher(right_node_, RECT_TOPIC, right_rect_cal, qos, ds::RIGHT_RECTIFIED_RAW, use_image_transport);
    depth_cam_pub_ = add_image_publisher(left_node_, DEPTH_TOPIC, left_rect_cal, qos, ds::LEFT_DISPARITY_RAW, use_image_transport);
    ni_depth_cam_pub_ = add_image_publisher(left_node_, OPENNI_DEPTH_TOPIC, left_rect_cal, qos, ds::LEFT_DISPARITY_RAW, use_image_transport);
    left_disparity_cost_pub_ = add_image_publisher(left_node_, COST_TOPIC, left_rect_cal, qos, ds::COST_RAW, use_image_transport);

    if (has_aux_camera_)
    {
        if (!calibration.aux)
        {
            throw std::runtime_error("Invalid aux calibration");
        }

        const auto aux_header = create_header(now, frame_id_aux_);
        const auto aux_rect_header = create_header(now, frame_id_rectified_aux_);

        const auto aux_cal = create_camera_info(calibration.aux.value(), aux_header, configuration.width, configuration.height);
        const auto aux_rect_cal = create_camera_info(calibration.aux.value(), aux_rect_header, configuration.width, configuration.height);

        aux_mono_cam_pub_ = add_image_publisher(aux_node_, MONO_TOPIC, aux_cal, qos, ds::AUX_LUMA_RAW, use_image_transport);
        aux_rgb_cam_pub_ = add_image_publisher(aux_node_, COLOR_TOPIC, aux_cal, qos, ds::AUX_RAW, use_image_transport);
        aux_rect_cam_pub_ = add_image_publisher(aux_node_, RECT_TOPIC, aux_rect_cal, qos, ds::AUX_LUMA_RECTIFIED_RAW, use_image_transport);
        aux_rgb_rect_cam_pub_ = add_image_publisher(aux_node_, RECT_TOPIC, aux_rect_cal, qos, ds::AUX_RECTIFIED_RAW, use_image_transport);

        color_point_cloud_pub_= add_pointcloud_publisher(COLOR_POINTCLOUD_TOPIC, qos,
                                                        {ds::LEFT_DISPARITY_RAW, ds::AUX_RECTIFIED_RAW});
    }

    luma_point_cloud_pub_= add_pointcloud_publisher(POINTCLOUD_TOPIC, qos,
                                                    {ds::LEFT_DISPARITY_RAW, ds::LEFT_RECTIFIED_RAW});

    left_stereo_disparity_pub_ = left_node_->create_publisher<stereo_msgs::msg::DisparityImage>(DISPARITY_IMAGE_TOPIC, qos);

    //
    // All image streams off

    stop();

    //
    // Publish device info

    publish_device_info(info_.device, info_.version);

    //
    // Publish the static transforms for our camera extrinsics for the left/right/aux frames. We will
    // use the left camera frame as the reference coordinate frame

    publish_static_tf(channel_->get_calibration());

    procesing_threads_.emplace_back(std::thread(&Camera::image_publisher, this));
    procesing_threads_.emplace_back(std::thread(&Camera::depth_publisher, this));
    procesing_threads_.emplace_back(std::thread(&Camera::point_cloud_publisher, this));
    procesing_threads_.emplace_back(std::thread(&Camera::color_publisher, this));

    //
    // Add our single image frame callback which will notify all the process threads which may be listening
    // to image frames

    channel_->add_image_frame_callback([this](const auto &frame){image_frame_notifier_.set_and_notify(frame);});

    //
    // Setup parameters
    //paramter_handle_ = add_on_set_parameters_callback(std::bind(&Camera::parameterCallback, this, std::placeholders::_1));
    //initalizeParameters(image_config);
}

Camera::~Camera()
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

void Camera::image_publisher()
{
    const auto timeout = std::make_optional(std::chrono::milliseconds{500});
    while (!shutdown_)
    {
        if (const auto image_frame = image_frame_notifier_.wait(timeout) ; image_frame)
        {
            for (const auto &[source, image] : image_frame->images)
            {
                switch (source)
                {
                    case lms::DataSource::LEFT_MONO_RAW:
                    {
                        if (numSubscribers(left_node_, MONO_TOPIC) == 0) continue;
                        publish_image(image, left_mono_cam_pub_, left_mono_image_, frame_id_left_);
                        break;
                    }
                    case lms::DataSource::RIGHT_MONO_RAW:
                    {
                        if (numSubscribers(right_node_, MONO_TOPIC) == 0) continue;
                        publish_image(image, right_mono_cam_pub_, right_mono_image_, frame_id_right_);
                        break;
                    }
                    //case lms::DataSource::LEFT_MONO_COMPRESSED:
                    //case lms::DataSource::RIGHT_MONO_COMPRESSED:
                    case lms::DataSource::LEFT_RECTIFIED_RAW:
                    {
                        if (numSubscribers(left_node_, RECT_TOPIC) == 0) continue;
                        publish_image(image, left_rect_cam_pub_, left_rect_image_, frame_id_rectified_left_);
                        break;
                    }
                    case lms::DataSource::RIGHT_RECTIFIED_RAW:
                    {
                        if (numSubscribers(right_node_, RECT_TOPIC) == 0) continue;
                        publish_image(image, right_rect_cam_pub_, right_rect_image_, frame_id_rectified_right_);
                        break;
                    }
                    //case lms::DataSource::LEFT_RECTIFIED_COMPRESSED:
                    //case lms::DataSource::RIGHT_RECTIFIED_COMPRESSED:
                    case lms::DataSource::LEFT_DISPARITY_RAW:
                    {
                        if (numSubscribers(left_node_, DISPARITY_TOPIC) > 0)
                        {
                            publish_image(image, left_disparity_pub_, left_disparity_image_, frame_id_rectified_left_);
                        }

                        if (numSubscribers(left_node_, DISPARITY_IMAGE_TOPIC) > 0)
                        {
                            populate_image(image, left_stereo_disparity_.image, frame_id_rectified_left_);
                            left_stereo_disparity_.f = image.calibration.P[0][0];
                            left_stereo_disparity_.t = -image_frame->calibration.right.rectified_translation()[0];
                            left_stereo_disparity_.min_disparity = 0.0f;
                            left_stereo_disparity_.max_disparity = 256.0f;
                            left_stereo_disparity_.delta_d = 1.0f/16.0f;

                            left_stereo_disparity_pub_->publish(left_stereo_disparity_);
                        }
                        break;
                    }
                    //case lms::DataSource::LEFT_DISPARITY_COMPRESSED:
                    //case lms::DataSource::AUX_COMPRESSED:
                    //case lms::DataSource::AUX_RECTIFIED_COMPRESSED:
                    case lms::DataSource::AUX_LUMA_RAW:
                    {
                        if (numSubscribers(aux_node_, MONO_TOPIC) == 0) continue;
                        publish_image(image, aux_mono_cam_pub_, aux_mono_image_, frame_id_aux_);
                        break;
                    }
                    case lms::DataSource::AUX_LUMA_RECTIFIED_RAW:
                    {
                        if (numSubscribers(aux_node_, RECT_TOPIC) == 0) continue;
                        publish_image(image, aux_rect_cam_pub_, aux_rect_image_, frame_id_rectified_aux_);
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
                        if (numSubscribers(aux_node_, COLOR_TOPIC) == 0) continue;
                        publish_image(image, aux_rgb_cam_pub_, aux_rgb_image_, frame_id_aux_);
                        break;
                    }
                    case lms::DataSource::AUX_RECTIFIED_RAW:
                    {
                        if (numSubscribers(aux_node_, RECT_COLOR_TOPIC) == 0) continue;
                        publish_image(image, aux_rgb_rect_cam_pub_, aux_rect_rgb_image_, frame_id_rectified_aux_);
                        break;
                    }
                    case lms::DataSource::COST_RAW:
                    {
                        if (numSubscribers(left_node_, COST_TOPIC) == 0) continue;
                        publish_image(image, left_disparity_cost_pub_, left_disparity_cost_image_, frame_id_rectified_left_);
                        break;
                    }
                    default: { RCLCPP_ERROR(get_logger(), "Camera: unknown image source"); continue;}
                }
            }
        }
    }
}

void Camera::depth_publisher()
{
    constexpr auto disparity_source = lms::DataSource::LEFT_DISPARITY_RAW;

    const auto timeout = std::make_optional(std::chrono::milliseconds{500});
    while (!shutdown_)
    {
        if (const auto image_frame = image_frame_notifier_.wait(timeout) ; image_frame)
        {
            if (image_frame->has_image(disparity_source))
            {
                if (numSubscribers(left_node_, DEPTH_TOPIC) > 0)
                {
                    if (const auto depth_image = lms::create_depth_image(image_frame.value(),
                                                                         lms::Image::PixelFormat::FLOAT32,
                                                                         disparity_source,
                                                                         std::numeric_limits<float>::quiet_NaN());
                                                                         depth_image)
                    {
                        publish_image(depth_image.value(), depth_cam_pub_, depth_image_, frame_id_rectified_left_);
                    }
                }

                if (numSubscribers(left_node_, OPENNI_DEPTH_TOPIC) > 0)
                {
                    if (const auto depth_image = lms::create_depth_image(image_frame.value(),
                                                                         lms::Image::PixelFormat::MONO16,
                                                                         disparity_source,
                                                                         0);
                                                                         depth_image)
                    {
                        publish_image(depth_image.value(), ni_depth_cam_pub_, ni_depth_image_, frame_id_rectified_left_);
                    }
                }
            }
        }
    }
}

void Camera::point_cloud_publisher()
{
    constexpr auto disparity_source = lms::DataSource::LEFT_DISPARITY_RAW;

    const auto timeout = std::make_optional(std::chrono::milliseconds{500});
    while (!shutdown_)
    {
        if (const auto image_frame = image_frame_notifier_.wait(timeout) ; image_frame)
        {
            if (image_frame->has_image(disparity_source))
            {
                if (numSubscribers(this, POINTCLOUD_TOPIC) > 0 && image_frame->has_image(lms::DataSource::LEFT_RECTIFIED_RAW))
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
                                                image_frame->frame_time,
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
                                                image_frame->frame_time,
                                                luma_point_cloud_pub_,
                                                luma_point_cloud_,
                                                frame_id_rectified_left_);
                        }
                    }
                }

                if (numSubscribers(this, COLOR_POINTCLOUD_TOPIC) > 0 &&
                    image_frame->has_image(lms::DataSource::AUX_LUMA_RECTIFIED_RAW) &&
                    image_frame->has_image(lms::DataSource::AUX_CHROMA_RECTIFIED_RAW))
                {
                    const auto bgr_image = lms::create_bgr_image(image_frame->get_image(lms::DataSource::AUX_LUMA_RECTIFIED_RAW),
                                                                 image_frame->get_image(lms::DataSource::AUX_CHROMA_RECTIFIED_RAW),
                                                                 lms::DataSource::AUX_RECTIFIED_RAW);

                    if (bgr_image)
                    {
                        if (const auto point_cloud = lms::create_color_pointcloud<std::array<uint8_t, 3>>(image_frame->get_image(disparity_source),
                                                                                                          bgr_image,
                                                                                                          pointcloud_max_range_,
                                                                                                          image_frame->calibration);
                                                                                                          point_cloud)
                        {
                            publish_point_cloud(point_cloud.value(),
                                                image_frame->frame_time,
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

void Camera::color_publisher()
{
    const auto timeout = std::make_optional(std::chrono::milliseconds{500});
    while (!shutdown_)
    {
        if (const auto image_frame = image_frame_notifier_.wait(timeout) ; image_frame)
        {
            if (numSubscribers(aux_node_, COLOR_TOPIC) > 0)
            {
                if (auto bgr_image = lms::create_bgr_image(image_frame->get_image(lms::DataSource::AUX_LUMA_RAW),
                                                             image_frame->get_image(lms::DataSource::AUX_CHROMA_RAW),
                                                             lms::DataSource::AUX_RAW);
                                                             bgr_image)
                {
                    publish_image(bgr_image.value(), aux_rgb_cam_pub_, aux_rgb_image_, frame_id_aux_);
                }
            }

            if (numSubscribers(aux_node_, RECT_COLOR_TOPIC) > 0)
            {
                if (auto bgr_image = lms::create_bgr_image(image_frame->get_image(lms::DataSource::AUX_LUMA_RECTIFIED_RAW),
                                                           image_frame->get_image(lms::DataSource::AUX_CHROMA_RECTIFIED_RAW),
                                                           lms::DataSource::AUX_RECTIFIED_RAW);
                                                           bgr_image)
                {
                    publish_image(bgr_image.value(), aux_rgb_rect_cam_pub_, aux_rect_rgb_image_, frame_id_rectified_aux_);
                }
            }
        }
    }
}

void Camera::stop()
{
    if (const auto status = channel_->stop_streams({lms::DataSource::ALL}); status != lms::Status::OK)
    {
        RCLCPP_ERROR(get_logger(), "Camera: failed to stop all streams: %s", lms::to_string(status).c_str());
    }

    std::lock_guard<std::mutex> lock{stream_mutex_};
    active_streams_.clear();
}

std::shared_ptr<ImagePublisher> Camera::add_image_publisher(rclcpp::Node::SharedPtr node,
                                                            const std::string &topic_name,
                                                            const sensor_msgs::msg::CameraInfo &camera_info,
                                                            const rclcpp::QoS &qos,
                                                            const multisense::DataSource &source,
                                                            bool use_image_transport)
{
    rclcpp::PublisherOptions options;
    options.event_callbacks.matched_callback =
        [this, source](const auto &info)
        {
            std::lock_guard<std::mutex> lock{this->stream_mutex_};

            if (info.current_count == 1)
            {
                this->active_streams_.push_back(source);
            }
            else if (info.current_count <= 0)
            {
                this->active_streams_.erase(std::find(std::begin(this->active_streams_),
                                                      std::end(this->active_streams_),
                                                      source));
            }

            if (const auto status = this->channel_->start_streams(this->active_streams_) ; status != lms::Status::OK)
            {
                RCLCPP_ERROR(get_logger(), "Unable to modify active streams");
            }
        };

    return std::make_shared<ImagePublisher>(node, topic_name, camera_info, qos, options, use_image_transport);
}
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr Camera::add_pointcloud_publisher(
        const std::string &topic_name,
        const rclcpp::QoS &qos,
        const std::vector<multisense::DataSource> &sources)
{
    rclcpp::PublisherOptions options;
    options.event_callbacks.matched_callback =
        [this, sources](const auto &info)
        {
            std::lock_guard<std::mutex> lock{this->stream_mutex_};

            if (info.current_count == 1)
            {
                this->active_streams_.insert(std::end(active_streams_), std::begin(sources), std::end(sources));
            }
            else if (info.current_count <= 0)
            {
                for (const auto &source : sources)
                {
                    this->active_streams_.erase(std::find(std::begin(this->active_streams_),
                                                          std::end(this->active_streams_),
                                                          source));
                }
            }

            if (const auto status = this->channel_->start_streams(this->active_streams_) ; status != lms::Status::OK)
            {
                RCLCPP_ERROR(get_logger(), "Unable to modify active streams");
            }
        };

    return create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, qos, options);
}


void Camera::publish_static_tf(const multisense::StereoCalibration &stereo_calibration)
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

void Camera::publish_device_info(const lms::MultiSenseInfo::DeviceInfo &info,
                                 const lms::MultiSenseInfo::SensorVersion &version)
{
    multisense_msgs::msg::DeviceInfo device_info_msg;

    device_info_msg.device_name     = info.camera_name;
    device_info_msg.build_date      = info.build_date;
    device_info_msg.serial_number   = info.serial_number;
    device_info_msg.device_revision = static_cast<int>(info.hardware_revision);

    for (const auto &pcb: info.pcb_info)
    {
        device_info_msg.pcb_serial_numbers.push_back(pcb.revision);
        device_info_msg.pcb_names.push_back(pcb.name);
    }

    device_info_msg.imager_name = info.imager_name;
    device_info_msg.imager_type = static_cast<int>(info.imager_type);
    device_info_msg.imager_width = info.imager_width;
    device_info_msg.imager_height = info.imager_height;

    device_info_msg.lens_name = info.lens_name;
    device_info_msg.lens_type = static_cast<int>(info.lens_type);
    device_info_msg.nominal_baseline = info.nominal_stereo_baseline;
    device_info_msg.nominal_focal_length = info.nominal_focal_length;
    device_info_msg.nominal_relative_aperture = info.nominal_relative_aperture;

    device_info_msg.lighting_type = static_cast<int>(info.lighting_type);

    device_info_msg.firmware_build_date = version.firmware_build_date;
    // TODO (malvarado): Fix this
    //device_info_msg.firmware_version = infon.firmware_version;
    device_info_msg.bitstream_version = version.hardware_version;

    device_info_pub_->publish(device_info_msg);

}

size_t Camera::numSubscribers(const rclcpp::Node::SharedPtr node, const std::string &topic)
{
    return numSubscribers(node.get(), topic);
}

size_t Camera::numSubscribers(const rclcpp::Node* node, const std::string &topic)
{
    const std::string full_topic = node->get_sub_namespace().empty() ? topic : node->get_sub_namespace()  + "/" + topic;

    if (!rclcpp::ok())
    {
        return 0;
    }

    return node->count_subscribers(full_topic) ;
}

//void Camera::initalizeParameters(const image::Config& config)
//{
//    //
//    // Sensor resolution
//
//    std::string valid_resolutions = "valid_resolutions [width, height, max disparity value]\n";
//    for (const auto& device_mode : device_modes_)
//    {
//        valid_resolutions += "[";
//        valid_resolutions += std::to_string(device_mode.width) + ", ";
//        valid_resolutions += std::to_string(device_mode.height) + ", ";
//        valid_resolutions += std::to_string(device_mode.disparities) + "]\n";
//    }
//
//
//    rcl_interfaces::msg::ParameterDescriptor sensor_resolution_desc;
//    sensor_resolution_desc.set__name("sensor_resolution")
//                          .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY)
//                          .set__description(valid_resolutions);
//    declare_parameter("sensor_resolution",
//                      std::vector<int64_t>{config.width(), config.height(), config.disparities()}, sensor_resolution_desc);
//    //
//    // Fps
//
//    float max_fps = 30.0f;
//
//    switch(device_info_.hardwareRevision)
//    {
//        case system::DeviceInfo::HARDWARE_REV_MULTISENSE_ST21:
//        {
//            max_fps = 29.97f;
//
//            break;
//        }
//        default:
//        {
//            switch (device_info_.imagerType)
//            {
//                case system::DeviceInfo::IMAGER_TYPE_CMV2000_GREY:
//                case system::DeviceInfo::IMAGER_TYPE_CMV2000_COLOR:
//                {
//                    max_fps = 30.0f;
//
//                    break;
//                }
//                case system::DeviceInfo::IMAGER_TYPE_CMV4000_GREY:
//                case system::DeviceInfo::IMAGER_TYPE_CMV4000_COLOR:
//                {
//                    max_fps = 15.0f;
//
//                    break;
//                }
//            }
//            break;
//        }
//    };
//
//    rcl_interfaces::msg::FloatingPointRange fps_range;
//    fps_range.set__from_value(1.0)
//              .set__to_value(max_fps);
//
//    rcl_interfaces::msg::ParameterDescriptor fps_desc;
//    fps_desc.set__name("fps")
//            .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//            .set__description("camera fps")
//            .set__floating_point_range({fps_range});
//    declare_parameter("fps", 10.0, fps_desc);
//
//    //
//    // Stereo Post Filtering
//
//    rcl_interfaces::msg::FloatingPointRange stereo_post_filter_range;
//    stereo_post_filter_range.set__from_value(0.0)
//                            .set__to_value(1.0);
//
//    rcl_interfaces::msg::ParameterDescriptor stereo_post_filter_desc;
//    stereo_post_filter_desc.set__name("stereo_post_filtering")
//                           .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//                           .set__description("SGM stereo post filter")
//                           .set__floating_point_range({stereo_post_filter_range});
//    declare_parameter("stereo_post_filtering", 0.75, stereo_post_filter_desc);
//
//    if (device_info_.hardwareRevision != system::DeviceInfo::HARDWARE_REV_MULTISENSE_ST21)
//    {
//
//        //
//        // Gain
//        if (next_gen_camera_)
//        {
//            rcl_interfaces::msg::FloatingPointRange gain_range;
//            gain_range.set__from_value(1.68421)
//                      .set__to_value(16.0);
//
//            rcl_interfaces::msg::ParameterDescriptor gain_desc;
//            gain_desc.set__name("gain")
//                     .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//                     .set__description("imager gain")
//                     .set__floating_point_range({gain_range});
//            declare_parameter("gain", 1.68421, gain_desc);
//        }
//        else
//        {
//            rcl_interfaces::msg::FloatingPointRange gain_range;
//            gain_range.set__from_value(1.0)
//                      .set__to_value(16.0);
//
//            rcl_interfaces::msg::ParameterDescriptor gain_desc;
//            gain_desc.set__name("gain")
//                     .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//                     .set__description("imager gain")
//                     .set__floating_point_range({gain_range});
//            declare_parameter("gain", 1.0, gain_desc);
//        }
//
//
//        //
//        // Auto exposure enable
//
//        rcl_interfaces::msg::ParameterDescriptor auto_exposure_enable_desc;
//        auto_exposure_enable_desc.set__name("auto_exposure")
//                                 .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
//                                 .set__description("enable auto exposure");
//        declare_parameter("auto_exposure", true, auto_exposure_enable_desc);
//
//        //
//        // Auto exposure max time
//
//        rcl_interfaces::msg::FloatingPointRange auto_exposure_max_time_range;
//        auto_exposure_max_time_range.set__from_value(0.0)
//                                    .set__to_value(0.033)
//                                    .set__step(0.001);
//
//        rcl_interfaces::msg::ParameterDescriptor auto_exposure_max_time_desc;
//        auto_exposure_max_time_desc.set__name("auto_exposure_max_time")
//                                   .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//                                   .set__description("max exposure time using auto exposure")
//                                   .set__floating_point_range({auto_exposure_max_time_range});
//        declare_parameter("auto_exposure_max_time", 0.01, auto_exposure_max_time_desc);
//
//        //
//        // Auto exposure decay
//
//        rcl_interfaces::msg::IntegerRange auto_exposure_decay_range;
//        auto_exposure_decay_range.set__from_value(1)
//                                 .set__to_value(10);
//
//        rcl_interfaces::msg::ParameterDescriptor auto_exposure_decay_desc;
//        auto_exposure_decay_desc.set__name("auto_exposure_decay")
//                                .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
//                                .set__description("auto exposure time decay")
//                                .set__integer_range({auto_exposure_decay_range});
//        declare_parameter("auto_exposure_decay", 7, auto_exposure_decay_desc);
//
//        //
//        // Auto exposure threshold
//
//        rcl_interfaces::msg::FloatingPointRange auto_exposure_thresh_range;
//        auto_exposure_thresh_range.set__from_value(0.0)
//                                  .set__to_value(1.0);
//
//        rcl_interfaces::msg::ParameterDescriptor auto_exposure_thresh_desc;
//        auto_exposure_thresh_desc.set__name("auto_exposure_thresh")
//                                .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//                                .set__description("auto exposure threshold")
//                                .set__floating_point_range({auto_exposure_thresh_range});
//
//        declare_parameter("auto_exposure_thresh", (next_gen_camera_ ? 0.85 : 0.75), auto_exposure_thresh_desc);
//
//        //
//        // Auto exposure target intensity
//
//        rcl_interfaces::msg::FloatingPointRange auto_exposure_target_intensity_range;
//        auto_exposure_target_intensity_range.set__from_value(0.0)
//                                            .set__to_value(1.0)
//                                            .set__step(0.01);
//
//        rcl_interfaces::msg::ParameterDescriptor auto_exposure_target_intensity_desc;
//        auto_exposure_target_intensity_desc.set__name("auto_exposure_target_intensity")
//                                           .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//                                           .set__description("auto exposure target intensity")
//                                           .set__floating_point_range({auto_exposure_target_intensity_range});
//        declare_parameter("auto_exposure_target_intensity", 0.5, auto_exposure_target_intensity_desc);
//
//        //
//        // Exposure time
//
//        rcl_interfaces::msg::FloatingPointRange exposure_time_range;
//        exposure_time_range.set__from_value(0.0)
//                           .set__to_value(0.033);
//
//        rcl_interfaces::msg::ParameterDescriptor exposure_time_desc;
//        exposure_time_desc.set__name("exposure_time")
//                          .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//                          .set__description("imager exposure time in seconds")
//                          .set__floating_point_range({exposure_time_range});
//        declare_parameter("exposure_time", 0.025, exposure_time_desc);
//
//        if (!has_aux_camera_ &&  supports_color_)
//        {
//            //
//            // Auto white balance enable
//
//            rcl_interfaces::msg::ParameterDescriptor auto_white_balance_enable_desc;
//            auto_white_balance_enable_desc.set__name("auto_white_balance")
//                     .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
//                     .set__description("enable auto white balance");
//            declare_parameter("auto_white_balance", true, auto_white_balance_enable_desc);
//
//            //
//            // Auto white balance decay
//
//            rcl_interfaces::msg::IntegerRange auto_white_balance_decay_range;
//            auto_white_balance_decay_range.set__from_value(0)
//                                          .set__to_value(20);
//
//            rcl_interfaces::msg::ParameterDescriptor auto_white_balance_decay_desc;
//            auto_white_balance_decay_desc.set__name("auto_white_balance_decay")
//                                         .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
//                                         .set__description("auto white balance decay")
//                                         .set__integer_range({auto_white_balance_decay_range});
//            declare_parameter("auto_white_balance_decay", 3, auto_white_balance_decay_desc);
//
//            //
//            // Auto white balance thresh
//
//            rcl_interfaces::msg::FloatingPointRange auto_white_balance_thresh_range;
//            auto_white_balance_thresh_range.set__from_value(0.0)
//                                           .set__to_value(1.0);
//
//            rcl_interfaces::msg::ParameterDescriptor auto_white_balance_thresh_desc;
//            auto_white_balance_thresh_desc.set__name("auto_white_balance_thresh")
//                                          .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//                                          .set__description("auto white balance threshold")
//                                          .set__floating_point_range({auto_white_balance_thresh_range});
//            declare_parameter("auto_white_balance_thresh", 0.5, auto_white_balance_thresh_desc);
//
//            //
//            // Auto white balance red
//
//            rcl_interfaces::msg::FloatingPointRange auto_white_balance_red_range;
//            auto_white_balance_red_range.set__from_value(0.2)
//                                        .set__to_value(4.0);
//
//            rcl_interfaces::msg::ParameterDescriptor auto_white_balance_red_desc;
//            auto_white_balance_red_desc.set__name("auto_white_balance_red")
//                                       .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//                                       .set__description("auto white balance red gain")
//                                       .set__floating_point_range({auto_white_balance_red_range});
//            declare_parameter("auto_white_balance_red", 1.0, auto_white_balance_red_desc);
//
//            //
//            // Auto white balance blue
//
//            rcl_interfaces::msg::FloatingPointRange auto_white_balance_blue_range;
//            auto_white_balance_blue_range.set__from_value(0.2)
//                                          .set__to_value(4.0);
//
//            rcl_interfaces::msg::ParameterDescriptor auto_white_balance_blue_desc;
//            auto_white_balance_blue_desc.set__name("auto_white_balance_blue")
//                                          .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//                                          .set__description("auto white balance blue gain")
//                                          .set__floating_point_range({auto_white_balance_blue_range});
//            declare_parameter("auto_white_balance_blue", 1.0, auto_white_balance_blue_desc);
//        }
//
//        //
//        // HDR enable
//
//        rcl_interfaces::msg::ParameterDescriptor hdr_enable_desc;
//        hdr_enable_desc.set__name("hdr_enable")
//                       .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
//                       .set__description("enable hdr");
//        declare_parameter("hdr_enable", false, hdr_enable_desc);
//
//        if (next_gen_camera_)
//        {
//            //
//            // Gamma
//            //
//            rcl_interfaces::msg::FloatingPointRange gamma_range;
//            gamma_range.set__from_value(1.0)
//                       .set__to_value(2.2)
//                       .set__step(0.01);
//
//            rcl_interfaces::msg::ParameterDescriptor gamma_desc;
//            gamma_desc.set__name("gamma")
//                       .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//                       .set__description("Gamma")
//                       .set__floating_point_range({gamma_range});
//            declare_parameter("gamma", 2.2, gamma_desc);
//        }
//
//        //
//        // Enable ROI auto exposure control
//
//        rcl_interfaces::msg::ParameterDescriptor auto_exposure_roi_enable_desc;
//        auto_exposure_roi_enable_desc.set__name("auto_exposure_roi_enable")
//                                   .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
//                                   .set__description("enable auto exposure ROI");
//        declare_parameter("auto_exposure_roi_enable", false, auto_exposure_roi_enable_desc);
//
//        //
//        // Auto exposure ROI x
//        //
//        rcl_interfaces::msg::IntegerRange auto_exposure_roi_x_range;
//        auto_exposure_roi_x_range.set__from_value(0)
//                       .set__to_value(device_info_.imagerWidth)
//                       .set__step(1);
//
//        rcl_interfaces::msg::ParameterDescriptor auto_exposure_roi_x_desc;
//        auto_exposure_roi_x_desc.set__name("auto_exposure_roi_x")
//                                 .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
//                                 .set__description("auto exposure ROI x value")
//                                 .set__integer_range({auto_exposure_roi_x_range});
//        declare_parameter("auto_exposure_roi_x", 0, auto_exposure_roi_x_desc);
//
//        //
//        // Auto exposure ROI y
//        //
//        rcl_interfaces::msg::IntegerRange auto_exposure_roi_y_range;
//        auto_exposure_roi_y_range.set__from_value(0)
//                       .set__to_value(device_info_.imagerHeight)
//                       .set__step(1);
//
//        rcl_interfaces::msg::ParameterDescriptor auto_exposure_roi_y_desc;
//        auto_exposure_roi_y_desc.set__name("auto_exposure_roi_y")
//                                 .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
//                                 .set__description("auto exposure ROI y value")
//                                 .set__integer_range({auto_exposure_roi_y_range});
//        declare_parameter("auto_exposure_roi_y", 0, auto_exposure_roi_y_desc);
//
//        //
//        // Auto exposure ROI width
//        //
//        rcl_interfaces::msg::IntegerRange auto_exposure_roi_width_range;
//        auto_exposure_roi_width_range.set__from_value(0)
//                       .set__to_value(device_info_.imagerWidth)
//                       .set__step(1);
//
//        rcl_interfaces::msg::ParameterDescriptor auto_exposure_roi_width_desc;
//        auto_exposure_roi_width_desc.set__name("auto_exposure_roi_width")
//                                 .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
//                                 .set__description("auto exposure ROI width value")
//                                 .set__integer_range({auto_exposure_roi_width_range});
//        declare_parameter("auto_exposure_roi_width", crl::multisense::Roi_Full_Image, auto_exposure_roi_width_desc);
//
//        //
//        // Auto exposure ROI height
//        //
//        rcl_interfaces::msg::IntegerRange auto_exposure_roi_height_range;
//        auto_exposure_roi_height_range.set__from_value(0)
//                       .set__to_value(device_info_.imagerHeight)
//                       .set__step(1);
//
//        rcl_interfaces::msg::ParameterDescriptor auto_exposure_roi_height_desc;
//        auto_exposure_roi_height_desc.set__name("auto_exposure_roi_height")
//                                 .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
//                                 .set__description("auto exposure ROI height value")
//                                 .set__integer_range({auto_exposure_roi_height_range});
//        declare_parameter("auto_exposure_roi_height", crl::multisense::Roi_Full_Image, auto_exposure_roi_height_desc);
//
//    }
//
//    if (has_aux_camera_ && aux_control_supported_)
//    {
//        //
//        // Aux Gain
//
//        rcl_interfaces::msg::FloatingPointRange aux_gain_range;
//        aux_gain_range.set__from_value(1.68421)
//                      .set__to_value(16.0)
//                      .set__step(0.00001);
//
//        rcl_interfaces::msg::ParameterDescriptor aux_gain_desc;
//        aux_gain_desc.set__name("aux_gain")
//                     .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//                     .set__description("aux_imager gain")
//                     .set__floating_point_range({aux_gain_range});
//        declare_parameter("aux_gain", 1.68421, aux_gain_desc);
//
//        //
//        // Aux Auto exposure enable
//
//        rcl_interfaces::msg::ParameterDescriptor aux_auto_exposure_enable_desc;
//        aux_auto_exposure_enable_desc.set__name("aux_auto_exposure")
//                                     .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
//                                     .set__description("aus enable auto exposure");
//        declare_parameter("aux_auto_exposure", true, aux_auto_exposure_enable_desc);
//
//        //
//        // Aux Auto exposure max time
//
//        rcl_interfaces::msg::FloatingPointRange aux_auto_exposure_max_time_range;
//        aux_auto_exposure_max_time_range.set__from_value(0.0)
//                                        .set__to_value(0.033)
//                                        .set__step(0.001);
//
//        rcl_interfaces::msg::ParameterDescriptor aux_auto_exposure_max_time_desc;
//        aux_auto_exposure_max_time_desc.set__name("aux_auto_exposure_max_time")
//                                       .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//                                       .set__description("aux max exposure time using auto exposure")
//                                       .set__floating_point_range({aux_auto_exposure_max_time_range});
//        declare_parameter("aux_auto_exposure_max_time", 0.01, aux_auto_exposure_max_time_desc);
//
//        //
//        // Aux Auto exposure decay
//
//        rcl_interfaces::msg::IntegerRange aux_auto_exposure_decay_range;
//        aux_auto_exposure_decay_range.set__from_value(1)
//                                     .set__to_value(10);
//
//        rcl_interfaces::msg::ParameterDescriptor aux_auto_exposure_decay_desc;
//        aux_auto_exposure_decay_desc.set__name("aux_auto_exposure_decay")
//                                    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
//                                    .set__description("aux_auto exposure time decay")
//                                    .set__integer_range({aux_auto_exposure_decay_range});
//        declare_parameter("aux_auto_exposure_decay", 7, aux_auto_exposure_decay_desc);
//
//        //
//        // Aux Auto exposure threshold
//
//        rcl_interfaces::msg::FloatingPointRange aux_auto_exposure_thresh_range;
//        aux_auto_exposure_thresh_range.set__from_value(0.0)
//                                      .set__to_value(1.0)
//                                      .set__step(0.01);
//
//        rcl_interfaces::msg::ParameterDescriptor aux_auto_exposure_thresh_desc;
//        aux_auto_exposure_thresh_desc.set__name("aux_auto_exposure_thresh")
//                                 .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//                                 .set__description("aux auto exposure threshold")
//                                 .set__floating_point_range({aux_auto_exposure_thresh_range});
//        declare_parameter("aux_auto_exposure_thresh", 0.85, aux_auto_exposure_thresh_desc);
//
//        //
//        // Aux Auto exposure target intensity
//
//        rcl_interfaces::msg::FloatingPointRange aux_auto_exposure_target_intensity_range;
//        aux_auto_exposure_target_intensity_range.set__from_value(0.0)
//                                                .set__to_value(1.0)
//                                                .set__step(0.01);
//
//        rcl_interfaces::msg::ParameterDescriptor aux_auto_exposure_target_intensity_desc;
//        aux_auto_exposure_target_intensity_desc.set__name("aux_auto_exposure_target_intensity")
//                                                .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//                                                .set__description("aux auto exposure target intensity")
//                                                .set__floating_point_range({aux_auto_exposure_target_intensity_range});
//        declare_parameter("aux_auto_exposure_target_intensity", 0.5, aux_auto_exposure_target_intensity_desc);
//
//        //
//        // Aux Exposure time
//
//        rcl_interfaces::msg::FloatingPointRange aux_exposure_time_range;
//        aux_exposure_time_range.set__from_value(0.0)
//                               .set__to_value(0.033)
//                               .set__step(0.000001);
//
//        rcl_interfaces::msg::ParameterDescriptor aux_exposure_time_desc;
//        aux_exposure_time_desc.set__name("aux_exposure_time")
//                              .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//                              .set__description("aux imager exposure time in seconds")
//                              .set__floating_point_range({aux_exposure_time_range});
//        declare_parameter("aux_exposure_time", 0.005, aux_exposure_time_desc);
//
//        //
//        // Aux Auto white balance enable
//
//        rcl_interfaces::msg::ParameterDescriptor aux_auto_white_balance_enable_desc;
//        aux_auto_white_balance_enable_desc.set__name("aux_ auto_white_balance")
//                                          .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
//                                          .set__description("enable aux auto white balance");
//        declare_parameter("aux_auto_white_balance", true, aux_auto_white_balance_enable_desc);
//
//        //
//        // Aux Auto white balance decay
//
//        rcl_interfaces::msg::IntegerRange aux_auto_white_balance_decay_range;
//        aux_auto_white_balance_decay_range.set__from_value(0)
//                                          .set__to_value(20);
//
//        rcl_interfaces::msg::ParameterDescriptor aux_auto_white_balance_decay_desc;
//        aux_auto_white_balance_decay_desc.set__name("aux_auto_white_balance_decay")
//                                         .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
//                                         .set__description("aux auto white balance decay")
//                                         .set__integer_range({aux_auto_white_balance_decay_range});
//        declare_parameter("aux_auto_white_balance_decay", 3, aux_auto_white_balance_decay_desc);
//
//        //
//        // Aux Auto white balance thresh
//
//        rcl_interfaces::msg::FloatingPointRange aux_auto_white_balance_thresh_range;
//        aux_auto_white_balance_thresh_range.set__from_value(0.0)
//                                           .set__to_value(1.0)
//                                           .set__step(0.01);
//
//        rcl_interfaces::msg::ParameterDescriptor aux_auto_white_balance_thresh_desc;
//        aux_auto_white_balance_thresh_desc.set__name("aux_auto_white_balance_thresh")
//                                          .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//                                          .set__description("aux auto white balance threshold")
//                                          .set__floating_point_range({aux_auto_white_balance_thresh_range});
//        declare_parameter("aux_auto_white_balance_thresh", 0.5, aux_auto_white_balance_thresh_desc);
//
//        //
//        // Aux Auto white balance red
//
//        rcl_interfaces::msg::FloatingPointRange aux_auto_white_balance_red_range;
//        aux_auto_white_balance_red_range.set__from_value(0.25)
//                                        .set__to_value(4.0)
//                                        .set__step(0.01);
//
//        rcl_interfaces::msg::ParameterDescriptor aux_auto_white_balance_red_desc;
//        aux_auto_white_balance_red_desc.set__name("aux_auto_white_balance_red")
//                                       .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//                                       .set__description("aux auto white balance red gain")
//                                       .set__floating_point_range({aux_auto_white_balance_red_range});
//        declare_parameter("aux_auto_white_balance_red", 1.0, aux_auto_white_balance_red_desc);
//
//        //
//        // Aux Auto white balance blue
//
//        rcl_interfaces::msg::FloatingPointRange aux_auto_white_balance_blue_range;
//        aux_auto_white_balance_blue_range.set__from_value(0.25)
//                                         .set__to_value(4.0)
//                                         .set__step(0.01);
//
//        rcl_interfaces::msg::ParameterDescriptor aux_auto_white_balance_blue_desc;
//        aux_auto_white_balance_blue_desc.set__name("aux_auto_white_balance_blue")
//                                         .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//                                         .set__description("aux auto white balance blue gain")
//                                         .set__floating_point_range({aux_auto_white_balance_blue_range});
//        declare_parameter("aux_auto_white_balance_blue", 1.0, aux_auto_white_balance_blue_desc);
//
//        //
//        // Aux HDR enable
//
//        rcl_interfaces::msg::ParameterDescriptor aux_hdr_enable_desc;
//        aux_hdr_enable_desc.set__name("aux_hdr_enable")
//                       .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
//                       .set__description("enable aux hdr");
//        declare_parameter("aux_hdr_enable", false, aux_hdr_enable_desc);
//
//        //
//        // Aux gamma
//        //
//        rcl_interfaces::msg::FloatingPointRange aux_gamma_range;
//        aux_gamma_range.set__from_value(1.0)
//                       .set__to_value(2.2)
//                       .set__step(0.01);
//
//        rcl_interfaces::msg::ParameterDescriptor aux_gamma_desc;
//        aux_gamma_desc.set__name("aux_gamma")
//                      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//                      .set__description("aux gamma")
//                      .set__floating_point_range({aux_gamma_range});
//        declare_parameter("aux_gamma", 2.2, aux_gamma_desc);
//
//        //
//        // Aux enable sharpening
//
//        rcl_interfaces::msg::ParameterDescriptor aux_sharpening_enable_desc;
//        aux_sharpening_enable_desc.set__name("aux_sharpening_enable")
//                                   .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
//                                   .set__description("enable aux sharpening");
//        declare_parameter("aux_sharpening_enable", false, aux_sharpening_enable_desc);
//
//        //
//        // Aux sharpening percentage
//        //
//        rcl_interfaces::msg::FloatingPointRange aux_sharpening_percentage_range;
//        aux_sharpening_percentage_range.set__from_value(0.0)
//                                       .set__to_value(100.0)
//                                       .set__step(0.01);
//
//        rcl_interfaces::msg::ParameterDescriptor aux_sharpening_percentage_desc;
//        aux_sharpening_percentage_desc.set__name("aux_sharpening_percentage")
//                                      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//                                      .set__description("aux sharpening percentage")
//                                      .set__floating_point_range({aux_sharpening_percentage_range});
//        declare_parameter("aux_sharpening_percentage", 0.0, aux_sharpening_percentage_desc);
//
//        //
//        // Aux sharpening limit
//        //
//        rcl_interfaces::msg::IntegerRange aux_sharpening_limit_range;
//        aux_sharpening_limit_range.set__from_value(0)
//                                  .set__to_value(255)
//                                  .set__step(1);
//
//        rcl_interfaces::msg::ParameterDescriptor aux_sharpening_limit_desc;
//        aux_sharpening_limit_desc.set__name("aux_sharpening_limit")
//                                 .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
//                                 .set__description("aux sharpening limit")
//                                 .set__integer_range({aux_sharpening_limit_range});
//        declare_parameter("aux_sharpening_limit", 0, aux_sharpening_limit_desc);
//
//        //
//        // Aux enable ROI auto exposure
//
//        rcl_interfaces::msg::ParameterDescriptor aux_auto_exposure_roi_enable_desc;
//        aux_auto_exposure_roi_enable_desc.set__name("aux_auto_exposure_roi_enable")
//                                   .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
//                                   .set__description("enable aux auto exposure ROI");
//        declare_parameter("aux_auto_exposure_roi_enable", false, aux_auto_exposure_roi_enable_desc);
//
//        //
//        // Aux ROI auto exposure x
//        //
//        rcl_interfaces::msg::IntegerRange aux_auto_exposure_roi_x_range;
//        aux_auto_exposure_roi_x_range.set__from_value(0)
//                       .set__to_value(device_info_.imagerWidth)
//                       .set__step(1);
//
//        rcl_interfaces::msg::ParameterDescriptor aux_auto_exposure_roi_x_desc;
//        aux_auto_exposure_roi_x_desc.set__name("aux_auto_exposure_roi_x")
//                                 .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
//                                 .set__description("aux auto exposure ROI x value")
//                                 .set__integer_range({aux_auto_exposure_roi_x_range});
//        declare_parameter("aux_auto_exposure_roi_x", 0, aux_auto_exposure_roi_x_desc);
//
//        //
//        // Aux ROI auto exposure y
//        //
//        rcl_interfaces::msg::IntegerRange aux_auto_exposure_roi_y_range;
//        aux_auto_exposure_roi_y_range.set__from_value(0)
//                       .set__to_value(device_info_.imagerHeight)
//                       .set__step(1);
//
//        rcl_interfaces::msg::ParameterDescriptor aux_auto_exposure_roi_y_desc;
//        aux_auto_exposure_roi_y_desc.set__name("aux_auto_exposure_roi_y")
//                                 .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
//                                 .set__description("aux auto exposure ROI y value")
//                                 .set__integer_range({aux_auto_exposure_roi_y_range});
//        declare_parameter("aux_auto_exposure_roi_y", 0, aux_auto_exposure_roi_y_desc);
//
//        //
//        // Aux ROI auto exposure width
//        //
//        rcl_interfaces::msg::IntegerRange aux_auto_exposure_roi_width_range;
//        aux_auto_exposure_roi_width_range.set__from_value(0)
//                       .set__to_value(device_info_.imagerWidth)
//                       .set__step(1);
//
//        rcl_interfaces::msg::ParameterDescriptor aux_auto_exposure_roi_width_desc;
//        aux_auto_exposure_roi_width_desc.set__name("aux_auto_exposure_roi_width")
//                                        .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
//                                        .set__description("aux auto exposure ROI width value")
//                                        .set__integer_range({aux_auto_exposure_roi_width_range});
//        declare_parameter("aux_auto_exposure_roi_width", crl::multisense::Roi_Full_Image, aux_auto_exposure_roi_width_desc);
//
//        //
//        // Aux ROI auto exposure height
//        //
//        rcl_interfaces::msg::IntegerRange aux_auto_exposure_roi_height_range;
//        aux_auto_exposure_roi_height_range.set__from_value(0)
//                       .set__to_value(device_info_.imagerHeight)
//                       .set__step(1);
//
//        rcl_interfaces::msg::ParameterDescriptor aux_auto_exposure_roi_height_desc;
//        aux_auto_exposure_roi_height_desc.set__name("aux_auto_exposure_roi_height")
//                                 .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
//                                 .set__description("aux auto exposure ROI height value")
//                                 .set__integer_range({aux_auto_exposure_roi_height_range});
//        declare_parameter("aux_auto_exposure_roi_height", crl::multisense::Roi_Full_Image, aux_auto_exposure_roi_height_desc);
//
//    }
//
//
//    //
//    // Border clip type
//
//    rcl_interfaces::msg::IntegerRange border_clip_type_range;
//    border_clip_type_range.set__from_value(static_cast<int>(BorderClip::NONE))
//                          .set__to_value(static_cast<int>(BorderClip::CIRCULAR))
//                          .set__step(1);
//
//    rcl_interfaces::msg::ParameterDescriptor border_clip_type_desc;
//    border_clip_type_desc.set__name("border_clip_type")
//                         .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
//                         .set__description("border clip type\n0: none\n1: rectangular clip\n2: circular clip")
//                         .set__integer_range({border_clip_type_range});
//    declare_parameter("border_clip_type", static_cast<int>(BorderClip::NONE), border_clip_type_desc);
//
//    //
//    // Border clip value
//
//    const double max_clip = (device_info_.imagerType == system::DeviceInfo::IMAGER_TYPE_CMV2000_GREY ||
//                             device_info_.imagerType == system::DeviceInfo::IMAGER_TYPE_CMV4000_COLOR) ? 400.0 : 200.0;
//    rcl_interfaces::msg::FloatingPointRange border_clip_value_range;
//    border_clip_value_range.set__from_value(0.0)
//                           .set__to_value(max_clip);
//
//    rcl_interfaces::msg::ParameterDescriptor border_clip_value_desc;
//    border_clip_value_desc.set__name("border_clip_value")
//                          .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//                          .set__description("border clip value in pixels")
//                          .set__floating_point_range({border_clip_value_range});
//    declare_parameter("border_clip_value", 0.0, border_clip_value_desc);
//
//    //
//    // Max pointcloud range
//
//    rcl_interfaces::msg::FloatingPointRange max_pointcloud_range_range;
//    max_pointcloud_range_range.set__from_value(0.1)
//                              .set__to_value(1000.0);
//
//    rcl_interfaces::msg::ParameterDescriptor max_pointcloud_range_desc;
//    max_pointcloud_range_desc.set__name("max pointcloud range")
//                             .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
//                             .set__description("max distance in meters between a stereo point and the camera")
//                             .set__floating_point_range({max_pointcloud_range_range});
//    declare_parameter("max_pointcloud_range", pointcloud_max_range_, max_pointcloud_range_desc);
//}
//
//rcl_interfaces::msg::SetParametersResult Camera::parameterCallback(const std::vector<rclcpp::Parameter>& parameters)
//{
//    rcl_interfaces::msg::SetParametersResult result;
//    result.set__successful(true);
//
//    auto image_config = stereo_calibration_manager_->config();
//    bool update_image_config = false;
//
//    std::optional<image::AuxConfig> aux_image_config = image::AuxConfig{};
//    bool update_aux_image_config = false;
//    if (aux_control_supported_)
//    {
//        if (const auto status = channel_->getAuxImageConfig(aux_image_config.value()); status != Status_Ok)
//        {
//            RCLCPP_WARN(get_logger(), "Camera: failed to query aux sensor configuration: %s",
//                        Channel::statusString(status));
//
//            aux_image_config = std::nullopt;
//        }
//    }
//    else
//    {
//        aux_image_config = std::nullopt;
//    }
//
//    for (const auto &parameter : parameters)
//    {
//        const auto type = parameter.get_type();
//        if (type == rclcpp::ParameterType::PARAMETER_NOT_SET)
//        {
//            continue;
//        }
//
//        const auto name = parameter.get_name();
//
//        if (name == "sensor_resolution")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
//            {
//                return result.set__successful(false).set__reason("invalid sensor resolution type");
//            }
//
//            const auto value = parameter.as_integer_array();
//
//            if (value.size() != 3)
//            {
//                return result.set__successful(false)
//                             .set__reason("Camera: Invalid sensor resolution. Must be [width, height, max disparity]");
//            }
//
//            const auto width = value[0];
//            const auto height = value[1];
//            const auto disparities = value[2];
//
//            //
//            // Ensure this is a valid resolution supported by the device
//
//            const bool supported = std::find_if(std::begin(device_modes_), std::end(device_modes_),
//                                                [&width, &height, &disparities](const system::DeviceMode& m)
//                                                {
//                                                    return width == static_cast<int64_t>(m.width) &&
//                                                           height == static_cast<int64_t>(m.height) &&
//                                                           disparities == static_cast<int64_t>(m.disparities);
//
//                                                }) != std::end(device_modes_);
//
//            if (!supported)
//            {
//                return result.set__successful(false).set__reason("Camera: unsupported resolution");
//            }
//
//            if (image_config.width() != width ||
//                image_config.height() != height ||
//                image_config.disparities() != disparities)
//            {
//                RCLCPP_WARN(get_logger(), "Camera: changing sensor resolution to %ldx%ld (%ld disparities), from %ux%u "
//                     "(%u disparities): reconfiguration may take up to 30 seconds",
//                         width, height, disparities,
//                         image_config.width(), image_config.height(), image_config.disparities());
//
//                image_config.setResolution(width, height);
//                image_config.setDisparities(disparities);
//                update_image_config = true;
//            }
//        }
//        else if(name == "fps")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid fps type");
//            }
//
//            const auto value = get_as_number<double>(parameter);
//            if (image_config.fps() != value)
//            {
//                image_config.setFps(value);
//                update_image_config = true;
//            }
//        }
//        else if(name == "stereo_post_filtering")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid stereo post filtering type");
//            }
//
//            const auto value = get_as_number<double>(parameter);
//            if (image_config.stereoPostFilterStrength() != value)
//            {
//                image_config.setStereoPostFilterStrength(value);
//                update_image_config = true;
//            }
//        }
//        else if(name == "gain")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid gain type");
//            }
//
//            const auto value = get_as_number<double>(parameter);
//            if (image_config.gain() != value)
//            {
//                image_config.setGain(value);
//                update_image_config = true;
//            }
//        }
//        else if(name == "auto_exposure")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
//            {
//                return result.set__successful(false).set__reason("invalid auto exposure type");
//            }
//
//            const auto value = parameter.as_bool();
//            if (image_config.autoExposure() != value)
//            {
//                image_config.setAutoExposure(value);
//                update_image_config = true;
//            }
//        }
//        else if(name == "auto_exposure_max_time")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid auto exposure max time type");
//            }
//
//            const auto value = static_cast<uint32_t>(get_as_number<double>(parameter) * 1e6);
//            if (image_config.autoExposureMax() != value)
//            {
//                image_config.setAutoExposureMax(value);
//                update_image_config = true;
//            }
//        }
//        else if(name == "auto_exposure_decay")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid auto exposure decay type");
//            }
//
//            const auto value = get_as_number<double>(parameter);
//            if (image_config.autoExposureDecay() != value)
//            {
//                image_config.setAutoExposureDecay(value);
//                update_image_config = true;
//            }
//        }
//        else if(name == "auto_exposure_thresh")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid auto exposure thresh type");
//            }
//
//            const auto value = get_as_number<double>(parameter);
//            if (image_config.autoExposureThresh() != value)
//            {
//                image_config.setAutoExposureThresh(value);
//                update_image_config = true;
//            }
//        }
//        else if(name == "auto_exposure_target_intensity")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid auto exposure target intensity type");
//            }
//
//            const auto value = get_as_number<double>(parameter);
//            if (image_config.autoExposureTargetIntensity() != value)
//            {
//                image_config.setAutoExposureTargetIntensity(value);
//                update_image_config = true;
//            }
//        }
//        else if(name == "exposure_time")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid exposure time type");
//            }
//
//            const auto value = static_cast<uint32_t>(get_as_number<double>(parameter) * 1e6);
//            if (image_config.exposure() != value)
//            {
//                image_config.setExposure(value);
//                update_image_config = true;
//            }
//        }
//        else if(name == "auto_white_balance")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
//            {
//                return result.set__successful(false).set__reason("invalid auto white balance type");
//            }
//
//            const auto value = parameter.as_bool();
//            if (image_config.autoWhiteBalance() != value)
//            {
//                image_config.setAutoWhiteBalance(value);
//                update_image_config = true;
//            }
//        }
//        else if(name == "auto_white_balance_decay")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid auto white balance decay type");
//            }
//
//            const auto value = get_as_number<double>(parameter);
//            if (image_config.autoWhiteBalanceDecay() != value)
//            {
//                image_config.setAutoWhiteBalanceDecay(value);
//                update_image_config = true;
//            }
//        }
//        else if(name == "auto_white_balance_thresh")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid auto white balance thresh type");
//            }
//
//            const auto value = get_as_number<double>(parameter);
//            if (image_config.autoWhiteBalanceThresh() != value)
//            {
//                image_config.setAutoWhiteBalanceThresh(value);
//                update_image_config = true;
//            }
//        }
//        else if(name == "auto_white_balance_red")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid auto white balance red type");
//            }
//
//            const auto value = get_as_number<double>(parameter);
//            if (image_config.whiteBalanceRed() != value)
//            {
//                image_config.setWhiteBalance(value, image_config.whiteBalanceBlue());
//                update_image_config = true;
//            }
//        }
//        else if(name == "auto_white_balance_blue")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid auto white balance blue type");
//            }
//
//            const auto value = get_as_number<double>(parameter);
//            if (image_config.whiteBalanceBlue() != value)
//            {
//                image_config.setWhiteBalance(image_config.whiteBalanceRed(), value);
//                update_image_config = true;
//            }
//        }
//        else if(name == "hdr_enable")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
//            {
//                return result.set__successful(false).set__reason("invalid hdr enable type");
//            }
//
//            const auto value = parameter.as_bool();
//            if (image_config.hdrEnabled() != value)
//            {
//                image_config.setHdr(value);
//                update_image_config = true;
//            }
//        }
//        else if(name == "gamma")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid gamma type");
//            }
//
//            const auto value = get_as_number<double>(parameter);
//            if (image_config.gamma() != value)
//            {
//                image_config.setGamma(value);
//                update_image_config = true;
//            }
//        }
//        else if(name == "auto_exposure_roi_enable")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
//            {
//                return result.set__successful(false).set__reason("invalid auto exposure ROI enable");
//            }
//
//            const auto value = parameter.as_bool();
//            if (enable_roi_auto_exposure_control_ != value)
//            {
//                enable_roi_auto_exposure_control_ = value;
//                update_image_config = true;
//            }
//        }
//        else if(name == "auto_exposure_roi_x")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid auto exposure ROI x");
//            }
//
//            const auto value = get_as_number<int>(parameter);
//            if (auto_exposure_roi_.x != value)
//            {
//                auto_exposure_roi_.x = value;
//                update_image_config = true;
//            }
//        }
//        else if(name == "auto_exposure_roi_y")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid auto exposure ROI y");
//            }
//
//            const auto value = get_as_number<int>(parameter);
//            if (auto_exposure_roi_.y != value)
//            {
//                auto_exposure_roi_.y = value;
//                update_image_config = true;
//            }
//        }
//        else if(name == "auto_exposure_roi_width")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid auto exposure ROI width");
//            }
//
//            const auto value = get_as_number<int>(parameter);
//            if (auto_exposure_roi_.width != value)
//            {
//                auto_exposure_roi_.width = value;
//                update_image_config = true;
//            }
//        }
//        else if(name == "auto_exposure_roi_height")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid auto exposure ROI height");
//            }
//
//            const auto value = get_as_number<int>(parameter);
//            if (auto_exposure_roi_.height != value)
//            {
//                auto_exposure_roi_.height = value;
//                update_image_config = true;
//            }
//        }
//        else if(name == "aux_gain" && aux_image_config)
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid aux gain type");
//            }
//
//            const auto value = get_as_number<double>(parameter);
//            if (aux_image_config->gain() != value)
//            {
//                aux_image_config->setGain(value);
//                update_aux_image_config = true;
//            }
//        }
//        else if(name == "aux_auto_exposure" && aux_image_config)
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
//            {
//                return result.set__successful(false).set__reason("invalid aux auto exposure type");
//            }
//
//            const auto value = parameter.as_bool();
//            if (aux_image_config->autoExposure() != value)
//            {
//                aux_image_config->setAutoExposure(value);
//                update_aux_image_config = true;
//            }
//        }
//        else if(name == "aux_auto_exposure_max_time" && aux_image_config)
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid aux auto exposure max time type");
//            }
//
//            const auto value = static_cast<uint32_t>(get_as_number<double>(parameter) * 1e6);
//            if (aux_image_config->autoExposureMax() != value)
//            {
//                aux_image_config->setAutoExposureMax(value);
//                update_aux_image_config = true;
//            }
//        }
//        else if(name == "aux_auto_exposure_decay" && aux_image_config)
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid aux auto exposure decay type");
//            }
//
//            const auto value = get_as_number<double>(parameter);
//            if (aux_image_config->autoExposureDecay() != value)
//            {
//                aux_image_config->setAutoExposureDecay(value);
//                update_aux_image_config = true;
//            }
//        }
//        else if(name == "aux_auto_exposure_thresh" && aux_image_config)
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid aux auto exposure thresh type");
//            }
//
//            const auto value = get_as_number<double>(parameter);
//            if (aux_image_config->autoExposureThresh() != value)
//            {
//                aux_image_config->setAutoExposureThresh(value);
//                update_aux_image_config = true;
//            }
//        }
//        else if(name == "aux_auto_exposure_target_intensity" && aux_image_config)
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid aux auto exposure target intensity type");
//            }
//
//            const auto value = get_as_number<double>(parameter);
//            if (aux_image_config->autoExposureTargetIntensity() != value)
//            {
//                aux_image_config->setAutoExposureTargetIntensity(value);
//                update_aux_image_config = true;
//            }
//        }
//        else if(name == "aux_exposure_time" && aux_image_config)
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid aux exposure time type");
//            }
//
//            const auto value = static_cast<uint32_t>(get_as_number<double>(parameter) * 1e6);
//            if (aux_image_config->exposure() != value)
//            {
//                aux_image_config->setExposure(value);
//                update_aux_image_config = true;
//            }
//        }
//        else if(name == "aux_auto_white_balance" && aux_image_config)
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
//            {
//                return result.set__successful(false).set__reason("invalid aux auto white balance type");
//            }
//
//            const auto value = parameter.as_bool();
//            if (aux_image_config->autoWhiteBalance() != value)
//            {
//                aux_image_config->setAutoWhiteBalance(value);
//                update_aux_image_config = true;
//            }
//        }
//        else if(name == "aux_auto_white_balance_decay" && aux_image_config)
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid aux auto white balance decay type");
//            }
//
//            const auto value = get_as_number<double>(parameter);
//            if (aux_image_config->autoWhiteBalanceDecay() != value)
//            {
//                aux_image_config->setAutoWhiteBalanceDecay(value);
//                update_aux_image_config = true;
//            }
//        }
//        else if(name == "aux_auto_white_balance_thresh" && aux_image_config)
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid aux auto white balance thresh type");
//            }
//
//            const auto value = get_as_number<double>(parameter);
//            if (aux_image_config->autoWhiteBalanceThresh() != value)
//            {
//                aux_image_config->setAutoWhiteBalanceThresh(value);
//                update_aux_image_config = true;
//            }
//        }
//        else if(name == "aux_auto_white_balance_red" && aux_image_config)
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid aux auto white balance red type");
//            }
//
//            const auto value = get_as_number<double>(parameter);
//            if (aux_image_config->whiteBalanceRed() != value)
//            {
//                aux_image_config->setWhiteBalance(value, aux_image_config->whiteBalanceBlue());
//                update_aux_image_config = true;
//            }
//        }
//        else if(name == "aux_auto_white_balance_blue" && aux_image_config)
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid aux auto white balance blue type");
//            }
//
//            const auto value = get_as_number<double>(parameter);
//            if (aux_image_config->whiteBalanceBlue() != value)
//            {
//                aux_image_config->setWhiteBalance(aux_image_config->whiteBalanceRed(), value);
//                update_aux_image_config = true;
//            }
//        }
//        else if(name == "aux_hdr_enable" && aux_image_config)
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
//            {
//                return result.set__successful(false).set__reason("invalid aux hdr enable type");
//            }
//
//            const auto value = parameter.as_bool();
//            if (aux_image_config->hdrEnabled() != value)
//            {
//                aux_image_config->setHdr(value);
//                update_aux_image_config = true;
//            }
//        }
//        else if(name == "aux_gamma" && aux_image_config)
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid aux gamma type");
//            }
//
//            const auto value = get_as_number<double>(parameter);
//            if (aux_image_config->gamma() != value)
//            {
//                aux_image_config->setGamma(value);
//                update_aux_image_config = true;
//            }
//        }
//        else if(name == "aux_sharpening_enable" && aux_image_config)
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
//            {
//                return result.set__successful(false).set__reason("invalid aux sharpening enable type");
//            }
//
//            const auto value = parameter.as_bool();
//            if (aux_image_config->enableSharpening() != value)
//            {
//                aux_image_config->enableSharpening(value);
//                update_aux_image_config = true;
//            }
//        }
//        else if(name == "aux_sharpening_percentage" && aux_image_config)
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid aux sharpening percentage type");
//            }
//
//            const auto value = get_as_number<double>(parameter);
//            if (aux_image_config->sharpeningPercentage() != value)
//            {
//                aux_image_config->setSharpeningPercentage(value);
//                update_aux_image_config = true;
//            }
//        }
//        else if(name == "aux_sharpening_limit" && aux_image_config)
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid aux sharpening limit type");
//            }
//
//            const auto value = get_as_number<int>(parameter);
//            if (aux_image_config->sharpeningLimit() != value)
//            {
//                aux_image_config->setSharpeningLimit(value);
//                update_aux_image_config = true;
//            }
//        }
//        else if(name == "aux_auto_exposure_roi_enable" && aux_image_config)
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
//            {
//                return result.set__successful(false).set__reason("invalid aux auto exposure ROI enable");
//            }
//
//            const auto value = parameter.as_bool();
//            if (enable_aux_roi_auto_exposure_control_ != value)
//            {
//                enable_aux_roi_auto_exposure_control_ = value;
//                update_aux_image_config = true;
//            }
//        }
//        else if(name == "aux_auto_exposure_roi_x" && aux_image_config)
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid aux auto exposure ROI x");
//            }
//
//            const auto value = get_as_number<int>(parameter);
//            if (aux_auto_exposure_roi_.x != value)
//            {
//                aux_auto_exposure_roi_.x = value;
//                update_aux_image_config = true;
//            }
//        }
//        else if(name == "aux_auto_exposure_roi_y" && aux_image_config)
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid aux auto exposure ROI y");
//            }
//
//            const auto value = get_as_number<int>(parameter);
//            if (aux_auto_exposure_roi_.y != value)
//            {
//                aux_auto_exposure_roi_.y = value;
//                update_aux_image_config = true;
//            }
//        }
//        else if(name == "aux_auto_exposure_roi_width" && aux_image_config)
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid aux auto exposure ROI width");
//            }
//
//            const auto value = get_as_number<int>(parameter);
//            if (aux_auto_exposure_roi_.width != value)
//            {
//                aux_auto_exposure_roi_.width = value;
//                update_aux_image_config = true;
//            }
//        }
//        else if(name == "aux_auto_exposure_roi_height" && aux_image_config)
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid aux auto exposure ROI height");
//            }
//
//            const auto value = get_as_number<int>(parameter);
//            if (aux_auto_exposure_roi_.height != value)
//            {
//                aux_auto_exposure_roi_.height = value;
//                update_aux_image_config = true;
//            }
//        }
//        else if (name == "border_clip_type")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid border clip type");
//            }
//
//            const auto value = static_cast<BorderClip>(get_as_number<int>(parameter));
//            if (border_clip_type_ != value)
//            {
//                border_clip_type_ = value;
//            }
//        }
//        else if (name == "border_clip_value")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid border clip value type");
//            }
//
//            const auto value = get_as_number<double>(parameter);
//            if (border_clip_value_ != value)
//            {
//                border_clip_value_ = value;
//            }
//        }
//        else if (name == "max_pointcloud_range")
//        {
//            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
//            {
//                return result.set__successful(false).set__reason("invalid max pointcloud range type");
//            }
//
//            pointcloud_max_range_ = get_as_number<double>(parameter);
//        }
//    }
//
//    //
//    // Update our ROI if we are enabled/disabled
//    if (enable_roi_auto_exposure_control_)
//    {
//        image_config.setAutoExposureRoi(auto_exposure_roi_.x,
//                                        auto_exposure_roi_.y,
//                                        auto_exposure_roi_.width,
//                                        auto_exposure_roi_.height);
//    }
//    else
//    {
//        image_config.setAutoExposureRoi(0, 0, crl::multisense::Roi_Full_Image, crl::multisense::Roi_Full_Image);
//    }
//
//    if (update_image_config)
//    {
//        if (const auto status = channel_->setImageConfig(image_config); status != Status_Ok)
//        {
//            return result.set__successful(false).set__reason(Channel::statusString(status));
//        }
//
//        //
//        // TODO remove after firmware fixes
//        std::this_thread::sleep_for(std::chrono::milliseconds(500));
//
//        if (const auto status = channel_->getImageConfig(image_config); status != Status_Ok)
//        {
//            RCLCPP_ERROR(get_logger(), "Camera: failed to query sensor configuration: %s",
//                         Channel::statusString(status));
//
//            return result.set__successful(false).set__reason(Channel::statusString(status));
//        }
//
//        //
//        // This is a no-op if the resolution of the camera did not change
//    }
//
//    //
//    // Update our aux ROI if we are enabled/disabled
//    if (enable_aux_roi_auto_exposure_control_)
//    {
//        aux_image_config->setAutoExposureRoi(aux_auto_exposure_roi_.x,
//                                             aux_auto_exposure_roi_.y,
//                                             aux_auto_exposure_roi_.width,
//                                             aux_auto_exposure_roi_.height);
//    }
//    else
//    {
//        aux_image_config->setAutoExposureRoi(0, 0, crl::multisense::Roi_Full_Image, crl::multisense::Roi_Full_Image);
//    }
//
//    if (update_aux_image_config && aux_image_config)
//    {
//        if (const auto status = channel_->setAuxImageConfig(aux_image_config.value()); status != Status_Ok)
//        {
//            return result.set__successful(false).set__reason(Channel::statusString(status));
//        }
//
//        //
//        // TODO remove after firmware fixes
//        std::this_thread::sleep_for(std::chrono::milliseconds(500));
//    }
//
//    return result;
//}

} // namespace
