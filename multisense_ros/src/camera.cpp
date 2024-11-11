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

#if defined(ROS_FOXY) || defined(ROS_GALACTIC)
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <multisense_msgs/msg/raw_cam_config.hpp>
#include <multisense_msgs/msg/raw_cam_cal.hpp>
#include <multisense_msgs/msg/device_info.hpp>
#include <multisense_msgs/msg/histogram.hpp>
#include <multisense_ros/camera.h>
#include <multisense_ros/parameter_utilities.h>
#include <multisense_ros/point_cloud_utilities.h>

using namespace crl::multisense;
using namespace std::chrono_literals;

namespace multisense_ros {

namespace { // anonymous

tf2::Matrix3x3 toRotation(float R[3][3])
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

//
// All of the data sources that we control here

constexpr DataSource allImageSources = (Source_Luma_Left            |
                                        Source_Luma_Right           |
                                        Source_Luma_Rectified_Left  |
                                        Source_Luma_Rectified_Right |
                                        Source_Chroma_Left          |
                                        Source_Luma_Aux             |
                                        Source_Luma_Rectified_Aux   |
                                        Source_Chroma_Aux           |
                                        Source_Chroma_Rectified_Aux |
                                        Source_Disparity            |
                                        Source_Disparity_Right      |
                                        Source_Disparity_Cost);

//
// Shims for C-style driver callbacks

void monoCB(const image::Header& header, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->monoCallback(header); }
void rectCB(const image::Header& header, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->rectCallback(header); }
void depthCB(const image::Header& header, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->depthCallback(header); }
void pointCB(const image::Header& header, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->pointCloudCallback(header); }
void rawCB(const image::Header& header, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->rawCamDataCallback(header); }
void colorCB(const image::Header& header, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->colorImageCallback(header); }
void dispCB(const image::Header& header, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->disparityImageCallback(header); }
void histCB(const image::Header& header, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->histogramCallback(header); }
void colorizeCB(const image::Header& header, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->colorizeCallback(header); }

bool isValidReprojectedPoint(const Eigen::Vector3f& pt, double squared_max_range)
{
    return pt[2] > 0.0f && std::isfinite(pt[2]) && pt.squaredNorm() < squared_max_range;
}

void writePoint(sensor_msgs::msg::PointCloud2 &pointcloud, size_t index, const Eigen::Vector3f &point, uint32_t color)
{
    float* cloudP = reinterpret_cast<float*>(&(pointcloud.data[index * pointcloud.point_step]));
    cloudP[0] = point[0];
    cloudP[1] = point[1];
    cloudP[2] = point[2];

    uint32_t* colorP = reinterpret_cast<uint32_t*>(&(cloudP[3]));
    colorP[0] = color;
}

void writePoint(sensor_msgs::msg::PointCloud2 &pointcloud,
                size_t pointcloud_index,
                const Eigen::Vector3f &point,
                size_t image_index,
                const image::Header &image)
{
    switch (image.bitsPerPixel)
    {
        case 8:
        {
            const uint32_t luma = static_cast<uint32_t>(reinterpret_cast<const uint8_t*>(image.imageDataP)[image_index]);
            return writePoint(pointcloud, pointcloud_index, point, luma);
        }
        case 16:
        {
            const uint32_t luma = static_cast<uint32_t>(reinterpret_cast<const uint16_t*>(image.imageDataP)[image_index]);
            return writePoint(pointcloud, pointcloud_index, point, luma);
        }
        case 32:
        {
            const uint32_t luma = reinterpret_cast<const uint32_t*>(image.imageDataP)[image_index];
            return writePoint(pointcloud, pointcloud_index, point, luma);
        }
    }
}

bool clipPoint(const BorderClip& borderClipType,
                      double borderClipValue,
                      size_t height,
                      size_t width,
                      size_t u,
                      size_t v)
{
    //
    // Precompute the maximum radius from the center of the image for a point
    // to be considered in the circle

    switch (borderClipType)
    {
        case BorderClip::NONE:
        {
            return false;
        }
        case BorderClip::RECTANGULAR:
        {
            return !( u >= borderClipValue && u <= width - borderClipValue &&
                      v >= borderClipValue && v <= height - borderClipValue);
        }
        case BorderClip::CIRCULAR:
        {
            const double halfWidth = static_cast<double>(width)/2.0;
            const double halfHeight = static_cast<double>(height)/2.0;

            const double radius = sqrt( halfWidth * halfWidth + halfHeight * halfHeight ) - borderClipValue;

            return !(Eigen::Vector2d{halfWidth - u, halfHeight - v}.norm() < radius);
        }
        default:
        {
            break;
        }
    }

    return true;
}

cv::Vec3b u_interpolate_color(double u, double v, const cv::Mat &image)
{
    const double width = image.cols;

    //
    // Interpolate in just the u dimension
    //
    const size_t min_u = static_cast<size_t>(std::min(std::max(std::floor(u), 0.), width - 1.));
    const size_t max_u = static_cast<size_t>(std::min(std::max(std::ceil(u), 0.), width - 1.));

    const cv::Vec3d element0 = image.at<cv::Vec3b>(width * v + min_u);
    const cv::Vec3d element1 = image.at<cv::Vec3b>(width * v + max_u);

    const size_t delta_u = max_u - min_u;

    const double u_ratio = delta_u == 0 ? 1. : (static_cast<double>(max_u) - u) / static_cast<double>(delta_u);

    const cv::Vec3b result = (element0 * u_ratio + element1 * (1. - u_ratio));

    return result;
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
               crl::multisense::Channel* driver,
               const std::string& tf_prefix):
    Node(node_name, options),
    driver_(driver),
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
    pointcloud_max_range_(15.0),
    active_streams_(Source_Unknown),
    last_frame_id_(-1),
    border_clip_type_(BorderClip::NONE),
    border_clip_value_(0)
{
    //
    // Query device and version information from sensor

    if (const auto status = driver_->getVersionInfo(version_info_); status != Status_Ok )
    {
        RCLCPP_ERROR(get_logger(), "Camera: failed to query version info: %s",
                     Channel::statusString(status));
        return;
    }

    if (const auto status = driver_->getDeviceInfo(device_info_); status != Status_Ok)
    {
        RCLCPP_ERROR(get_logger(), "Camera: failed to query device info: %s",
                     Channel::statusString(status));
        return;
    }

    if (const auto status = driver_->getDeviceModes(device_modes_); status != Status_Ok)
    {
        RCLCPP_ERROR(get_logger(), "Camera: failed to query device modes: %s",
                     Channel::statusString(status));
        return;
    }

    if (system::DeviceInfo::HARDWARE_REV_MULTISENSE_C6S2_S27 == device_info_.hardwareRevision ||
        system::DeviceInfo::HARDWARE_REV_MULTISENSE_S30 == device_info_.hardwareRevision)
    {
        if (version_info_.sensorFirmwareVersion >= 0x0600) {
            aux_control_supported_ = true;
        }
        else
        {
            aux_control_supported_ = false;
            RCLCPP_WARN(get_logger(),
                        "Camera: MultiSense firmware version does not support the updated aux camera exposure controls. "
                        "The ROS2 driver will work normally, but you will have limited control over aux camera exposure parameters. "
                        "Please contact support@carnegierobotics.com for an updated firmware version greater than 6.0 to "
                        "enable aux camera exposure controls.");
        }
    }

    //
    // Get the camera config

    image::Config image_config;
    if (const auto status = driver_->getImageConfig(image_config); status != Status_Ok)
    {
        RCLCPP_ERROR(get_logger(), "Camera: failed to query sensor configuration: %s",
                     Channel::statusString(status));
        return;
    }

    //
    // Get the calibraton

    image::Calibration image_calibration;
    if (const auto status = driver_->getImageCalibration(image_calibration); status != Status_Ok)
    {
        RCLCPP_ERROR(get_logger(), "Camera: failed to query sensor calibration: %s",
                     Channel::statusString(status));
        return;
    }

    //
    // S27/S30 cameras have a 3rd aux color camera and no left color camera

    has_aux_camera_ = system::DeviceInfo::HARDWARE_REV_MULTISENSE_C6S2_S27 == device_info_.hardwareRevision ||
                      system::DeviceInfo::HARDWARE_REV_MULTISENSE_S30 == device_info_.hardwareRevision;

    supports_color_ = has_aux_camera_ ||
                      (system::DeviceInfo::HARDWARE_REV_MULTISENSE_KS21 != device_info_.hardwareRevision &&
                       system::DeviceInfo::HARDWARE_REV_MULTISENSE_ST21 != device_info_.hardwareRevision &&
                       system::DeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_VPB != device_info_.hardwareRevision &&
                       system::DeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_STEREO != device_info_.hardwareRevision &&
                       system::DeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_MONOCAM != device_info_.hardwareRevision);

    next_gen_camera_ = system::DeviceInfo::HARDWARE_REV_MULTISENSE_C6S2_S27 == device_info_.hardwareRevision ||
                       system::DeviceInfo::HARDWARE_REV_MULTISENSE_S30 == device_info_.hardwareRevision ||
                       system::DeviceInfo::HARDWARE_REV_MULTISENSE_KS21 == device_info_.hardwareRevision ||
                       system::DeviceInfo::HARDWARE_REV_MULTISENSE_MONOCAM == device_info_.hardwareRevision ||
                       system::DeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_STEREO == device_info_.hardwareRevision ||
                       system::DeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_MONOCAM == device_info_.hardwareRevision;

    stereo_calibration_manager_ = std::make_shared<StereoCalibrationManger>(image_config, image_calibration, device_info_);

    timer_ = create_wall_timer(500ms, std::bind(&Camera::timerCallback, this));

    //
    // Latching QoS. Note subscribers need to also use the transient_local durability

    const auto latching_qos = rclcpp::QoS(1).transient_local();

    //
    // Topics published for all device types

    device_info_pub_ = calibration_node_->create_publisher<multisense_msgs::msg::DeviceInfo>(DEVICE_INFO_TOPIC, latching_qos);
    raw_cam_cal_pub_ = calibration_node_->create_publisher<multisense_msgs::msg::RawCamCal>(RAW_CAM_CAL_TOPIC, latching_qos);
    raw_cam_config_pub_ = calibration_node_->create_publisher<multisense_msgs::msg::RawCamConfig>(RAW_CAM_CONFIG_TOPIC, latching_qos);
    histogram_pub_ = create_publisher<multisense_msgs::msg::Histogram>(HISTOGRAM_TOPIC, rclcpp::SensorDataQoS());

    //
    // Image publishers

    left_mono_cam_pub_  = left_node_->create_publisher<sensor_msgs::msg::Image>(MONO_TOPIC, rclcpp::SensorDataQoS());
    right_mono_cam_pub_ = right_node_->create_publisher<sensor_msgs::msg::Image>(MONO_TOPIC, rclcpp::SensorDataQoS());
    left_rect_cam_pub_  = left_node_->create_publisher<sensor_msgs::msg::Image>(RECT_TOPIC, rclcpp::SensorDataQoS());
    right_rect_cam_pub_ = right_node_->create_publisher<sensor_msgs::msg::Image>(RECT_TOPIC, rclcpp::SensorDataQoS());
    depth_cam_pub_      = left_node_->create_publisher<sensor_msgs::msg::Image>(DEPTH_TOPIC, rclcpp::SensorDataQoS());
    ni_depth_cam_pub_   = left_node_->create_publisher<sensor_msgs::msg::Image>(OPENNI_DEPTH_TOPIC, rclcpp::SensorDataQoS());

    if (supports_color_)
    {
        if (has_aux_camera_)
        {
            aux_mono_cam_pub_ = aux_node_->create_publisher<sensor_msgs::msg::Image>(MONO_TOPIC, rclcpp::SensorDataQoS());
            aux_rgb_cam_pub_ = aux_node_->create_publisher<sensor_msgs::msg::Image>(COLOR_TOPIC, rclcpp::SensorDataQoS());
            aux_rect_cam_pub_ = aux_node_->create_publisher<sensor_msgs::msg::Image>(RECT_TOPIC, rclcpp::SensorDataQoS());
            aux_rgb_rect_cam_pub_ = aux_node_->create_publisher<sensor_msgs::msg::Image>(RECT_COLOR_TOPIC, rclcpp::SensorDataQoS());

            aux_mono_cam_info_pub_ = aux_node_->create_publisher<sensor_msgs::msg::CameraInfo>(MONO_CAMERA_INFO_TOPIC, latching_qos);
            aux_rgb_cam_info_pub_ = aux_node_->create_publisher<sensor_msgs::msg::CameraInfo>(COLOR_CAMERA_INFO_TOPIC, latching_qos);
            aux_rect_cam_info_pub_ = aux_node_->create_publisher<sensor_msgs::msg::CameraInfo>(RECT_CAMERA_INFO_TOPIC, latching_qos);
            aux_rgb_rect_cam_info_pub_ = aux_node_->create_publisher<sensor_msgs::msg::CameraInfo>(RECT_COLOR_CAMERA_INFO_TOPIC, latching_qos);
        }
        else
        {
            left_rgb_cam_pub_   = left_node_->create_publisher<sensor_msgs::msg::Image>(COLOR_TOPIC, rclcpp::SensorDataQoS());
            left_rgb_rect_cam_pub_ = left_node_->create_publisher<sensor_msgs::msg::Image>(RECT_COLOR_TOPIC, rclcpp::SensorDataQoS());

            left_rgb_cam_info_pub_  = left_node_->create_publisher<sensor_msgs::msg::CameraInfo>(COLOR_CAMERA_INFO_TOPIC, latching_qos);
            left_rgb_rect_cam_info_pub_  = left_node_->create_publisher<sensor_msgs::msg::CameraInfo>(RECT_COLOR_CAMERA_INFO_TOPIC, latching_qos);
        }

        color_point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(COLOR_POINTCLOUD_TOPIC, rclcpp::SensorDataQoS());
        color_organized_point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(COLOR_ORGANIZED_POINTCLOUD_TOPIC, rclcpp::SensorDataQoS());
    }

    luma_point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(POINTCLOUD_TOPIC, rclcpp::SensorDataQoS());
    luma_organized_point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(ORGANIZED_POINTCLOUD_TOPIC, rclcpp::SensorDataQoS());
    raw_cam_data_pub_   = calibration_node_->create_publisher<multisense_msgs::msg::RawCamData>(RAW_CAM_DATA_TOPIC, rclcpp::SensorDataQoS());

    left_disparity_pub_ = left_node_->create_publisher<sensor_msgs::msg::Image>(DISPARITY_TOPIC, rclcpp::SensorDataQoS());

    left_stereo_disparity_pub_ = left_node_->create_publisher<stereo_msgs::msg::DisparityImage>(DISPARITY_IMAGE_TOPIC, rclcpp::SensorDataQoS());

    right_disparity_pub_ = right_node_->create_publisher<sensor_msgs::msg::Image>(DISPARITY_TOPIC, rclcpp::SensorDataQoS());
    right_stereo_disparity_pub_ = right_node_->create_publisher<stereo_msgs::msg::DisparityImage>(DISPARITY_IMAGE_TOPIC, rclcpp::SensorDataQoS());
    left_disparity_cost_pub_ = left_node_->create_publisher<sensor_msgs::msg::Image>(COST_TOPIC, rclcpp::SensorDataQoS());

    //
    // Camera info topic publishers

    left_mono_cam_info_pub_ = left_node_->create_publisher<sensor_msgs::msg::CameraInfo>(MONO_CAMERA_INFO_TOPIC, latching_qos);
    right_mono_cam_info_pub_ = right_node_->create_publisher<sensor_msgs::msg::CameraInfo>(MONO_CAMERA_INFO_TOPIC, latching_qos);
    left_rect_cam_info_pub_ = left_node_->create_publisher<sensor_msgs::msg::CameraInfo>(RECT_CAMERA_INFO_TOPIC, latching_qos);
    right_rect_cam_info_pub_ = right_node_->create_publisher<sensor_msgs::msg::CameraInfo>(RECT_CAMERA_INFO_TOPIC, latching_qos);
    left_disp_cam_info_pub_ = left_node_->create_publisher<sensor_msgs::msg::CameraInfo>(DISPARITY_CAMERA_INFO_TOPIC, latching_qos);
    depth_cam_info_pub_ = left_node_->create_publisher<sensor_msgs::msg::CameraInfo>(DEPTH_CAMERA_INFO_TOPIC, latching_qos);
    right_disp_cam_info_pub_ = right_node_->create_publisher<sensor_msgs::msg::CameraInfo>(DISPARITY_CAMERA_INFO_TOPIC, latching_qos);
    left_cost_cam_info_pub_ = left_node_->create_publisher<sensor_msgs::msg::CameraInfo>(COST_CAMERA_INFO_TOPIC, latching_qos);

    //
    // All image streams off

    stop();

    //
    // Publish device info

    multisense_msgs::msg::DeviceInfo device_info_msg;

    device_info_msg.device_name     = device_info_.name;
    device_info_msg.build_date      = device_info_.buildDate;
    device_info_msg.serial_number   = device_info_.serialNumber;
    device_info_msg.device_revision = device_info_.hardwareRevision;

    device_info_msg.number_of_pcbs = device_info_.pcbs.size();
    for (const auto &pcb: device_info_.pcbs)
    {
        device_info_msg.pcb_serial_numbers.push_back(pcb.revision);
        device_info_msg.pcb_names.push_back(pcb.name);
    }

    device_info_msg.imager_name              = device_info_.imagerName;
    device_info_msg.imager_type              = device_info_.imagerType;
    device_info_msg.imager_width             = device_info_.imagerWidth;
    device_info_msg.imager_height            = device_info_.imagerHeight;

    device_info_msg.lens_name                = device_info_.lensName;
    device_info_msg.lens_type                = device_info_.lensType;
    device_info_msg.nominal_baseline         = device_info_.nominalBaseline;
    device_info_msg.nominal_focal_length      = device_info_.nominalFocalLength;
    device_info_msg.nominal_relative_aperture = device_info_.nominalRelativeAperture;

    device_info_msg.lighting_type            = device_info_.lightingType;
    device_info_msg.number_of_lights          = device_info_.numberOfLights;

    device_info_msg.laser_name               = device_info_.laserName;
    device_info_msg.laser_type               = device_info_.laserType;

    device_info_msg.motor_name               = device_info_.motorName;
    device_info_msg.motor_type               = device_info_.motorType;
    device_info_msg.motor_gear_reduction      = device_info_.motorGearReduction;

    device_info_msg.api_build_date            = version_info_.apiBuildDate;
    device_info_msg.api_version              = version_info_.apiVersion;
    device_info_msg.firmware_build_date       = version_info_.sensorFirmwareBuildDate;
    device_info_msg.firmware_version         = version_info_.sensorFirmwareVersion;
    device_info_msg.bitstream_version        = version_info_.sensorHardwareVersion;
    device_info_msg.bitstream_magic          = version_info_.sensorHardwareMagic;
    device_info_msg.fpga_dna                 = version_info_.sensorFpgaDna;

    device_info_pub_->publish(device_info_msg);

    //
    // Publish image calibration

    multisense_msgs::msg::RawCamCal cal;
    const float *cP = reinterpret_cast<const float *>(&(image_calibration.left.M[0][0]));
    for(uint32_t i=0; i<9; i++) cal.left_m[i] = cP[i];
    cP = reinterpret_cast<const float *>(&(image_calibration.left.D[0]));
    for(uint32_t i=0; i<8; i++) cal.left_d[i] = cP[i];
    cP = reinterpret_cast<const float *>(&(image_calibration.left.R[0][0]));
    for(uint32_t i=0; i<9; i++) cal.left_r[i] = cP[i];
    cP = reinterpret_cast<const float *>(&(image_calibration.left.P[0][0]));
    for(uint32_t i=0; i<12; i++) cal.left_p[i] = cP[i];

    cP = reinterpret_cast<const float *>(&(image_calibration.right.M[0][0]));
    for(uint32_t i=0; i<9; i++) cal.right_m[i] = cP[i];
    cP = reinterpret_cast<const float *>(&(image_calibration.right.D[0]));
    for(uint32_t i=0; i<8; i++) cal.right_d[i] = cP[i];
    cP = reinterpret_cast<const float *>(&(image_calibration.right.R[0][0]));
    for(uint32_t i=0; i<9; i++) cal.right_r[i] = cP[i];
    cP = reinterpret_cast<const float *>(&(image_calibration.right.P[0][0]));
    for(uint32_t i=0; i<12; i++) cal.right_p[i] = cP[i];

    raw_cam_cal_pub_->publish(cal);

    //
    // Publish the static transforms for our camera extrinsics for the left/right/aux frames. We will
    // use the left camera frame as the reference coordinate frame

    const bool has_aux_extrinsics = has_aux_camera_ && stereo_calibration_manager_->validAux();

    std::vector<geometry_msgs::msg::TransformStamped> stamped_transforms(3 + (has_aux_extrinsics ? 2 : 0));

    tf2::Transform rectified_left_T_left{toRotation(image_calibration.left.R), tf2::Vector3{0., 0., 0.}};
    stamped_transforms[0].header.stamp = rclcpp::Clock().now();
    stamped_transforms[0].header.frame_id = frame_id_rectified_left_;
    stamped_transforms[0].child_frame_id = frame_id_left_;
    stamped_transforms[0].transform = tf2::toMsg(rectified_left_T_left);

    tf2::Transform rectified_right_T_rectified_left{tf2::Matrix3x3::getIdentity(),
                                                    tf2::Vector3{stereo_calibration_manager_->T(), 0., 0.}};
    stamped_transforms[1].header.stamp = rclcpp::Clock().now();
    stamped_transforms[1].header.frame_id = frame_id_rectified_left_;
    stamped_transforms[1].child_frame_id = frame_id_rectified_right_;
    stamped_transforms[1].transform = tf2::toMsg(rectified_right_T_rectified_left.inverse());

    tf2::Transform rectified_right_T_right{toRotation(image_calibration.right.R), tf2::Vector3{0., 0., 0.}};
    stamped_transforms[2].header.stamp = rclcpp::Clock().now();
    stamped_transforms[2].header.frame_id = frame_id_rectified_right_;
    stamped_transforms[2].child_frame_id = frame_id_right_;
    stamped_transforms[2].transform = tf2::toMsg(rectified_right_T_right);

    if (has_aux_extrinsics)
    {
        tf2::Transform rectified_aux_T_rectified_left{tf2::Matrix3x3::getIdentity(),
                                                      tf2::Vector3{stereo_calibration_manager_->aux_T(), 0., 0.}};
        stamped_transforms[3].header.stamp = rclcpp::Clock().now();
        stamped_transforms[3].header.frame_id = frame_id_rectified_left_;
        stamped_transforms[3].child_frame_id = frame_id_rectified_aux_;
        stamped_transforms[3].transform = tf2::toMsg(rectified_aux_T_rectified_left.inverse());

        tf2::Transform rectified_aux_T_aux{toRotation(image_calibration.aux.R), tf2::Vector3{0., 0., 0.}};
        stamped_transforms[4].header.stamp = rclcpp::Clock().now();
        stamped_transforms[4].header.frame_id = frame_id_rectified_aux_;
        stamped_transforms[4].child_frame_id = frame_id_aux_;
        stamped_transforms[4].transform = tf2::toMsg(rectified_aux_T_aux);
    }

    static_tf_broadcaster_->sendTransform(stamped_transforms);

    updateConfig(image_config);

    //
    // Initialze point cloud data structures

    luma_point_cloud_ = initialize_pointcloud<float>(true, frame_id_rectified_left_, "intensity");
    color_point_cloud_ = initialize_pointcloud<float>(true, frame_id_rectified_left_, "rgb");
    luma_organized_point_cloud_ = initialize_pointcloud<float>(false, frame_id_rectified_left_, "intensity");
    color_organized_point_cloud_ = initialize_pointcloud<float>(false, frame_id_rectified_left_, "rgb");

    //
    // Add driver-level callbacks.
    //
    //    -Driver creates individual background thread for each callback.
    //    -Images are queued (depth=5) per callback, with oldest silently dropped if not keeping up.
    //    -All images presented are backed by a referenced buffer system (no copying of image data is done.)

    driver_->addIsolatedCallback(colorizeCB, Source_Luma_Rectified_Aux | Source_Chroma_Rectified_Aux | Source_Luma_Aux |
                                             Source_Luma_Left | Source_Chroma_Left | Source_Luma_Rectified_Left, this);
    driver_->addIsolatedCallback(monoCB,  Source_Luma_Left | Source_Luma_Right | Source_Luma_Aux, this);
    driver_->addIsolatedCallback(rectCB,  Source_Luma_Rectified_Left | Source_Luma_Rectified_Right | Source_Luma_Rectified_Aux, this);
    driver_->addIsolatedCallback(depthCB, Source_Disparity, this);
    driver_->addIsolatedCallback(pointCB, Source_Disparity, this);
    driver_->addIsolatedCallback(rawCB,   Source_Disparity, this);
    driver_->addIsolatedCallback(colorCB, Source_Chroma_Left | Source_Chroma_Rectified_Aux | Source_Chroma_Aux, this);
    driver_->addIsolatedCallback(dispCB,  Source_Disparity | Source_Disparity_Right | Source_Disparity_Cost, this);
    driver_->addIsolatedCallback(histCB,  allImageSources, this);

    //
    // Setup parameters

    paramter_handle_ = add_on_set_parameters_callback(std::bind(&Camera::parameterCallback, this, std::placeholders::_1));

    initalizeParameters(image_config);
}

Camera::~Camera()
{
    stop();

    driver_->removeIsolatedCallback(colorizeCB);
    driver_->removeIsolatedCallback(monoCB);
    driver_->removeIsolatedCallback(rectCB);
    driver_->removeIsolatedCallback(depthCB);
    driver_->removeIsolatedCallback(pointCB);
    driver_->removeIsolatedCallback(rawCB);
    driver_->removeIsolatedCallback(colorCB);
    driver_->removeIsolatedCallback(dispCB);
}

void Camera::histogramCallback(const image::Header& header)
{
    if (last_frame_id_ >= header.frameId)
        return;

    last_frame_id_ = header.frameId;

    if (numSubscribers(this, HISTOGRAM_TOPIC) > 0)
    {
        multisense_msgs::msg::Histogram rh;
        image::Histogram          mh;

        if (const auto status = driver_->getImageHistogram(header.frameId, mh); status == Status_Ok)
        {
            rh.frame_count = header.frameId;
            rh.time_stamp = rclcpp::Time(header.timeSeconds, 1000 * header.timeMicroSeconds);
            rh.width  = header.width;
            rh.height = header.height;
            switch(header.source) {
            case Source_Chroma_Left:
            case Source_Chroma_Right:
                rh.width  *= 2;
                rh.height *= 2;
            }

            rh.exposure_time = header.exposure;
            rh.gain          = header.gain;
            rh.fps           = header.framesPerSecond;
            rh.channels      = mh.channels;
            rh.bins          = mh.bins;
            rh.data          = mh.data;
            histogram_pub_->publish(rh);
        }
    }
}

void Camera::disparityImageCallback(const image::Header& header)
{
    if (!((Source_Disparity == header.source && numSubscribers(left_node_, DISPARITY_TOPIC) > 0) ||
          (Source_Disparity_Right == header.source && numSubscribers(right_node_, DISPARITY_TOPIC) > 0) ||
          (Source_Disparity_Cost == header.source && numSubscribers(left_node_, COST_TOPIC) > 0) ||
          (Source_Disparity == header.source && numSubscribers(left_node_, DISPARITY_IMAGE_TOPIC) > 0) ||
          (Source_Disparity_Right == header.source && numSubscribers(right_node_, DISPARITY_IMAGE_TOPIC) > 0) ))
    {
        return;
    }

    const size_t image_size = (header.width * header.height * header.bitsPerPixel) / 8;

    const rclcpp::Time t(header.timeSeconds, 1000 * header.timeMicroSeconds);

    switch(header.source) {
    case Source_Disparity:
    case Source_Disparity_Right:
    {
        sensor_msgs::msg::Image *imageP   = NULL;
        sensor_msgs::msg::CameraInfo camInfo;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubP = nullptr;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camInfoPubP = nullptr;
        rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr stereoDisparityPubP = nullptr;
        stereo_msgs::msg::DisparityImage *stereoDisparityImageP = NULL;
        rclcpp::Node::SharedPtr node = nullptr;


        if (Source_Disparity == header.source)
        {
            pubP                    = left_disparity_pub_;
            imageP                  = &left_disparity_image_;
            imageP->header.frame_id = frame_id_rectified_left_;
            camInfo                 = stereo_calibration_manager_->leftCameraInfo(frame_id_rectified_left_, t);
            camInfoPubP             = left_disp_cam_info_pub_;
            stereoDisparityPubP     = left_stereo_disparity_pub_;
            stereoDisparityImageP   = &left_stereo_disparity_;
            stereoDisparityImageP->header.frame_id = frame_id_rectified_left_;
            node = left_node_;
        }
        else
        {
            pubP                    = right_disparity_pub_;
            imageP                  = &right_disparity_image_;
            imageP->header.frame_id = frame_id_rectified_right_;
            camInfo                 = stereo_calibration_manager_->rightCameraInfo(frame_id_rectified_right_, t);
            camInfoPubP             = right_disp_cam_info_pub_;
            stereoDisparityPubP     = right_stereo_disparity_pub_;
            stereoDisparityImageP   = &right_stereo_disparity_;
            stereoDisparityImageP->header.frame_id = frame_id_rectified_right_;
            node = right_node_;
        }

        if (numSubscribers(node, DISPARITY_TOPIC) > 0)
        {
            imageP->data.resize(image_size);
            memcpy(&imageP->data[0], header.imageDataP, image_size);

            imageP->header.stamp    = t;
            imageP->height          = header.height;
            imageP->width           = header.width;
            imageP->is_bigendian    = (htonl(1) == 1);

            switch(header.bitsPerPixel) {
                case 8:
                    imageP->encoding = sensor_msgs::image_encodings::MONO8;
                    imageP->step     = header.width;
                    break;
                case 16:
                    imageP->encoding = sensor_msgs::image_encodings::MONO16;
                    imageP->step     = header.width * 2;
                    break;
            }

            pubP->publish(*imageP);
        }

        if (numSubscribers(node, DISPARITY_IMAGE_TOPIC) > 0)
        {
            //
            // If our current image resolution is using non-square pixels, i.e.
            // fx != fy then warn the user. This support is lacking in
            // stereo_msgs::DisparityImage and stereo_image_proc

            if (camInfo.p[0] != camInfo.p[5])
            {
                RCLCPP_WARN(get_logger(), "Camera: Current camera configuration has non-square pixels (fx != fy).\
                                           The stereo_msgs/DisparityImage does not account for\
                                           this. Be careful when reprojecting to a pointcloud");
            }

            //
            // Our final floating point image will be serialized into uint8_t
            // meaning we need to allocate 4 bytes per pixel

            uint32_t floatingPointImageSize = header.width * header.height * 4;
            stereoDisparityImageP->image.data.resize(floatingPointImageSize);

            stereoDisparityImageP->header.stamp = t;

            stereoDisparityImageP->image.height = header.height;
            stereoDisparityImageP->image.width = header.width;
            stereoDisparityImageP->image.is_bigendian = (htonl(1) == 1);
            stereoDisparityImageP->image.header.stamp = t;
            stereoDisparityImageP->image.header.frame_id = stereoDisparityImageP->header.frame_id;
            stereoDisparityImageP->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            stereoDisparityImageP->image.step = 4 * header.width;


            stereoDisparityImageP->f = camInfo.p[0];

            //
            // Our Tx is negative. The DisparityImage message expects Tx to be
            // positive

            stereoDisparityImageP->t = fabs(stereo_calibration_manager_->T());
            stereoDisparityImageP->min_disparity = 0;
            stereoDisparityImageP->max_disparity = stereo_calibration_manager_->config().disparities();
            stereoDisparityImageP->delta_d = 1./16.;

            //
            // The stereo_msgs::DisparityImage message expects the disparity
            // image to be floating point. We will use OpenCV to perform our
            // element-wise division


            cv::Mat_<uint16_t> tmpImage(header.height,
                                        header.width,
                                        reinterpret_cast<uint16_t*>(
                                        const_cast<void*>(header.imageDataP)));

            //
            // We will copy our data directly into our output message

            cv::Mat_<float> floatingPointImage(header.height,
                                               header.width,
                                               reinterpret_cast<float*>(&stereoDisparityImageP->image.data[0]));

            //
            // Convert our disparity to floating point by dividing by 16 and
            // copy the result to the output message

            floatingPointImage = tmpImage / 16.0;

            stereoDisparityPubP->publish(*stereoDisparityImageP);
        }

        camInfoPubP->publish(camInfo);

        break;
    }
    case Source_Disparity_Cost:

        left_disparity_cost_image_.data.resize(image_size);
        memcpy(&left_disparity_cost_image_.data[0], header.imageDataP, image_size);

        left_disparity_cost_image_.header.frame_id = frame_id_rectified_left_;
        left_disparity_cost_image_.header.stamp    = t;
        left_disparity_cost_image_.height          = header.height;
        left_disparity_cost_image_.width           = header.width;

        left_disparity_cost_image_.encoding        = sensor_msgs::image_encodings::MONO8;
        left_disparity_cost_image_.is_bigendian    = (htonl(1) == 1);
        left_disparity_cost_image_.step            = header.width;

        left_disparity_cost_pub_->publish(left_disparity_cost_image_);

        left_cost_cam_info_pub_->publish(stereo_calibration_manager_->leftCameraInfo(frame_id_rectified_left_, t));

        break;
    }
}

void Camera::monoCallback(const image::Header& header)
{
    if (Source_Luma_Left != header.source &&
        Source_Luma_Right != header.source &&
        Source_Luma_Aux != header.source)
    {
        RCLCPP_ERROR(get_logger(), "Camera: unexpected image source: 0x%lx", header.source);
        return;
    }

    const rclcpp::Time t(header.timeSeconds, 1000 * header.timeMicroSeconds);

    switch(header.source)
    {
        case Source_Luma_Left:
        {
            if (numSubscribers(left_node_, MONO_TOPIC) == 0)
            {
                break;
            }

            left_mono_image_.data.resize(header.imageLength);
            memcpy(&left_mono_image_.data[0], header.imageDataP, header.imageLength);

            left_mono_image_.header.frame_id = frame_id_left_;
            left_mono_image_.header.stamp    = t;
            left_mono_image_.height          = header.height;
            left_mono_image_.width           = header.width;

            switch(header.bitsPerPixel) {
                case 8:
                    left_mono_image_.encoding = sensor_msgs::image_encodings::MONO8;
                    left_mono_image_.step     = header.width;
                    break;
                case 16:
                    left_mono_image_.encoding = sensor_msgs::image_encodings::MONO16;
                    left_mono_image_.step     = header.width * 2;
                    break;
            }

            left_mono_image_.is_bigendian    = (htonl(1) == 1);

            left_mono_cam_pub_->publish(left_mono_image_);

            //
            // Publish a specific camera info message for the left mono image

            left_mono_cam_info_pub_->publish(stereo_calibration_manager_->leftCameraInfo(frame_id_left_, t));

            break;
        }
        case Source_Luma_Right:
        {
            if (numSubscribers(right_node_, MONO_TOPIC) == 0)
            {
                break;
            }

            right_mono_image_.data.resize(header.imageLength);
            memcpy(&right_mono_image_.data[0], header.imageDataP, header.imageLength);

            right_mono_image_.header.frame_id = frame_id_right_;
            right_mono_image_.header.stamp    = t;
            right_mono_image_.height          = header.height;
            right_mono_image_.width           = header.width;

            switch(header.bitsPerPixel)
            {
                case 8:
                    right_mono_image_.encoding = sensor_msgs::image_encodings::MONO8;
                    right_mono_image_.step     = header.width;
                    break;
                case 16:
                    right_mono_image_.encoding = sensor_msgs::image_encodings::MONO16;
                    right_mono_image_.step     = header.width * 2;
                    break;
            }
            right_mono_image_.is_bigendian    = (htonl(1) == 1);

            right_mono_cam_pub_->publish(right_mono_image_);

            //
            // Publish a specific camera info message for the left mono image

            right_mono_cam_info_pub_->publish(stereo_calibration_manager_->rightCameraInfo(frame_id_right_, t));

            break;
        }
        case Source_Luma_Aux:
        {
            if (numSubscribers(aux_node_, MONO_TOPIC) == 0)
            {
                break;
            }

            aux_mono_image_.data.resize(header.imageLength);
            memcpy(&aux_mono_image_.data[0], header.imageDataP, header.imageLength);

            aux_mono_image_.header.frame_id = frame_id_aux_;
            aux_mono_image_.header.stamp    = t;
            aux_mono_image_.height          = header.height;
            aux_mono_image_.width           = header.width;

            switch(header.bitsPerPixel) {
                case 8:
                    aux_mono_image_.encoding = sensor_msgs::image_encodings::MONO8;
                    aux_mono_image_.step     = header.width;
                    break;
                case 16:
                    aux_mono_image_.encoding = sensor_msgs::image_encodings::MONO16;
                    aux_mono_image_.step     = header.width * 2;
                    break;
            }
            aux_mono_image_.is_bigendian    = (htonl(1) == 1);

            aux_mono_cam_pub_->publish(aux_mono_image_);

            //
            // Publish a specific camera info message for the aux mono image

            aux_mono_cam_info_pub_->publish(stereo_calibration_manager_->auxCameraInfo(frame_id_aux_, t));
            break;

        }
        }
}

void Camera::rectCallback(const image::Header& header)
{
    if (Source_Luma_Rectified_Left != header.source &&
        Source_Luma_Rectified_Right != header.source &&
        Source_Luma_Rectified_Aux != header.source) {

        RCLCPP_ERROR(get_logger(), "Camera: unexpected image source: 0x%lx", header.source);
        return;
    }

    const rclcpp::Time t(header.timeSeconds, 1000 * header.timeMicroSeconds);

    switch(header.source)
    {
        case Source_Luma_Rectified_Left:
        {
            if (numSubscribers(left_node_, RECT_TOPIC) == 0)
            {
                break;
            }

            left_rect_image_.data.resize(header.imageLength);
            memcpy(&left_rect_image_.data[0], header.imageDataP, header.imageLength);

            left_rect_image_.header.frame_id = frame_id_rectified_left_;
            left_rect_image_.header.stamp    = t;
            left_rect_image_.height          = header.height;
            left_rect_image_.width           = header.width;


            switch(header.bitsPerPixel) {
                case 8:
                    left_rect_image_.encoding = sensor_msgs::image_encodings::MONO8;
                    left_rect_image_.step     = header.width;

                    break;
                case 16:
                    left_rect_image_.encoding = sensor_msgs::image_encodings::MONO16;
                    left_rect_image_.step     = header.width * 2;

                    break;
            }

            left_rect_image_.is_bigendian    = (htonl(1) == 1);

            const auto left_camera_info = stereo_calibration_manager_->leftCameraInfo(frame_id_rectified_left_, t);

            left_rect_cam_pub_->publish(left_rect_image_);

            left_rect_cam_info_pub_->publish(left_camera_info);

            break;
        }
        case Source_Luma_Rectified_Right:
        {
            if (numSubscribers(right_node_, RECT_TOPIC) == 0)
            {
                break;
            }

            right_rect_image_.data.resize(header.imageLength);
            memcpy(&right_rect_image_.data[0], header.imageDataP, header.imageLength);

            right_rect_image_.header.frame_id = frame_id_rectified_right_;
            right_rect_image_.header.stamp    = t;
            right_rect_image_.height          = header.height;
            right_rect_image_.width           = header.width;

            switch(header.bitsPerPixel)
            {
                case 8:
                {
                    right_rect_image_.encoding = sensor_msgs::image_encodings::MONO8;
                    right_rect_image_.step     = header.width;
                    break;
                }
                case 16:
                {
                    right_rect_image_.encoding = sensor_msgs::image_encodings::MONO16;
                    right_rect_image_.step     = header.width * 2;
                    break;
                }
            }

            right_rect_image_.is_bigendian = (htonl(1) == 1);

            const auto right_camera_info = stereo_calibration_manager_->rightCameraInfo(frame_id_rectified_right_, t);

            right_rect_cam_pub_->publish(right_rect_image_);

            right_rect_cam_info_pub_->publish(right_camera_info);

            break;
        }
        case Source_Luma_Rectified_Aux:
        {
            if (numSubscribers(aux_node_, RECT_TOPIC) == 0)
            {
                break;
            }

            aux_rect_image_.data.resize(header.imageLength);
            memcpy(&aux_rect_image_.data[0], header.imageDataP, header.imageLength);

            aux_rect_image_.header.frame_id = frame_id_rectified_aux_;
            aux_rect_image_.header.stamp    = t;
            aux_rect_image_.height          = header.height;
            aux_rect_image_.width           = header.width;

            switch(header.bitsPerPixel) {
                case 8:
                    aux_rect_image_.encoding = sensor_msgs::image_encodings::MONO8;
                    aux_rect_image_.step     = header.width;
                    break;
                case 16:
                    aux_rect_image_.encoding = sensor_msgs::image_encodings::MONO16;
                    aux_rect_image_.step     = header.width * 2;
                    break;
            }

            aux_rect_image_.is_bigendian = (htonl(1) == 1);

            const auto aux_camera_info = stereo_calibration_manager_->auxCameraInfo(frame_id_rectified_aux_, t);

            aux_rect_cam_pub_->publish(aux_rect_image_);

            aux_rect_cam_info_pub_->publish(aux_camera_info);

            break;
        }
    }
}

void Camera::depthCallback(const image::Header& header)
{
    if (Source_Disparity != header.source)
    {
        RCLCPP_ERROR(get_logger(), "Camera: unexpected image source: 0x%lx", header.source);
        return;
    }

    const size_t ni_depth_subscribers = numSubscribers(left_node_, OPENNI_DEPTH_TOPIC);
    const size_t depth_subscribers = numSubscribers(left_node_, DEPTH_TOPIC);

    if (ni_depth_subscribers == 0 && depth_subscribers == 0)
    {
        return;
    }

    const float    bad_point = std::numeric_limits<float>::quiet_NaN();
    const uint32_t depthSize = header.height * header.width * sizeof(float);
    const uint32_t niDepthSize = header.height * header.width * sizeof(uint16_t);
    const uint32_t imageSize = header.width * header.height;

    const rclcpp::Time t(header.timeSeconds, 1000 * header.timeMicroSeconds);
    depth_image_.header.stamp = t;
    depth_image_.header.frame_id = frame_id_rectified_left_;
    depth_image_.height          = header.height;
    depth_image_.width           = header.width;
    depth_image_.is_bigendian    = (htonl(1) == 1);

    ni_depth_image_ = depth_image_;

    ni_depth_image_.encoding           = sensor_msgs::image_encodings::MONO16;
    ni_depth_image_.step               = header.width * 2;

    depth_image_.encoding        = sensor_msgs::image_encodings::TYPE_32FC1;
    depth_image_.step            = header.width * 4;

    depth_image_.data.resize(depthSize);
    ni_depth_image_.data.resize(niDepthSize);

    float *depthImageP = reinterpret_cast<float*>(&depth_image_.data[0]);
    uint16_t *niDepthImageP = reinterpret_cast<uint16_t*>(&ni_depth_image_.data[0]);

    //
    // Disparity is in 32-bit floating point

    if (32 == header.bitsPerPixel)
    {
        //
        // Depth = focal_length*baseline/disparity
        // From the Q matrix used to reproject disparity images using non-isotropic
        // pixels we see that z = (fx*fy*Tx). Normalizing z so that
        // the scale factor on the homogeneous cartesian coordinate is 1 results
        // in z =  (fx*fy*Tx)/(-fy*d) or z = (fx*Tx)/(-d).
        // The 4th element of the right camera projection matrix is defined
        // as fx*Tx.

        const double scale = stereo_calibration_manager_->rightCameraInfo(frame_id_rectified_right_, t).p[3];

        const float *disparityImageP = reinterpret_cast<const float*>(header.imageDataP);

        for (uint32_t i = 0 ; i < imageSize ; ++i)
        {
            if (0.0 >= disparityImageP[i])
            {
                depthImageP[i] = bad_point;
                niDepthImageP[i] = 0;
            }
            else
            {
                depthImageP[i] = scale / disparityImageP[i];
                niDepthImageP[i] = static_cast<uint16_t>(depthImageP[i] * 1000);
            }
        }
    }
    else if (16 == header.bitsPerPixel)
    {
        //
        // Depth = focal_length*baseline/disparity
        // From the Q matrix used to reproject disparity images using non-isotropic
        // pixels we see that z = (fx*fy*Tx). Normalizing z so that
        // the scale factor on the homogeneous cartesian coordinate is 1 results
        // in z =  (fx*fy*Tx)/(-fy*d) or z = (fx*Tx)/(-d). Because our disparity
        // image is 16 bits we must also divide by 16 making z = (fx*Tx*16)/(-d)
        // The 4th element of the right camera projection matrix is defined
        // as fx*Tx.

        const float scale = stereo_calibration_manager_->rightCameraInfo(frame_id_rectified_right_, t).p[3] * -16.0f;

        const uint16_t *disparityImageP = reinterpret_cast<const uint16_t*>(header.imageDataP);

        for (uint32_t i = 0 ; i < imageSize ; ++i)
        {
            if (0 == disparityImageP[i])
            {
                depthImageP[i] = bad_point;
                niDepthImageP[i] = 0;
            }
            else
            {
                depthImageP[i] = scale / disparityImageP[i];
                niDepthImageP[i] = static_cast<uint16_t>(depthImageP[i] * 1000);
            }
        }

    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Camera: unsupported disparity bpp: %d", header.bitsPerPixel);
        return;
    }

    if (0 != ni_depth_subscribers)
    {
        ni_depth_cam_pub_->publish(ni_depth_image_);
    }

    if (0 != depth_subscribers)
    {
        depth_cam_pub_->publish(depth_image_);
    }

    depth_cam_info_pub_->publish(stereo_calibration_manager_->leftCameraInfo(frame_id_rectified_left_, t));
}

void Camera::pointCloudCallback(const image::Header& header)
{
    if (Source_Disparity != header.source)
    {
        RCLCPP_ERROR(get_logger(), "Camera: unexpected image source: 0x%lx", header.source);
        return;
    }

    //
    // Get the corresponding visual images so we can colorize properly

    std::shared_ptr<BufferWrapper<image::Header>> left_luma_rect = nullptr;
    std::shared_ptr<BufferWrapper<image::Header>> left_luma = nullptr;
    std::shared_ptr<BufferWrapper<image::Header>> left_chroma = nullptr;
    std::shared_ptr<BufferWrapper<image::Header>> aux_luma_rectified = nullptr;
    std::shared_ptr<BufferWrapper<image::Header>> aux_chroma_rectified = nullptr;

    if (const auto image = image_buffers_.find(Source_Luma_Rectified_Left);
            image != std::end(image_buffers_) && image->second->data().frameId == header.frameId)
    {
        left_luma_rect = image->second;
    }

    if (const auto image = image_buffers_.find(Source_Luma_Left);
            image != std::end(image_buffers_) && image->second->data().frameId == header.frameId)
    {
        left_luma = image->second;
    }

    if (const auto image = image_buffers_.find(Source_Chroma_Left);
            image != std::end(image_buffers_) && image->second->data().frameId == header.frameId)
    {
        left_chroma = image->second;
    }

    const auto aux_luma_rectified_image = image_buffers_.find(Source_Luma_Rectified_Aux);
    if (aux_luma_rectified_image != std::end(image_buffers_) && aux_luma_rectified_image->second->data().frameId == header.frameId)
    {
        aux_luma_rectified = aux_luma_rectified_image->second;
    }

    const auto aux_chroma_rectified_image = image_buffers_.find(Source_Chroma_Rectified_Aux);
    if (aux_chroma_rectified_image != std::end(image_buffers_) && aux_chroma_rectified_image->second->data().frameId == header.frameId)
    {
        aux_chroma_rectified = aux_chroma_rectified_image->second;
    }

    const bool color_data = (has_aux_camera_ && aux_luma_rectified && aux_chroma_rectified && stereo_calibration_manager_->validAux()) ||
                            (!has_aux_camera_ && supports_color_ && left_luma && left_chroma);

    //
    // Check if we have all the data to publish and if the user wants us to publish

    const bool pub_pointcloud = numSubscribers(this, POINTCLOUD_TOPIC) > 0 && left_luma_rect;
    const bool pub_color_pointcloud = numSubscribers(this, COLOR_POINTCLOUD_TOPIC) > 0 && color_data;
    const bool pub_organized_pointcloud = numSubscribers(this, ORGANIZED_POINTCLOUD_TOPIC) > 0 && left_luma_rect;
    const bool pub_color_organized_pointcloud = numSubscribers(this, COLOR_ORGANIZED_POINTCLOUD_TOPIC) > 0 && color_data;

    if (!(pub_pointcloud || pub_color_pointcloud || pub_organized_pointcloud || pub_color_organized_pointcloud))
    {
        return;
    }

    const rclcpp::Time t(header.timeSeconds, 1000 * header.timeMicroSeconds);

    //
    // Resize our corresponding pointclouds if we plan on publishing them

    if (pub_pointcloud)
    {
        luma_point_cloud_.header.stamp = t;
        luma_point_cloud_.data.resize(header.width * header.height * luma_point_cloud_.point_step);
    }

    if (pub_color_pointcloud)
    {
        color_point_cloud_.header.stamp = t;
        color_point_cloud_.data.resize(header.width * header.height * color_point_cloud_.point_step);
    }

    if (pub_organized_pointcloud)
    {
        luma_organized_point_cloud_.header.stamp = t;
        luma_organized_point_cloud_.data.resize(header.width * header.height * luma_organized_point_cloud_.point_step);
        luma_organized_point_cloud_.width = header.width;
        luma_organized_point_cloud_.height = header.height;
        luma_organized_point_cloud_.row_step = header.width * luma_organized_point_cloud_.point_step;
    }

    if (pub_color_organized_pointcloud)
    {
        color_organized_point_cloud_.header.stamp = t;
        color_organized_point_cloud_.data.resize(header.width * header.height * color_organized_point_cloud_.point_step);
        color_organized_point_cloud_.width = header.width;
        color_organized_point_cloud_.height = header.height;
        color_organized_point_cloud_.row_step = header.width * color_organized_point_cloud_.point_step;
    }

    const Eigen::Matrix4d Q = stereo_calibration_manager_->Q();

    const Eigen::Vector3f invalid_point(std::numeric_limits<float>::quiet_NaN(),
                                        std::numeric_limits<float>::quiet_NaN(),
                                        std::numeric_limits<float>::quiet_NaN());

    //
    // Create rectified color image upfront if we are planning to publish color pointclouds

    std::optional<cv::Mat> rectified_color = std::nullopt;
    if (!has_aux_camera_ && supports_color_ && (pub_color_pointcloud || pub_color_organized_pointcloud))
    {
        const auto &luma = left_luma->data();

        pointcloud_color_buffer_.resize(3 * luma.width * luma.height);
        pointcloud_rect_color_buffer_.resize(3 * luma.width * luma.height);
        ycbcrToBgr(luma, left_chroma->data(), &(pointcloud_color_buffer_[0]));

        cv::Mat rgb_image(luma.height, luma.width, CV_8UC3, &(pointcloud_color_buffer_[0]));
        cv::Mat rect_rgb_image(luma.height, luma.width, CV_8UC3, &(pointcloud_rect_color_buffer_[0]));

        const auto left_remap = stereo_calibration_manager_->leftRemap();

        cv::remap(rgb_image, rect_rgb_image, left_remap->map1, left_remap->map2, cv::INTER_LINEAR);

        rectified_color = std::make_optional(std::move(rect_rgb_image));
    }
    else if(has_aux_camera_ && (pub_color_pointcloud || pub_color_organized_pointcloud))
    {
        const auto &luma = aux_luma_rectified->data();

        pointcloud_rect_color_buffer_.resize(3 * luma.width * luma.height);

        ycbcrToBgr(luma, aux_chroma_rectified->data(), reinterpret_cast<uint8_t*>(&(pointcloud_rect_color_buffer_[0])));

        cv::Mat rect_rgb_image(luma.height, luma.width, CV_8UC3, &(pointcloud_rect_color_buffer_[0]));

        rectified_color = std::move(rect_rgb_image);
    }

    //
    // Iterate through our disparity image once populating our pointcloud structures if we plan to publish them

    uint32_t packed_color = 0;

    const double squared_max_range = pointcloud_max_range_ * pointcloud_max_range_;

    const double aux_T = has_aux_camera_ ? stereo_calibration_manager_->aux_T() : stereo_calibration_manager_->T();
    const double T = stereo_calibration_manager_->T();

    size_t valid_points = 0;
    for (size_t y = 0 ; y < header.height ; ++y)
    {
        for (size_t x = 0 ; x < header.width ; ++x)
        {
            const size_t index = y * header.width + x;

            double disparity = 0.0f;
            switch(header.bitsPerPixel)
            {
                case 16:
                {
                    disparity = static_cast<double>(reinterpret_cast<const uint16_t*>(header.imageDataP)[index]) / 16.0f;
                    break;
                }
                case 32:
                {
                    disparity = static_cast<double>(reinterpret_cast<const float*>(header.imageDataP)[index]);
                    break;
                }
                default:
                {
                    RCLCPP_ERROR(get_logger(), "Camera: unsupported disparity depth: %d", header.bitsPerPixel);
                    return;
                }
            }

            //
            // We have a valid rectified color image meaning we plan to publish color pointcloud topics. Assembe the
            // color pixel here since it may be needed in our organized pointclouds

            if (rectified_color)
            {
                packed_color = 0;

                const double color_d = has_aux_camera_ ? (disparity * aux_T) / T : 0.0;

                const auto color_pixel = has_aux_camera_ ? u_interpolate_color(std::max(x - color_d, 0.), y, rectified_color.value()) :
                                                          rectified_color->at<cv::Vec3b>(y, x);

                packed_color |= color_pixel[2] << 16 | color_pixel[1] << 8 | color_pixel[0];
            }

            //
            // If our disparity is 0 pixels our corresponding 3D point is infinite. If we plan to publish organized
            // pointclouds we will need to add a invalid point to our pointcloud(s)

            if (disparity == 0.0f || clipPoint(border_clip_type_, border_clip_value_, header.width, header.height, x, y))
            {
                if (pub_organized_pointcloud)
                {
                    writePoint(luma_organized_point_cloud_, index, invalid_point, index, left_luma_rect->data());
                }

                if (pub_color_organized_pointcloud)
                {
                    writePoint(color_organized_point_cloud_, index, invalid_point, packed_color);
                }

                continue;
            }

            const Eigen::Vector3f point = ((Q * Eigen::Vector4d(static_cast<double>(x),
                                                                static_cast<double>(y),
                                                                disparity,
                                                                1.0)).hnormalized()).cast<float>();


            const bool valid = isValidReprojectedPoint(point, squared_max_range);

            if (pub_pointcloud && valid)
            {
                writePoint(luma_point_cloud_, valid_points, point, index, left_luma_rect->data());
            }

            if(pub_color_pointcloud && valid)
            {
                writePoint(color_point_cloud_, valid_points, point, packed_color);
            }

            if (pub_organized_pointcloud)
            {
                writePoint(luma_organized_point_cloud_, index, valid ? point : invalid_point, index, left_luma_rect->data());
            }

            if (pub_color_organized_pointcloud)
            {
                writePoint(color_organized_point_cloud_, index, valid ? point : invalid_point, packed_color);
            }

            if (valid)
            {
                ++valid_points;
            }
        }
    }

    if (pub_pointcloud)
    {
        luma_point_cloud_.height = 1;
        luma_point_cloud_.row_step = valid_points * luma_point_cloud_.point_step;
        luma_point_cloud_.width = valid_points;
        luma_point_cloud_.data.resize(valid_points * luma_point_cloud_.point_step);
        luma_point_cloud_pub_->publish(luma_point_cloud_);
    }

    if(pub_color_pointcloud)
    {
        color_point_cloud_.height = 1;
        color_point_cloud_.row_step = valid_points * color_point_cloud_.point_step;
        color_point_cloud_.width = valid_points;
        color_point_cloud_.data.resize(valid_points * color_point_cloud_.point_step);
        color_point_cloud_pub_->publish(color_point_cloud_);
    }

    if (pub_organized_pointcloud)
    {
        luma_organized_point_cloud_pub_->publish(luma_organized_point_cloud_);
    }

    if (pub_color_organized_pointcloud)
    {
        color_organized_point_cloud_pub_->publish(color_organized_point_cloud_);
    }
}

void Camera::rawCamDataCallback(const image::Header& header)
{
    if (numSubscribers(calibration_node_, RAW_CAM_DATA_TOPIC) == 0)
    {
        return;
    }

    if(Source_Disparity == header.source)
    {
        if (const auto image = image_buffers_.find(Source_Luma_Rectified_Left);
                image != std::end(image_buffers_) && image->second->data().frameId == header.frameId)
        {
            const auto luma_ptr = image->second;
            const auto &left_luma_rect = luma_ptr->data();

            const uint32_t left_luma_image_size = left_luma_rect.width * left_luma_rect.height;

            raw_cam_data_.gray_scale_image.resize(left_luma_image_size);
            memcpy(&(raw_cam_data_.gray_scale_image[0]),
                   left_luma_rect.imageDataP,
                   left_luma_image_size * sizeof(uint8_t));

            raw_cam_data_.frames_per_second = left_luma_rect.framesPerSecond;
            raw_cam_data_.gain              = left_luma_rect.gain;
            raw_cam_data_.exposure_time     = left_luma_rect.exposure;
            raw_cam_data_.frame_count       = left_luma_rect.frameId;
            raw_cam_data_.time_stamp = rclcpp::Time(left_luma_rect.timeSeconds, 1000 * left_luma_rect.timeMicroSeconds);
            raw_cam_data_.width             = left_luma_rect.width;
            raw_cam_data_.height            = left_luma_rect.height;

            const uint32_t disparity_size = header.width * header.height;

            raw_cam_data_.disparity_image.resize(disparity_size);
            memcpy(&(raw_cam_data_.disparity_image[0]),
                   header.imageDataP,
                   disparity_size * header.bitsPerPixel == 16 ? sizeof(uint16_t) : sizeof(uint32_t));

            raw_cam_data_pub_->publish(raw_cam_data_);
        }
    }
}

void Camera::colorImageCallback(const image::Header& header)
{
    //
    // The left-luma image is currently published before the matching chroma image so this can just trigger on that

    if (Source_Chroma_Left != header.source &&
        Source_Chroma_Rectified_Aux != header.source &&
        Source_Chroma_Aux != header.source)
    {
        RCLCPP_ERROR(get_logger(), "Camera: unexpected image source: 0x%lx", header.source);
        return;
    }

    const rclcpp::Time t(header.timeSeconds, 1000 * header.timeMicroSeconds);

    switch (header.source)
    {
    case Source_Chroma_Left:
    {
        const auto color_subscribers = numSubscribers(left_node_, COLOR_TOPIC);
        const auto color_rect_subscribers = numSubscribers(left_node_, RECT_COLOR_TOPIC);

        if (color_subscribers == 0 && color_rect_subscribers == 0)
        {
            return;
        }

        const auto left_luma = image_buffers_.find(Source_Luma_Left);
        if (left_luma == std::end(image_buffers_))
        {
            return;
        }

        const auto luma_ptr = left_luma->second;

        if (header.frameId == luma_ptr->data().frameId)
        {
            const size_t height    = luma_ptr->data().height;
            const size_t width     = luma_ptr->data().width;
            const size_t image_size = 3 * height * width;

            left_rgb_image_.data.resize(image_size);

            left_rgb_image_.header.frame_id = frame_id_left_;
            left_rgb_image_.header.stamp = t;
            left_rgb_image_.height = height;
            left_rgb_image_.width  = width;

            left_rgb_image_.encoding        = sensor_msgs::image_encodings::BGR8;
            left_rgb_image_.is_bigendian    = (htonl(1) == 1);
            left_rgb_image_.step            = 3 * width;

            //
            // Convert YCbCr 4:2:0 to RGB

            ycbcrToBgr(luma_ptr->data(), header, reinterpret_cast<uint8_t*>(&(left_rgb_image_.data[0])));

            if (color_subscribers != 0)
            {
                left_rgb_cam_pub_->publish(left_rgb_image_);

                left_rgb_cam_info_pub_->publish(stereo_calibration_manager_->leftCameraInfo(frame_id_left_, t));
            }

            if (color_rect_subscribers > 0)
            {
                left_rgb_rect_image_.data.resize(image_size);

                const auto remaps = stereo_calibration_manager_->leftRemap();

                const cv::Mat rgb_image(height, width, CV_8UC3, &(left_rgb_image_.data[0]));
                cv::Mat rect_rgb_image(height, width, CV_8UC3, &(left_rgb_rect_image_.data[0]));

                cv::remap(rgb_image, rect_rgb_image, remaps->map1, remaps->map2, cv::INTER_LINEAR);

                left_rgb_rect_image_.header.frame_id = frame_id_rectified_left_;
                left_rgb_rect_image_.header.stamp    = t;
                left_rgb_rect_image_.height          = height;
                left_rgb_rect_image_.width           = width;

                left_rgb_rect_image_.encoding        = sensor_msgs::image_encodings::BGR8;
                left_rgb_rect_image_.is_bigendian    = (htonl(1) == 1);
                left_rgb_rect_image_.step            = 3 * width;

                const auto left_camera_info = stereo_calibration_manager_->leftCameraInfo(frame_id_rectified_left_, t);

                left_rgb_rect_cam_pub_->publish(left_rgb_rect_image_);

                left_rgb_rect_cam_info_pub_->publish(left_camera_info);
            }
        }

        break;
    }
    case Source_Chroma_Rectified_Aux:
    {
        if(numSubscribers(aux_node_, RECT_COLOR_TOPIC) == 0)
        {
            return;
        }

        const auto aux_luma = image_buffers_.find(Source_Luma_Rectified_Aux);
        if (aux_luma == std::end(image_buffers_))
        {
            return;
        }

        const auto luma_ptr = aux_luma->second;

        if (header.frameId == luma_ptr->data().frameId) {

            const uint32_t height    = luma_ptr->data().height;
            const uint32_t width     = luma_ptr->data().width;
            const uint32_t imageSize = 3 * height * width;

            aux_rect_rgb_image_.data.resize(imageSize);

            aux_rect_rgb_image_.header.frame_id = frame_id_rectified_aux_;
            aux_rect_rgb_image_.header.stamp    = t;
            aux_rect_rgb_image_.height          = height;
            aux_rect_rgb_image_.width           = width;

            aux_rect_rgb_image_.encoding        = sensor_msgs::image_encodings::BGR8;
            aux_rect_rgb_image_.is_bigendian    = (htonl(1) == 1);
            aux_rect_rgb_image_.step            = 3 * width;

            //
            // Convert YCbCr 4:2:0 to RGB

            ycbcrToBgr(luma_ptr->data(), header, reinterpret_cast<uint8_t*>(&(aux_rect_rgb_image_.data[0])));

            const auto aux_camera_info = stereo_calibration_manager_->auxCameraInfo(frame_id_rectified_aux_, t);

            aux_rgb_rect_cam_pub_->publish(aux_rect_rgb_image_);

            aux_rgb_rect_cam_info_pub_->publish(aux_camera_info);
        }

        break;
    }
    case Source_Chroma_Aux:
    {
        if(numSubscribers(aux_node_, COLOR_TOPIC) == 0)
        {
            return;
        }

        const auto aux_luma = image_buffers_.find(Source_Luma_Aux);
        if (aux_luma == std::end(image_buffers_)) {
            return;
        }

        const auto luma_ptr = aux_luma->second;

        if (header.frameId == luma_ptr->data().frameId) {
            const uint32_t height    = luma_ptr->data().height;
            const uint32_t width     = luma_ptr->data().width;
            const uint32_t imageSize = 3 * height * width;

            aux_rgb_image_.data.resize(imageSize);

            aux_rgb_image_.header.frame_id = frame_id_aux_;
            aux_rgb_image_.header.stamp    = t;
            aux_rgb_image_.height          = height;
            aux_rgb_image_.width           = width;

            aux_rgb_image_.encoding        = sensor_msgs::image_encodings::BGR8;
            aux_rgb_image_.is_bigendian    = (htonl(1) == 1);
            aux_rgb_image_.step            = 3 * width;

            //
            // Convert YCbCr 4:2:0 to RGB

            ycbcrToBgr(luma_ptr->data(), header, reinterpret_cast<uint8_t*>(&(aux_rgb_image_.data[0])));

            const auto aux_camera_info = stereo_calibration_manager_->auxCameraInfo(frame_id_aux_, t);

            aux_rgb_cam_pub_->publish(aux_rgb_image_);

            aux_rgb_cam_info_pub_->publish(aux_camera_info);

            break;
        }
    }
    }
}

void Camera::colorizeCallback(const image::Header& header)
{
    if (Source_Luma_Rectified_Aux != header.source &&
        Source_Chroma_Rectified_Aux != header.source &&
        Source_Luma_Aux != header.source &&
        Source_Luma_Left != header.source &&
        Source_Chroma_Left != header.source &&
        Source_Luma_Rectified_Left != header.source)
    {
        RCLCPP_ERROR(get_logger(), "Camera: unexpected image source: 0x%lx", header.source);
        return;
    }

    image_buffers_[header.source] = std::make_shared<BufferWrapper<crl::multisense::image::Header>>(driver_, header);
}

void Camera::updateConfig(const image::Config& config)
{
    stereo_calibration_manager_->updateConfig(config);

    //
    // Publish the "raw" config message

    multisense_msgs::msg::RawCamConfig cfg;

    cfg.width             = config.width();
    cfg.height            = config.height();
    cfg.frames_per_second = config.fps();
    cfg.gain              = config.gain();
    cfg.exposure_time     = config.exposure();

    cfg.fx    = config.fx();
    cfg.fy    = config.fy();
    cfg.cx    = config.cx();
    cfg.cy    = config.cy();
    cfg.tx    = config.tx();
    cfg.ty    = config.ty();
    cfg.tz    = config.tz();
    cfg.roll  = config.roll();
    cfg.pitch = config.pitch();
    cfg.yaw   = config.yaw();

    raw_cam_config_pub_->publish(cfg);

    //
    // Republish our camera info topics since the resolution changed

    publishAllCameraInfo();
}

void Camera::publishAllCameraInfo()
{
    const auto stamp = rclcpp::Clock().now();

    const auto left_camera_info = stereo_calibration_manager_->leftCameraInfo(frame_id_left_, stamp);
    const auto right_camera_info = stereo_calibration_manager_->rightCameraInfo(frame_id_right_, stamp);

    const auto left_rectified_camera_info = stereo_calibration_manager_->leftCameraInfo(frame_id_rectified_left_, stamp);
    const auto right_rectified_camera_info = stereo_calibration_manager_->rightCameraInfo(frame_id_rectified_right_, stamp);

    //
    // Republish camera info messages outside of image callbacks.
    // The camera info publishers are latching so the messages
    // will persist until a new message is published in one of the image
    // callbacks. This makes it easier when a user is trying access a camera_info
    // for a topic which they are not subscribed to

    if (supports_color_)
    {
        if (has_aux_camera_) {

            aux_mono_cam_info_pub_->publish(stereo_calibration_manager_->auxCameraInfo(frame_id_aux_, stamp));
            aux_rect_cam_info_pub_->publish(stereo_calibration_manager_->auxCameraInfo(frame_id_rectified_aux_, stamp));

            aux_rgb_cam_info_pub_->publish(stereo_calibration_manager_->auxCameraInfo(frame_id_aux_, stamp));
            aux_rgb_rect_cam_info_pub_->publish(stereo_calibration_manager_->auxCameraInfo(frame_id_rectified_aux_, stamp));
        }
        else
        {
            left_rgb_cam_info_pub_->publish(left_camera_info);
            left_rgb_rect_cam_info_pub_->publish(left_rectified_camera_info);
        }
    }

    right_disp_cam_info_pub_->publish(right_camera_info);
    left_cost_cam_info_pub_->publish(left_camera_info);

    left_mono_cam_info_pub_->publish(left_camera_info);
    left_rect_cam_info_pub_->publish(left_rectified_camera_info);
    right_mono_cam_info_pub_->publish(right_camera_info);
    right_rect_cam_info_pub_->publish(right_rectified_camera_info);
    left_disp_cam_info_pub_->publish(left_rectified_camera_info);
    depth_cam_info_pub_->publish(left_rectified_camera_info);

}

void Camera::stop()
{
    if (const auto status = driver_->stopStreams(allImageSources); status != Status_Ok)
    {
        RCLCPP_ERROR(get_logger(), "Camera: failed to stop all streams: %s",
                     Channel::statusString(status));
    }
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

void Camera::timerCallback()
{
    DataSource enable = Source_Unknown;

    enable |= numSubscribers(left_node_, MONO_TOPIC) > 0 ?  Source_Luma_Left : enable;
    enable |= numSubscribers(right_node_, MONO_TOPIC) > 0 ?  Source_Luma_Right : enable;
    enable |= numSubscribers(left_node_, RECT_TOPIC) > 0 ?  Source_Luma_Rectified_Left : enable;
    enable |= numSubscribers(right_node_, RECT_TOPIC) > 0 ?  Source_Luma_Rectified_Right : enable;
    enable |= numSubscribers(left_node_, DEPTH_TOPIC) > 0 ?  Source_Disparity : enable;
    enable |= numSubscribers(left_node_, OPENNI_DEPTH_TOPIC) > 0 ?  Source_Disparity : enable;

    if (supports_color_)
    {
        if (has_aux_camera_)
        {
            enable |= numSubscribers(aux_node_, MONO_TOPIC) > 0 ?  Source_Luma_Aux : enable;
            enable |= numSubscribers(aux_node_, COLOR_TOPIC) > 0 ?  Source_Luma_Aux | Source_Chroma_Aux: enable;
            enable |= numSubscribers(aux_node_, RECT_TOPIC) > 0 ?  Source_Luma_Rectified_Aux : enable;
            enable |= numSubscribers(aux_node_, RECT_COLOR_TOPIC) > 0 ?  Source_Luma_Rectified_Aux | Source_Chroma_Rectified_Aux : enable;
        }
        else
        {
            enable |= numSubscribers(left_node_, COLOR_TOPIC) > 0 ?  Source_Luma_Left | Source_Chroma_Left : enable;
            enable |= numSubscribers(left_node_, COLOR_TOPIC) > 0 ?  Source_Luma_Left | Source_Chroma_Left : enable;
        }

        const auto point_cloud_color_topics = has_aux_camera_ ? Source_Luma_Rectified_Aux | Source_Chroma_Rectified_Aux :
                                                                Source_Luma_Left | Source_Chroma_Left;

        enable |= numSubscribers(this, COLOR_POINTCLOUD_TOPIC) > 0 ?  Source_Disparity | point_cloud_color_topics : enable;
        enable |= numSubscribers(this, COLOR_ORGANIZED_POINTCLOUD_TOPIC) > 0 ?  Source_Disparity | point_cloud_color_topics : enable;
    }

    enable |= numSubscribers(this, POINTCLOUD_TOPIC) > 0 ?  Source_Luma_Rectified_Left | Source_Disparity : enable;
    enable |= numSubscribers(this, ORGANIZED_POINTCLOUD_TOPIC) > 0 ?  Source_Luma_Rectified_Left | Source_Disparity : enable;
    enable |= numSubscribers(calibration_node_, RAW_CAM_DATA_TOPIC) > 0 ?  Source_Luma_Rectified_Left | Source_Disparity : enable;
    enable |= numSubscribers(left_node_, DISPARITY_TOPIC) > 0 ?  Source_Disparity : enable;
    enable |= numSubscribers(left_node_, DISPARITY_IMAGE_TOPIC) > 0 ?  Source_Disparity : enable;

    enable |= numSubscribers(right_node_, DISPARITY_TOPIC) > 0 ?  Source_Disparity_Right : enable;
    enable |= numSubscribers(right_node_, DISPARITY_IMAGE_TOPIC) > 0 ?  Source_Disparity_Right : enable;
    enable |= numSubscribers(left_node_, COST_TOPIC) > 0 ?  Source_Disparity_Cost : enable;

    //
    // We need to start or stop a stream

    if (enable ^ active_streams_)
    {
        DataSource start = Source_Unknown;
        DataSource stop = Source_Unknown;

        for(uint32_t i=0; i<32; ++i)
        {
            //
            // There are two cases:
            // -We want to enable the stream but it is not enabled in our active streams
            // -We want to disable the stream but it is enabled in our active streams

            if ((1<<i) & enable && !((1<<i) & active_streams_))
            {
                start |= (1<<i);
            }
            else if (!((1<<i) & enable) && (1<<i) & active_streams_)
            {
                stop |= (1<<i);
            }
        }

        if (start != Source_Unknown)
        {
            if (const auto status = driver_->startStreams(start); status != Status_Ok)
            {
                RCLCPP_ERROR(get_logger(), "Camera: failed to start streams 0x%lx: %s",
                             start, Channel::statusString(status));
                return;
            }
        }

        if(stop != Source_Unknown)
        {
            if (const auto status = driver_->stopStreams(stop); status != Status_Ok)
            {
                RCLCPP_ERROR(get_logger(), "Camera: failed to stop streams 0x%lx: %s\n",
                             stop, Channel::statusString(status));
                return;
            }
        }

        //
        // Our stream updates succeeded. We can update our cached active streams

        active_streams_ = enable;
    }
}

void Camera::initalizeParameters(const image::Config& config)
{
    //
    // Sensor resolution

    std::string valid_resolutions = "valid_resolutions [width, height, max disparity value]\n";
    for (const auto& device_mode : device_modes_)
    {
        valid_resolutions += "[";
        valid_resolutions += std::to_string(device_mode.width) + ", ";
        valid_resolutions += std::to_string(device_mode.height) + ", ";
        valid_resolutions += std::to_string(device_mode.disparities) + "]\n";
    }


    rcl_interfaces::msg::ParameterDescriptor sensor_resolution_desc;
    sensor_resolution_desc.set__name("sensor_resolution")
                          .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY)
                          .set__description(valid_resolutions);
    declare_parameter("sensor_resolution",
                      std::vector<int64_t>{config.width(), config.height(), config.disparities()}, sensor_resolution_desc);
    //
    // Fps

    float max_fps = 30.0f;

    switch(device_info_.hardwareRevision)
    {
        case system::DeviceInfo::HARDWARE_REV_MULTISENSE_ST21:
        {
            max_fps = 29.97f;

            break;
        }
        default:
        {
            switch (device_info_.imagerType)
            {
                case system::DeviceInfo::IMAGER_TYPE_CMV2000_GREY:
                case system::DeviceInfo::IMAGER_TYPE_CMV2000_COLOR:
                {
                    max_fps = 30.0f;

                    break;
                }
                case system::DeviceInfo::IMAGER_TYPE_CMV4000_GREY:
                case system::DeviceInfo::IMAGER_TYPE_CMV4000_COLOR:
                {
                    max_fps = 15.0f;

                    break;
                }
            }
            break;
        }
    };

    rcl_interfaces::msg::FloatingPointRange fps_range;
    fps_range.set__from_value(1.0)
              .set__to_value(max_fps);

    rcl_interfaces::msg::ParameterDescriptor fps_desc;
    fps_desc.set__name("fps")
            .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
            .set__description("camera fps")
            .set__floating_point_range({fps_range});
    declare_parameter("fps", 10.0, fps_desc);

    //
    // Stereo Post Filtering

    rcl_interfaces::msg::FloatingPointRange stereo_post_filter_range;
    stereo_post_filter_range.set__from_value(0.0)
                            .set__to_value(1.0);

    rcl_interfaces::msg::ParameterDescriptor stereo_post_filter_desc;
    stereo_post_filter_desc.set__name("stereo_post_filtering")
                           .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                           .set__description("SGM stereo post filter")
                           .set__floating_point_range({stereo_post_filter_range});
    declare_parameter("stereo_post_filtering", 0.75, stereo_post_filter_desc);

    if (device_info_.hardwareRevision != system::DeviceInfo::HARDWARE_REV_MULTISENSE_ST21)
    {

        //
        // Gain
        if (next_gen_camera_)
        {
            rcl_interfaces::msg::FloatingPointRange gain_range;
            gain_range.set__from_value(1.68421)
                      .set__to_value(16.0);

            rcl_interfaces::msg::ParameterDescriptor gain_desc;
            gain_desc.set__name("gain")
                     .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                     .set__description("imager gain")
                     .set__floating_point_range({gain_range});
            declare_parameter("gain", 1.68421, gain_desc);
        }
        else
        {
            rcl_interfaces::msg::FloatingPointRange gain_range;
            gain_range.set__from_value(1.0)
                      .set__to_value(16.0);

            rcl_interfaces::msg::ParameterDescriptor gain_desc;
            gain_desc.set__name("gain")
                     .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                     .set__description("imager gain")
                     .set__floating_point_range({gain_range});
            declare_parameter("gain", 1.0, gain_desc);
        }


        //
        // Auto exposure enable

        rcl_interfaces::msg::ParameterDescriptor auto_exposure_enable_desc;
        auto_exposure_enable_desc.set__name("auto_exposure")
                                 .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
                                 .set__description("enable auto exposure");
        declare_parameter("auto_exposure", true, auto_exposure_enable_desc);

        //
        // Auto exposure max time

        rcl_interfaces::msg::FloatingPointRange auto_exposure_max_time_range;
        auto_exposure_max_time_range.set__from_value(0.0)
                                    .set__to_value(0.033)
                                    .set__step(0.001);

        rcl_interfaces::msg::ParameterDescriptor auto_exposure_max_time_desc;
        auto_exposure_max_time_desc.set__name("auto_exposure_max_time")
                                   .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                                   .set__description("max exposure time using auto exposure")
                                   .set__floating_point_range({auto_exposure_max_time_range});
        declare_parameter("auto_exposure_max_time", 0.01, auto_exposure_max_time_desc);

        //
        // Auto exposure decay

        rcl_interfaces::msg::IntegerRange auto_exposure_decay_range;
        auto_exposure_decay_range.set__from_value(1)
                                 .set__to_value(10);

        rcl_interfaces::msg::ParameterDescriptor auto_exposure_decay_desc;
        auto_exposure_decay_desc.set__name("auto_exposure_decay")
                                .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                                .set__description("auto exposure time decay")
                                .set__integer_range({auto_exposure_decay_range});
        declare_parameter("auto_exposure_decay", 7, auto_exposure_decay_desc);

        //
        // Auto exposure threshold

        rcl_interfaces::msg::FloatingPointRange auto_exposure_thresh_range;
        auto_exposure_thresh_range.set__from_value(0.0)
                                  .set__to_value(1.0);

        rcl_interfaces::msg::ParameterDescriptor auto_exposure_thresh_desc;
        auto_exposure_thresh_desc.set__name("auto_exposure_thresh")
                                .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                                .set__description("auto exposure threshold")
                                .set__floating_point_range({auto_exposure_thresh_range});

        declare_parameter("auto_exposure_thresh", (next_gen_camera_ ? 0.85 : 0.75), auto_exposure_thresh_desc);

        //
        // Auto exposure target intensity

        rcl_interfaces::msg::FloatingPointRange auto_exposure_target_intensity_range;
        auto_exposure_target_intensity_range.set__from_value(0.0)
                                            .set__to_value(1.0)
                                            .set__step(0.01);

        rcl_interfaces::msg::ParameterDescriptor auto_exposure_target_intensity_desc;
        auto_exposure_target_intensity_desc.set__name("auto_exposure_target_intensity")
                                           .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                                           .set__description("auto exposure target intensity")
                                           .set__floating_point_range({auto_exposure_target_intensity_range});
        declare_parameter("auto_exposure_target_intensity", 0.5, auto_exposure_target_intensity_desc);

        //
        // Exposure time

        rcl_interfaces::msg::FloatingPointRange exposure_time_range;
        exposure_time_range.set__from_value(0.0)
                           .set__to_value(0.033);

        rcl_interfaces::msg::ParameterDescriptor exposure_time_desc;
        exposure_time_desc.set__name("exposure_time")
                          .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                          .set__description("imager exposure time in seconds")
                          .set__floating_point_range({exposure_time_range});
        declare_parameter("exposure_time", 0.025, exposure_time_desc);

        if (!has_aux_camera_ &&  supports_color_)
        {
            //
            // Auto white balance enable

            rcl_interfaces::msg::ParameterDescriptor auto_white_balance_enable_desc;
            auto_white_balance_enable_desc.set__name("auto_white_balance")
                     .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
                     .set__description("enable auto white balance");
            declare_parameter("auto_white_balance", true, auto_white_balance_enable_desc);

            //
            // Auto white balance decay

            rcl_interfaces::msg::IntegerRange auto_white_balance_decay_range;
            auto_white_balance_decay_range.set__from_value(0)
                                          .set__to_value(20);

            rcl_interfaces::msg::ParameterDescriptor auto_white_balance_decay_desc;
            auto_white_balance_decay_desc.set__name("auto_white_balance_decay")
                                         .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                                         .set__description("auto white balance decay")
                                         .set__integer_range({auto_white_balance_decay_range});
            declare_parameter("auto_white_balance_decay", 3, auto_white_balance_decay_desc);

            //
            // Auto white balance thresh

            rcl_interfaces::msg::FloatingPointRange auto_white_balance_thresh_range;
            auto_white_balance_thresh_range.set__from_value(0.0)
                                           .set__to_value(1.0);

            rcl_interfaces::msg::ParameterDescriptor auto_white_balance_thresh_desc;
            auto_white_balance_thresh_desc.set__name("auto_white_balance_thresh")
                                          .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                                          .set__description("auto white balance threshold")
                                          .set__floating_point_range({auto_white_balance_thresh_range});
            declare_parameter("auto_white_balance_thresh", 0.5, auto_white_balance_thresh_desc);

            //
            // Auto white balance red

            rcl_interfaces::msg::FloatingPointRange auto_white_balance_red_range;
            auto_white_balance_red_range.set__from_value(0.2)
                                        .set__to_value(4.0);

            rcl_interfaces::msg::ParameterDescriptor auto_white_balance_red_desc;
            auto_white_balance_red_desc.set__name("auto_white_balance_red")
                                       .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                                       .set__description("auto white balance red gain")
                                       .set__floating_point_range({auto_white_balance_red_range});
            declare_parameter("auto_white_balance_red", 1.0, auto_white_balance_red_desc);

            //
            // Auto white balance blue

            rcl_interfaces::msg::FloatingPointRange auto_white_balance_blue_range;
            auto_white_balance_blue_range.set__from_value(0.2)
                                          .set__to_value(4.0);

            rcl_interfaces::msg::ParameterDescriptor auto_white_balance_blue_desc;
            auto_white_balance_blue_desc.set__name("auto_white_balance_blue")
                                          .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                                          .set__description("auto white balance blue gain")
                                          .set__floating_point_range({auto_white_balance_blue_range});
            declare_parameter("auto_white_balance_blue", 1.0, auto_white_balance_blue_desc);
        }

        //
        // HDR enable

        rcl_interfaces::msg::ParameterDescriptor hdr_enable_desc;
        hdr_enable_desc.set__name("hdr_enable")
                       .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
                       .set__description("enable hdr");
        declare_parameter("hdr_enable", false, hdr_enable_desc);

        if (next_gen_camera_)
        {
            //
            // Gamma
            //
            rcl_interfaces::msg::FloatingPointRange gamma_range;
            gamma_range.set__from_value(1.0)
                       .set__to_value(2.2)
                       .set__step(0.01);

            rcl_interfaces::msg::ParameterDescriptor gamma_desc;
            gamma_desc.set__name("gamma")
                       .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                       .set__description("Gamma")
                       .set__floating_point_range({gamma_range});
            declare_parameter("gamma", 2.2, gamma_desc);
        }

        //
        // Enable ROI auto exposure control

        rcl_interfaces::msg::ParameterDescriptor auto_exposure_roi_enable_desc;
        auto_exposure_roi_enable_desc.set__name("auto_exposure_roi_enable")
                                   .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
                                   .set__description("enable auto exposure ROI");
        declare_parameter("auto_exposure_roi_enable", false, auto_exposure_roi_enable_desc);

        //
        // Auto exposure ROI x
        //
        rcl_interfaces::msg::IntegerRange auto_exposure_roi_x_range;
        auto_exposure_roi_x_range.set__from_value(0)
                       .set__to_value(device_info_.imagerWidth)
                       .set__step(1);

        rcl_interfaces::msg::ParameterDescriptor auto_exposure_roi_x_desc;
        auto_exposure_roi_x_desc.set__name("auto_exposure_roi_x")
                                 .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                                 .set__description("auto exposure ROI x value")
                                 .set__integer_range({auto_exposure_roi_x_range});
        declare_parameter("auto_exposure_roi_x", 0, auto_exposure_roi_x_desc);

        //
        // Auto exposure ROI y
        //
        rcl_interfaces::msg::IntegerRange auto_exposure_roi_y_range;
        auto_exposure_roi_y_range.set__from_value(0)
                       .set__to_value(device_info_.imagerHeight)
                       .set__step(1);

        rcl_interfaces::msg::ParameterDescriptor auto_exposure_roi_y_desc;
        auto_exposure_roi_y_desc.set__name("auto_exposure_roi_y")
                                 .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                                 .set__description("auto exposure ROI y value")
                                 .set__integer_range({auto_exposure_roi_y_range});
        declare_parameter("auto_exposure_roi_y", 0, auto_exposure_roi_y_desc);

        //
        // Auto exposure ROI width
        //
        rcl_interfaces::msg::IntegerRange auto_exposure_roi_width_range;
        auto_exposure_roi_width_range.set__from_value(0)
                       .set__to_value(device_info_.imagerWidth)
                       .set__step(1);

        rcl_interfaces::msg::ParameterDescriptor auto_exposure_roi_width_desc;
        auto_exposure_roi_width_desc.set__name("auto_exposure_roi_width")
                                 .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                                 .set__description("auto exposure ROI width value")
                                 .set__integer_range({auto_exposure_roi_width_range});
        declare_parameter("auto_exposure_roi_width", crl::multisense::Roi_Full_Image, auto_exposure_roi_width_desc);

        //
        // Auto exposure ROI height
        //
        rcl_interfaces::msg::IntegerRange auto_exposure_roi_height_range;
        auto_exposure_roi_height_range.set__from_value(0)
                       .set__to_value(device_info_.imagerHeight)
                       .set__step(1);

        rcl_interfaces::msg::ParameterDescriptor auto_exposure_roi_height_desc;
        auto_exposure_roi_height_desc.set__name("auto_exposure_roi_height")
                                 .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                                 .set__description("auto exposure ROI height value")
                                 .set__integer_range({auto_exposure_roi_height_range});
        declare_parameter("auto_exposure_roi_height", crl::multisense::Roi_Full_Image, auto_exposure_roi_height_desc);

    }

    if (has_aux_camera_ && aux_control_supported_)
    {
        //
        // Aux Gain

        rcl_interfaces::msg::FloatingPointRange aux_gain_range;
        aux_gain_range.set__from_value(1.68421)
                      .set__to_value(16.0)
                      .set__step(0.00001);

        rcl_interfaces::msg::ParameterDescriptor aux_gain_desc;
        aux_gain_desc.set__name("aux_gain")
                     .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                     .set__description("aux_imager gain")
                     .set__floating_point_range({aux_gain_range});
        declare_parameter("aux_gain", 1.68421, aux_gain_desc);

        //
        // Aux Auto exposure enable

        rcl_interfaces::msg::ParameterDescriptor aux_auto_exposure_enable_desc;
        aux_auto_exposure_enable_desc.set__name("aux_auto_exposure")
                                     .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
                                     .set__description("aus enable auto exposure");
        declare_parameter("aux_auto_exposure", true, aux_auto_exposure_enable_desc);

        //
        // Aux Auto exposure max time

        rcl_interfaces::msg::FloatingPointRange aux_auto_exposure_max_time_range;
        aux_auto_exposure_max_time_range.set__from_value(0.0)
                                        .set__to_value(0.033)
                                        .set__step(0.001);

        rcl_interfaces::msg::ParameterDescriptor aux_auto_exposure_max_time_desc;
        aux_auto_exposure_max_time_desc.set__name("aux_auto_exposure_max_time")
                                       .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                                       .set__description("aux max exposure time using auto exposure")
                                       .set__floating_point_range({aux_auto_exposure_max_time_range});
        declare_parameter("aux_auto_exposure_max_time", 0.01, aux_auto_exposure_max_time_desc);

        //
        // Aux Auto exposure decay

        rcl_interfaces::msg::IntegerRange aux_auto_exposure_decay_range;
        aux_auto_exposure_decay_range.set__from_value(1)
                                     .set__to_value(10);

        rcl_interfaces::msg::ParameterDescriptor aux_auto_exposure_decay_desc;
        aux_auto_exposure_decay_desc.set__name("aux_auto_exposure_decay")
                                    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                                    .set__description("aux_auto exposure time decay")
                                    .set__integer_range({aux_auto_exposure_decay_range});
        declare_parameter("aux_auto_exposure_decay", 7, aux_auto_exposure_decay_desc);

        //
        // Aux Auto exposure threshold

        rcl_interfaces::msg::FloatingPointRange aux_auto_exposure_thresh_range;
        aux_auto_exposure_thresh_range.set__from_value(0.0)
                                      .set__to_value(1.0)
                                      .set__step(0.01);

        rcl_interfaces::msg::ParameterDescriptor aux_auto_exposure_thresh_desc;
        aux_auto_exposure_thresh_desc.set__name("aux_auto_exposure_thresh")
                                 .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                                 .set__description("aux auto exposure threshold")
                                 .set__floating_point_range({aux_auto_exposure_thresh_range});
        declare_parameter("aux_auto_exposure_thresh", 0.85, aux_auto_exposure_thresh_desc);

        //
        // Aux Auto exposure target intensity

        rcl_interfaces::msg::FloatingPointRange aux_auto_exposure_target_intensity_range;
        aux_auto_exposure_target_intensity_range.set__from_value(0.0)
                                                .set__to_value(1.0)
                                                .set__step(0.01);

        rcl_interfaces::msg::ParameterDescriptor aux_auto_exposure_target_intensity_desc;
        aux_auto_exposure_target_intensity_desc.set__name("aux_auto_exposure_target_intensity")
                                                .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                                                .set__description("aux auto exposure target intensity")
                                                .set__floating_point_range({aux_auto_exposure_target_intensity_range});
        declare_parameter("aux_auto_exposure_target_intensity", 0.5, aux_auto_exposure_target_intensity_desc);

        //
        // Aux Exposure time

        rcl_interfaces::msg::FloatingPointRange aux_exposure_time_range;
        aux_exposure_time_range.set__from_value(0.0)
                               .set__to_value(0.033)
                               .set__step(0.000001);

        rcl_interfaces::msg::ParameterDescriptor aux_exposure_time_desc;
        aux_exposure_time_desc.set__name("aux_exposure_time")
                              .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                              .set__description("aux imager exposure time in seconds")
                              .set__floating_point_range({aux_exposure_time_range});
        declare_parameter("aux_exposure_time", 0.005, aux_exposure_time_desc);

        //
        // Aux Auto white balance enable

        rcl_interfaces::msg::ParameterDescriptor aux_auto_white_balance_enable_desc;
        aux_auto_white_balance_enable_desc.set__name("aux_ auto_white_balance")
                                          .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
                                          .set__description("enable aux auto white balance");
        declare_parameter("aux_auto_white_balance", true, aux_auto_white_balance_enable_desc);

        //
        // Aux Auto white balance decay

        rcl_interfaces::msg::IntegerRange aux_auto_white_balance_decay_range;
        aux_auto_white_balance_decay_range.set__from_value(0)
                                          .set__to_value(20);

        rcl_interfaces::msg::ParameterDescriptor aux_auto_white_balance_decay_desc;
        aux_auto_white_balance_decay_desc.set__name("aux_auto_white_balance_decay")
                                         .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                                         .set__description("aux auto white balance decay")
                                         .set__integer_range({aux_auto_white_balance_decay_range});
        declare_parameter("aux_auto_white_balance_decay", 3, aux_auto_white_balance_decay_desc);

        //
        // Aux Auto white balance thresh

        rcl_interfaces::msg::FloatingPointRange aux_auto_white_balance_thresh_range;
        aux_auto_white_balance_thresh_range.set__from_value(0.0)
                                           .set__to_value(1.0)
                                           .set__step(0.01);

        rcl_interfaces::msg::ParameterDescriptor aux_auto_white_balance_thresh_desc;
        aux_auto_white_balance_thresh_desc.set__name("aux_auto_white_balance_thresh")
                                          .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                                          .set__description("aux auto white balance threshold")
                                          .set__floating_point_range({aux_auto_white_balance_thresh_range});
        declare_parameter("aux_auto_white_balance_thresh", 0.5, aux_auto_white_balance_thresh_desc);

        //
        // Aux Auto white balance red

        rcl_interfaces::msg::FloatingPointRange aux_auto_white_balance_red_range;
        aux_auto_white_balance_red_range.set__from_value(0.25)
                                        .set__to_value(4.0)
                                        .set__step(0.01);

        rcl_interfaces::msg::ParameterDescriptor aux_auto_white_balance_red_desc;
        aux_auto_white_balance_red_desc.set__name("aux_auto_white_balance_red")
                                       .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                                       .set__description("aux auto white balance red gain")
                                       .set__floating_point_range({aux_auto_white_balance_red_range});
        declare_parameter("aux_auto_white_balance_red", 1.0, aux_auto_white_balance_red_desc);

        //
        // Aux Auto white balance blue

        rcl_interfaces::msg::FloatingPointRange aux_auto_white_balance_blue_range;
        aux_auto_white_balance_blue_range.set__from_value(0.25)
                                         .set__to_value(4.0)
                                         .set__step(0.01);

        rcl_interfaces::msg::ParameterDescriptor aux_auto_white_balance_blue_desc;
        aux_auto_white_balance_blue_desc.set__name("aux_auto_white_balance_blue")
                                         .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                                         .set__description("aux auto white balance blue gain")
                                         .set__floating_point_range({aux_auto_white_balance_blue_range});
        declare_parameter("aux_auto_white_balance_blue", 1.0, aux_auto_white_balance_blue_desc);

        //
        // Aux HDR enable

        rcl_interfaces::msg::ParameterDescriptor aux_hdr_enable_desc;
        aux_hdr_enable_desc.set__name("aux_hdr_enable")
                       .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
                       .set__description("enable aux hdr");
        declare_parameter("aux_hdr_enable", false, aux_hdr_enable_desc);

        //
        // Aux gamma
        //
        rcl_interfaces::msg::FloatingPointRange aux_gamma_range;
        aux_gamma_range.set__from_value(1.0)
                       .set__to_value(2.2)
                       .set__step(0.01);

        rcl_interfaces::msg::ParameterDescriptor aux_gamma_desc;
        aux_gamma_desc.set__name("aux_gamma")
                      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                      .set__description("aux gamma")
                      .set__floating_point_range({aux_gamma_range});
        declare_parameter("aux_gamma", 2.2, aux_gamma_desc);

        //
        // Aux enable sharpening

        rcl_interfaces::msg::ParameterDescriptor aux_sharpening_enable_desc;
        aux_sharpening_enable_desc.set__name("aux_sharpening_enable")
                                   .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
                                   .set__description("enable aux sharpening");
        declare_parameter("aux_sharpening_enable", false, aux_sharpening_enable_desc);

        //
        // Aux sharpening percentage
        //
        rcl_interfaces::msg::FloatingPointRange aux_sharpening_percentage_range;
        aux_sharpening_percentage_range.set__from_value(0.0)
                                       .set__to_value(100.0)
                                       .set__step(0.01);

        rcl_interfaces::msg::ParameterDescriptor aux_sharpening_percentage_desc;
        aux_sharpening_percentage_desc.set__name("aux_sharpening_percentage")
                                      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                                      .set__description("aux sharpening percentage")
                                      .set__floating_point_range({aux_sharpening_percentage_range});
        declare_parameter("aux_sharpening_percentage", 0.0, aux_sharpening_percentage_desc);

        //
        // Aux sharpening limit
        //
        rcl_interfaces::msg::IntegerRange aux_sharpening_limit_range;
        aux_sharpening_limit_range.set__from_value(0)
                                  .set__to_value(255)
                                  .set__step(1);

        rcl_interfaces::msg::ParameterDescriptor aux_sharpening_limit_desc;
        aux_sharpening_limit_desc.set__name("aux_sharpening_limit")
                                 .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                                 .set__description("aux sharpening limit")
                                 .set__integer_range({aux_sharpening_limit_range});
        declare_parameter("aux_sharpening_limit", 0, aux_sharpening_limit_desc);

        //
        // Aux enable ROI auto exposure

        rcl_interfaces::msg::ParameterDescriptor aux_auto_exposure_roi_enable_desc;
        aux_auto_exposure_roi_enable_desc.set__name("aux_auto_exposure_roi_enable")
                                   .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
                                   .set__description("enable aux auto exposure ROI");
        declare_parameter("aux_auto_exposure_roi_enable", false, aux_auto_exposure_roi_enable_desc);

        //
        // Aux ROI auto exposure x
        //
        rcl_interfaces::msg::IntegerRange aux_auto_exposure_roi_x_range;
        aux_auto_exposure_roi_x_range.set__from_value(0)
                       .set__to_value(device_info_.imagerWidth)
                       .set__step(1);

        rcl_interfaces::msg::ParameterDescriptor aux_auto_exposure_roi_x_desc;
        aux_auto_exposure_roi_x_desc.set__name("aux_auto_exposure_roi_x")
                                 .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                                 .set__description("aux auto exposure ROI x value")
                                 .set__integer_range({aux_auto_exposure_roi_x_range});
        declare_parameter("aux_auto_exposure_roi_x", 0, aux_auto_exposure_roi_x_desc);

        //
        // Aux ROI auto exposure y
        //
        rcl_interfaces::msg::IntegerRange aux_auto_exposure_roi_y_range;
        aux_auto_exposure_roi_y_range.set__from_value(0)
                       .set__to_value(device_info_.imagerHeight)
                       .set__step(1);

        rcl_interfaces::msg::ParameterDescriptor aux_auto_exposure_roi_y_desc;
        aux_auto_exposure_roi_y_desc.set__name("aux_auto_exposure_roi_y")
                                 .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                                 .set__description("aux auto exposure ROI y value")
                                 .set__integer_range({aux_auto_exposure_roi_y_range});
        declare_parameter("aux_auto_exposure_roi_y", 0, aux_auto_exposure_roi_y_desc);

        //
        // Aux ROI auto exposure width
        //
        rcl_interfaces::msg::IntegerRange aux_auto_exposure_roi_width_range;
        aux_auto_exposure_roi_width_range.set__from_value(0)
                       .set__to_value(device_info_.imagerWidth)
                       .set__step(1);

        rcl_interfaces::msg::ParameterDescriptor aux_auto_exposure_roi_width_desc;
        aux_auto_exposure_roi_width_desc.set__name("aux_auto_exposure_roi_width")
                                        .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                                        .set__description("aux auto exposure ROI width value")
                                        .set__integer_range({aux_auto_exposure_roi_width_range});
        declare_parameter("aux_auto_exposure_roi_width", crl::multisense::Roi_Full_Image, aux_auto_exposure_roi_width_desc);

        //
        // Aux ROI auto exposure height
        //
        rcl_interfaces::msg::IntegerRange aux_auto_exposure_roi_height_range;
        aux_auto_exposure_roi_height_range.set__from_value(0)
                       .set__to_value(device_info_.imagerHeight)
                       .set__step(1);

        rcl_interfaces::msg::ParameterDescriptor aux_auto_exposure_roi_height_desc;
        aux_auto_exposure_roi_height_desc.set__name("aux_auto_exposure_roi_height")
                                 .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                                 .set__description("aux auto exposure ROI height value")
                                 .set__integer_range({aux_auto_exposure_roi_height_range});
        declare_parameter("aux_auto_exposure_roi_height", crl::multisense::Roi_Full_Image, aux_auto_exposure_roi_height_desc);

    }


    //
    // Border clip type

    rcl_interfaces::msg::IntegerRange border_clip_type_range;
    border_clip_type_range.set__from_value(static_cast<int>(BorderClip::NONE))
                          .set__to_value(static_cast<int>(BorderClip::CIRCULAR))
                          .set__step(1);

    rcl_interfaces::msg::ParameterDescriptor border_clip_type_desc;
    border_clip_type_desc.set__name("border_clip_type")
                         .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                         .set__description("border clip type\n0: none\n1: rectangular clip\n2: circular clip")
                         .set__integer_range({border_clip_type_range});
    declare_parameter("border_clip_type", static_cast<int>(BorderClip::NONE), border_clip_type_desc);

    //
    // Border clip value

    const double max_clip = (device_info_.imagerType == system::DeviceInfo::IMAGER_TYPE_CMV2000_GREY ||
                             device_info_.imagerType == system::DeviceInfo::IMAGER_TYPE_CMV4000_COLOR) ? 400.0 : 200.0;
    rcl_interfaces::msg::FloatingPointRange border_clip_value_range;
    border_clip_value_range.set__from_value(0.0)
                           .set__to_value(max_clip);

    rcl_interfaces::msg::ParameterDescriptor border_clip_value_desc;
    border_clip_value_desc.set__name("border_clip_value")
                          .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                          .set__description("border clip value in pixels")
                          .set__floating_point_range({border_clip_value_range});
    declare_parameter("border_clip_value", 0.0, border_clip_value_desc);

    //
    // Max pointcloud range

    rcl_interfaces::msg::FloatingPointRange max_pointcloud_range_range;
    max_pointcloud_range_range.set__from_value(0.1)
                              .set__to_value(1000.0);

    rcl_interfaces::msg::ParameterDescriptor max_pointcloud_range_desc;
    max_pointcloud_range_desc.set__name("max pointcloud range")
                             .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                             .set__description("max distance in meters between a stereo point and the camera")
                             .set__floating_point_range({max_pointcloud_range_range});
    declare_parameter("max_pointcloud_range", pointcloud_max_range_, max_pointcloud_range_desc);
}

rcl_interfaces::msg::SetParametersResult Camera::parameterCallback(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.set__successful(true);

    auto image_config = stereo_calibration_manager_->config();
    bool update_image_config = false;

    std::optional<image::AuxConfig> aux_image_config = image::AuxConfig{};
    bool update_aux_image_config = false;
    if (aux_control_supported_)
    {
        if (const auto status = driver_->getAuxImageConfig(aux_image_config.value()); status != Status_Ok)
        {
            RCLCPP_WARN(get_logger(), "Camera: failed to query aux sensor configuration: %s",
                        Channel::statusString(status));

            aux_image_config = std::nullopt;
        }
    }
    else
    {
        aux_image_config = std::nullopt;
    }

    for (const auto &parameter : parameters)
    {
        const auto type = parameter.get_type();
        if (type == rclcpp::ParameterType::PARAMETER_NOT_SET)
        {
            continue;
        }

        const auto name = parameter.get_name();

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
                             .set__reason("Camera: Invalid sensor resolution. Must be [width, height, max disparity]");
            }

            const auto width = value[0];
            const auto height = value[1];
            const auto disparities = value[2];

            //
            // Ensure this is a valid resolution supported by the device

            const bool supported = std::find_if(std::begin(device_modes_), std::end(device_modes_),
                                                [&width, &height, &disparities](const system::DeviceMode& m)
                                                {
                                                    return width == static_cast<int64_t>(m.width) &&
                                                           height == static_cast<int64_t>(m.height) &&
                                                           disparities == static_cast<int64_t>(m.disparities);

                                                }) != std::end(device_modes_);

            if (!supported)
            {
                return result.set__successful(false).set__reason("Camera: unsupported resolution");
            }

            if (image_config.width() != width ||
                image_config.height() != height ||
                image_config.disparities() != disparities)
            {
                RCLCPP_WARN(get_logger(), "Camera: changing sensor resolution to %ldx%ld (%ld disparities), from %ux%u "
                     "(%u disparities): reconfiguration may take up to 30 seconds",
                         width, height, disparities,
                         image_config.width(), image_config.height(), image_config.disparities());

                image_config.setResolution(width, height);
                image_config.setDisparities(disparities);
                update_image_config = true;
            }
        }
        else if(name == "fps")
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid fps type");
            }

            const auto value = get_as_number<double>(parameter);
            if (image_config.fps() != value)
            {
                image_config.setFps(value);
                update_image_config = true;
            }
        }
        else if(name == "stereo_post_filtering")
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid stereo post filtering type");
            }

            const auto value = get_as_number<double>(parameter);
            if (image_config.stereoPostFilterStrength() != value)
            {
                image_config.setStereoPostFilterStrength(value);
                update_image_config = true;
            }
        }
        else if(name == "gain")
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid gain type");
            }

            const auto value = get_as_number<double>(parameter);
            if (image_config.gain() != value)
            {
                image_config.setGain(value);
                update_image_config = true;
            }
        }
        else if(name == "auto_exposure")
        {
            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
            {
                return result.set__successful(false).set__reason("invalid auto exposure type");
            }

            const auto value = parameter.as_bool();
            if (image_config.autoExposure() != value)
            {
                image_config.setAutoExposure(value);
                update_image_config = true;
            }
        }
        else if(name == "auto_exposure_max_time")
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid auto exposure max time type");
            }

            const auto value = static_cast<uint32_t>(get_as_number<double>(parameter) * 1e6);
            if (image_config.autoExposureMax() != value)
            {
                image_config.setAutoExposureMax(value);
                update_image_config = true;
            }
        }
        else if(name == "auto_exposure_decay")
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid auto exposure decay type");
            }

            const auto value = get_as_number<double>(parameter);
            if (image_config.autoExposureDecay() != value)
            {
                image_config.setAutoExposureDecay(value);
                update_image_config = true;
            }
        }
        else if(name == "auto_exposure_thresh")
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid auto exposure thresh type");
            }

            const auto value = get_as_number<double>(parameter);
            if (image_config.autoExposureThresh() != value)
            {
                image_config.setAutoExposureThresh(value);
                update_image_config = true;
            }
        }
        else if(name == "auto_exposure_target_intensity")
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid auto exposure target intensity type");
            }

            const auto value = get_as_number<double>(parameter);
            if (image_config.autoExposureTargetIntensity() != value)
            {
                image_config.setAutoExposureTargetIntensity(value);
                update_image_config = true;
            }
        }
        else if(name == "exposure_time")
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid exposure time type");
            }

            const auto value = static_cast<uint32_t>(get_as_number<double>(parameter) * 1e6);
            if (image_config.exposure() != value)
            {
                image_config.setExposure(value);
                update_image_config = true;
            }
        }
        else if(name == "auto_white_balance")
        {
            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
            {
                return result.set__successful(false).set__reason("invalid auto white balance type");
            }

            const auto value = parameter.as_bool();
            if (image_config.autoWhiteBalance() != value)
            {
                image_config.setAutoWhiteBalance(value);
                update_image_config = true;
            }
        }
        else if(name == "auto_white_balance_decay")
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid auto white balance decay type");
            }

            const auto value = get_as_number<double>(parameter);
            if (image_config.autoWhiteBalanceDecay() != value)
            {
                image_config.setAutoWhiteBalanceDecay(value);
                update_image_config = true;
            }
        }
        else if(name == "auto_white_balance_thresh")
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid auto white balance thresh type");
            }

            const auto value = get_as_number<double>(parameter);
            if (image_config.autoWhiteBalanceThresh() != value)
            {
                image_config.setAutoWhiteBalanceThresh(value);
                update_image_config = true;
            }
        }
        else if(name == "auto_white_balance_red")
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid auto white balance red type");
            }

            const auto value = get_as_number<double>(parameter);
            if (image_config.whiteBalanceRed() != value)
            {
                image_config.setWhiteBalance(value, image_config.whiteBalanceBlue());
                update_image_config = true;
            }
        }
        else if(name == "auto_white_balance_blue")
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid auto white balance blue type");
            }

            const auto value = get_as_number<double>(parameter);
            if (image_config.whiteBalanceBlue() != value)
            {
                image_config.setWhiteBalance(image_config.whiteBalanceRed(), value);
                update_image_config = true;
            }
        }
        else if(name == "hdr_enable")
        {
            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
            {
                return result.set__successful(false).set__reason("invalid hdr enable type");
            }

            const auto value = parameter.as_bool();
            if (image_config.hdrEnabled() != value)
            {
                image_config.setHdr(value);
                update_image_config = true;
            }
        }
        else if(name == "gamma")
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid gamma type");
            }

            const auto value = get_as_number<double>(parameter);
            if (image_config.gamma() != value)
            {
                image_config.setGamma(value);
                update_image_config = true;
            }
        }
        else if(name == "auto_exposure_roi_enable")
        {
            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
            {
                return result.set__successful(false).set__reason("invalid auto exposure ROI enable");
            }

            const auto value = parameter.as_bool();
            if (enable_roi_auto_exposure_control_ != value)
            {
                enable_roi_auto_exposure_control_ = value;
                update_image_config = true;
            }
        }
        else if(name == "auto_exposure_roi_x")
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid auto exposure ROI x");
            }

            const auto value = get_as_number<int>(parameter);
            if (auto_exposure_roi_.x != value)
            {
                auto_exposure_roi_.x = value;
                update_image_config = true;
            }
        }
        else if(name == "auto_exposure_roi_y")
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid auto exposure ROI y");
            }

            const auto value = get_as_number<int>(parameter);
            if (auto_exposure_roi_.y != value)
            {
                auto_exposure_roi_.y = value;
                update_image_config = true;
            }
        }
        else if(name == "auto_exposure_roi_width")
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid auto exposure ROI width");
            }

            const auto value = get_as_number<int>(parameter);
            if (auto_exposure_roi_.width != value)
            {
                auto_exposure_roi_.width = value;
                update_image_config = true;
            }
        }
        else if(name == "auto_exposure_roi_height")
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid auto exposure ROI height");
            }

            const auto value = get_as_number<int>(parameter);
            if (auto_exposure_roi_.height != value)
            {
                auto_exposure_roi_.height = value;
                update_image_config = true;
            }
        }
        else if(name == "aux_gain" && aux_image_config)
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid aux gain type");
            }

            const auto value = get_as_number<double>(parameter);
            if (aux_image_config->gain() != value)
            {
                aux_image_config->setGain(value);
                update_aux_image_config = true;
            }
        }
        else if(name == "aux_auto_exposure" && aux_image_config)
        {
            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
            {
                return result.set__successful(false).set__reason("invalid aux auto exposure type");
            }

            const auto value = parameter.as_bool();
            if (aux_image_config->autoExposure() != value)
            {
                aux_image_config->setAutoExposure(value);
                update_aux_image_config = true;
            }
        }
        else if(name == "aux_auto_exposure_max_time" && aux_image_config)
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid aux auto exposure max time type");
            }

            const auto value = static_cast<uint32_t>(get_as_number<double>(parameter) * 1e6);
            if (aux_image_config->autoExposureMax() != value)
            {
                aux_image_config->setAutoExposureMax(value);
                update_aux_image_config = true;
            }
        }
        else if(name == "aux_auto_exposure_decay" && aux_image_config)
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid aux auto exposure decay type");
            }

            const auto value = get_as_number<double>(parameter);
            if (aux_image_config->autoExposureDecay() != value)
            {
                aux_image_config->setAutoExposureDecay(value);
                update_aux_image_config = true;
            }
        }
        else if(name == "aux_auto_exposure_thresh" && aux_image_config)
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid aux auto exposure thresh type");
            }

            const auto value = get_as_number<double>(parameter);
            if (aux_image_config->autoExposureThresh() != value)
            {
                aux_image_config->setAutoExposureThresh(value);
                update_aux_image_config = true;
            }
        }
        else if(name == "aux_auto_exposure_target_intensity" && aux_image_config)
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid aux auto exposure target intensity type");
            }

            const auto value = get_as_number<double>(parameter);
            if (aux_image_config->autoExposureTargetIntensity() != value)
            {
                aux_image_config->setAutoExposureTargetIntensity(value);
                update_aux_image_config = true;
            }
        }
        else if(name == "aux_exposure_time" && aux_image_config)
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid aux exposure time type");
            }

            const auto value = static_cast<uint32_t>(get_as_number<double>(parameter) * 1e6);
            if (aux_image_config->exposure() != value)
            {
                aux_image_config->setExposure(value);
                update_aux_image_config = true;
            }
        }
        else if(name == "aux_auto_white_balance" && aux_image_config)
        {
            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
            {
                return result.set__successful(false).set__reason("invalid aux auto white balance type");
            }

            const auto value = parameter.as_bool();
            if (aux_image_config->autoWhiteBalance() != value)
            {
                aux_image_config->setAutoWhiteBalance(value);
                update_aux_image_config = true;
            }
        }
        else if(name == "aux_auto_white_balance_decay" && aux_image_config)
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid aux auto white balance decay type");
            }

            const auto value = get_as_number<double>(parameter);
            if (aux_image_config->autoWhiteBalanceDecay() != value)
            {
                aux_image_config->setAutoWhiteBalanceDecay(value);
                update_aux_image_config = true;
            }
        }
        else if(name == "aux_auto_white_balance_thresh" && aux_image_config)
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid aux auto white balance thresh type");
            }

            const auto value = get_as_number<double>(parameter);
            if (aux_image_config->autoWhiteBalanceThresh() != value)
            {
                aux_image_config->setAutoWhiteBalanceThresh(value);
                update_aux_image_config = true;
            }
        }
        else if(name == "aux_auto_white_balance_red" && aux_image_config)
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid aux auto white balance red type");
            }

            const auto value = get_as_number<double>(parameter);
            if (aux_image_config->whiteBalanceRed() != value)
            {
                aux_image_config->setWhiteBalance(value, aux_image_config->whiteBalanceBlue());
                update_aux_image_config = true;
            }
        }
        else if(name == "aux_auto_white_balance_blue" && aux_image_config)
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid aux auto white balance blue type");
            }

            const auto value = get_as_number<double>(parameter);
            if (aux_image_config->whiteBalanceBlue() != value)
            {
                aux_image_config->setWhiteBalance(aux_image_config->whiteBalanceRed(), value);
                update_aux_image_config = true;
            }
        }
        else if(name == "aux_hdr_enable" && aux_image_config)
        {
            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
            {
                return result.set__successful(false).set__reason("invalid aux hdr enable type");
            }

            const auto value = parameter.as_bool();
            if (aux_image_config->hdrEnabled() != value)
            {
                aux_image_config->setHdr(value);
                update_aux_image_config = true;
            }
        }
        else if(name == "aux_gamma" && aux_image_config)
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid aux gamma type");
            }

            const auto value = get_as_number<double>(parameter);
            if (aux_image_config->gamma() != value)
            {
                aux_image_config->setGamma(value);
                update_aux_image_config = true;
            }
        }
        else if(name == "aux_sharpening_enable" && aux_image_config)
        {
            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
            {
                return result.set__successful(false).set__reason("invalid aux sharpening enable type");
            }

            const auto value = parameter.as_bool();
            if (aux_image_config->enableSharpening() != value)
            {
                aux_image_config->enableSharpening(value);
                update_aux_image_config = true;
            }
        }
        else if(name == "aux_sharpening_percentage" && aux_image_config)
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid aux sharpening percentage type");
            }

            const auto value = get_as_number<double>(parameter);
            if (aux_image_config->sharpeningPercentage() != value)
            {
                aux_image_config->setSharpeningPercentage(value);
                update_aux_image_config = true;
            }
        }
        else if(name == "aux_sharpening_limit" && aux_image_config)
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid aux sharpening limit type");
            }

            const auto value = get_as_number<int>(parameter);
            if (aux_image_config->sharpeningLimit() != value)
            {
                aux_image_config->setSharpeningLimit(value);
                update_aux_image_config = true;
            }
        }
        else if(name == "aux_auto_exposure_roi_enable" && aux_image_config)
        {
            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
            {
                return result.set__successful(false).set__reason("invalid aux auto exposure ROI enable");
            }

            const auto value = parameter.as_bool();
            if (enable_aux_roi_auto_exposure_control_ != value)
            {
                enable_aux_roi_auto_exposure_control_ = value;
                update_aux_image_config = true;
            }
        }
        else if(name == "aux_auto_exposure_roi_x" && aux_image_config)
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid aux auto exposure ROI x");
            }

            const auto value = get_as_number<int>(parameter);
            if (aux_auto_exposure_roi_.x != value)
            {
                aux_auto_exposure_roi_.x = value;
                update_aux_image_config = true;
            }
        }
        else if(name == "aux_auto_exposure_roi_y" && aux_image_config)
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid aux auto exposure ROI y");
            }

            const auto value = get_as_number<int>(parameter);
            if (aux_auto_exposure_roi_.y != value)
            {
                aux_auto_exposure_roi_.y = value;
                update_aux_image_config = true;
            }
        }
        else if(name == "aux_auto_exposure_roi_width" && aux_image_config)
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid aux auto exposure ROI width");
            }

            const auto value = get_as_number<int>(parameter);
            if (aux_auto_exposure_roi_.width != value)
            {
                aux_auto_exposure_roi_.width = value;
                update_aux_image_config = true;
            }
        }
        else if(name == "aux_auto_exposure_roi_height" && aux_image_config)
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid aux auto exposure ROI height");
            }

            const auto value = get_as_number<int>(parameter);
            if (aux_auto_exposure_roi_.height != value)
            {
                aux_auto_exposure_roi_.height = value;
                update_aux_image_config = true;
            }
        }
        else if (name == "border_clip_type")
        {
            if (type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid border clip type");
            }

            const auto value = static_cast<BorderClip>(get_as_number<int>(parameter));
            if (border_clip_type_ != value)
            {
                border_clip_type_ = value;
            }
        }
        else if (name == "border_clip_value")
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid border clip value type");
            }

            const auto value = get_as_number<double>(parameter);
            if (border_clip_value_ != value)
            {
                border_clip_value_ = value;
            }
        }
        else if (name == "max_pointcloud_range")
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid max pointcloud range type");
            }

            pointcloud_max_range_ = get_as_number<double>(parameter);
        }
    }

    //
    // Update our ROI if we are enabled/disabled
    if (enable_roi_auto_exposure_control_)
    {
        image_config.setAutoExposureRoi(auto_exposure_roi_.x,
                                        auto_exposure_roi_.y,
                                        auto_exposure_roi_.width,
                                        auto_exposure_roi_.height);
    }
    else
    {
        image_config.setAutoExposureRoi(0, 0, crl::multisense::Roi_Full_Image, crl::multisense::Roi_Full_Image);
    }

    if (update_image_config)
    {
        if (const auto status = driver_->setImageConfig(image_config); status != Status_Ok)
        {
            return result.set__successful(false).set__reason(Channel::statusString(status));
        }

        //
        // TODO remove after firmware fixes
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        if (const auto status = driver_->getImageConfig(image_config); status != Status_Ok)
        {
            RCLCPP_ERROR(get_logger(), "Camera: failed to query sensor configuration: %s",
                         Channel::statusString(status));

            return result.set__successful(false).set__reason(Channel::statusString(status));
        }

        //
        // This is a no-op if the resolution of the camera did not change
        updateConfig(image_config);
    }

    //
    // Update our aux ROI if we are enabled/disabled
    if (enable_aux_roi_auto_exposure_control_)
    {
        aux_image_config->setAutoExposureRoi(aux_auto_exposure_roi_.x,
                                             aux_auto_exposure_roi_.y,
                                             aux_auto_exposure_roi_.width,
                                             aux_auto_exposure_roi_.height);
    }
    else
    {
        aux_image_config->setAutoExposureRoi(0, 0, crl::multisense::Roi_Full_Image, crl::multisense::Roi_Full_Image);
    }

    if (update_aux_image_config && aux_image_config)
    {
        if (const auto status = driver_->setAuxImageConfig(aux_image_config.value()); status != Status_Ok)
        {
            return result.set__successful(false).set__reason(Channel::statusString(status));
        }

        //
        // TODO remove after firmware fixes
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return result;
}

} // namespace
