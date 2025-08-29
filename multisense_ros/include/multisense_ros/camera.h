/**
 * @file camera.h
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

#ifndef MULTISENSE_ROS_CAMERA_H
#define MULTISENSE_ROS_CAMERA_H

#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <multisense_msgs/msg/device_info.hpp>
#include <multisense_msgs/msg/histogram.hpp>
#include <multisense_msgs/msg/raw_cam_cal.hpp>
#include <multisense_msgs/msg/raw_cam_config.hpp>
#include <multisense_msgs/msg/raw_cam_data.hpp>

#include <MultiSense/MultiSenseChannel.hh>

#include <multisense_ros/camera_utilities.h>

#include <optional>

namespace multisense_ros {

enum class BorderClip {NONE, RECTANGULAR, CIRCULAR};

struct RegionOfIntrest
{
    int x = 0;
    int y = 0;
    int width = crl::multisense::Roi_Full_Image;
    int height = crl::multisense::Roi_Full_Image;
};

class Camera : public rclcpp::Node
{
public:
    Camera(const std::string& node_name,
           const rclcpp::NodeOptions& options,
           crl::multisense::Channel* driver,
           const std::string& tf_prefix,
           bool use_sensor_qos);

    ~Camera();

    void monoCallback(const crl::multisense::image::Header& header);
    void rectCallback(const crl::multisense::image::Header& header);
    void depthCallback(const crl::multisense::image::Header& header);
    void pointCloudCallback(const crl::multisense::image::Header& header);
    void rawCamDataCallback(const crl::multisense::image::Header& header);
    void colorImageCallback(const crl::multisense::image::Header& header);
    void disparityImageCallback(const crl::multisense::image::Header& header);
    void histogramCallback(const crl::multisense::image::Header& header);
    void colorizeCallback(const crl::multisense::image::Header& header);

private:

    //
    // Node names

    static constexpr char LEFT[] = "left";
    static constexpr char RIGHT[] = "right";
    static constexpr char AUX[] = "aux";
    static constexpr char CALIBRATION[] = "calibration";

    //
    // Frames

    static constexpr char LEFT_CAMERA_FRAME[] = "/left_camera_frame";
    static constexpr char LEFT_RECTIFIED_FRAME[] = "/left_camera_optical_frame";
    static constexpr char RIGHT_CAMERA_FRAME[] = "/right_camera_frame";
    static constexpr char RIGHT_RECTIFIED_FRAME[] = "/right_camera_optical_frame";
    static constexpr char AUX_CAMERA_FRAME[] = "/aux_camera_frame";
    static constexpr char AUX_RECTIFIED_FRAME[] = "/aux_camera_optical_frame";

    //
    // Topic names

    static constexpr char DEVICE_INFO_TOPIC[] = "device_info";
    static constexpr char RAW_CAM_CAL_TOPIC[] = "raw_cam_cal";
    static constexpr char RAW_CAM_CONFIG_TOPIC[] = "raw_cam_config";
    static constexpr char RAW_CAM_DATA_TOPIC[] = "raw_cam_data";
    static constexpr char HISTOGRAM_TOPIC[] = "histogram";
    static constexpr char MONO_TOPIC[] = "image_mono";
    static constexpr char RECT_TOPIC[] = "image_rect";
    static constexpr char DISPARITY_TOPIC[] = "disparity";
    static constexpr char DISPARITY_IMAGE_TOPIC[] = "disparity_image";
    static constexpr char DEPTH_TOPIC[] = "depth";
    static constexpr char OPENNI_DEPTH_TOPIC[] = "openni_depth";
    static constexpr char COST_TOPIC[] = "cost";
    static constexpr char COLOR_TOPIC[] = "image_color";
    static constexpr char RECT_COLOR_TOPIC[] = "image_rect_color";
    static constexpr char POINTCLOUD_TOPIC[] = "image_points2";
    static constexpr char COLOR_POINTCLOUD_TOPIC[] = "image_points2_color";
    static constexpr char ORGANIZED_POINTCLOUD_TOPIC[] = "organized_image_points2";
    static constexpr char COLOR_ORGANIZED_POINTCLOUD_TOPIC[] = "organized_image_points2_color";
    static constexpr char MONO_CAMERA_INFO_TOPIC[] = "image_mono/camera_info";
    static constexpr char RECT_CAMERA_INFO_TOPIC[] = "image_rect/camera_info";
    static constexpr char COLOR_CAMERA_INFO_TOPIC[] = "image_color/camera_info";
    static constexpr char RECT_COLOR_CAMERA_INFO_TOPIC[] = "image_rect_color/camera_info";
    static constexpr char DEPTH_CAMERA_INFO_TOPIC[] = "depth/camera_info";
    static constexpr char DISPARITY_CAMERA_INFO_TOPIC[] = "disparity/camera_info";
    static constexpr char COST_CAMERA_INFO_TOPIC[] = "cost/camera_info";

    //
    // Device stream control

    size_t numSubscribers(const rclcpp::Node::SharedPtr node, const std::string &topic);
    size_t numSubscribers(const rclcpp::Node* node, const std::string &topic);
    void stop();

    //
    // Update the sensor calibration parameters

    void updateConfig(const crl::multisense::image::Config& config);

    //
    // Republish camera info messages by publishing the current messages
    // Used whenever the resolution of the camera changes

    void publishAllCameraInfo();

    //
    // Callback to check subscriptions to our publishers. This duplicates the behavior of the ROS1
    // SubscriberStatusCallback in a much less elegant manner. Until that functionality is added to ROS2 we will
    // poll to enable our lazy publishing scheme.

    void timerCallback();

    //
    // Helper function to setup the nodes configuration parameters. In ROS2 this takes the place of dynamic
    // reconfigure

    void initalizeParameters(const crl::multisense::image::Config& config);

    //
    // Parameter management

    OnSetParametersCallbackHandle::SharedPtr paramter_handle_;

    rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter>& parameters);

    //
    // CRL sensor API

    crl::multisense::Channel* driver_;

    //
    // ROS2 timer for checking publisher status

    rclcpp::TimerBase::SharedPtr timer_;

    //
    // Sub nodes

    rclcpp::Node::SharedPtr left_node_;
    rclcpp::Node::SharedPtr right_node_;
    rclcpp::Node::SharedPtr aux_node_;
    rclcpp::Node::SharedPtr calibration_node_;

    //
    // Data publishers

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  left_mono_cam_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  right_mono_cam_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  left_rect_cam_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  right_rect_cam_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  depth_cam_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  ni_depth_cam_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  left_rgb_cam_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  left_rgb_rect_cam_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  aux_rgb_cam_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  aux_mono_cam_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  aux_rect_cam_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  aux_rgb_rect_cam_pub_;

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_mono_cam_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_mono_cam_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_rect_cam_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_rect_cam_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_disp_cam_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_disp_cam_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_cost_cam_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_rgb_cam_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_rgb_rect_cam_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_cam_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr aux_mono_cam_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr aux_rgb_cam_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr aux_rect_cam_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr aux_rgb_rect_cam_info_pub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr luma_point_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr color_point_cloud_pub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr luma_organized_point_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr color_organized_point_cloud_pub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_disparity_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_disparity_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_disparity_cost_pub_;

    rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr left_stereo_disparity_pub_;
    rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr right_stereo_disparity_pub_;

    //
    // Raw data publishers
    //
    rclcpp::Publisher<multisense_msgs::msg::RawCamData>::SharedPtr raw_cam_data_pub_;
    rclcpp::Publisher<multisense_msgs::msg::RawCamConfig>::SharedPtr raw_cam_config_pub_;
    rclcpp::Publisher<multisense_msgs::msg::RawCamCal>::SharedPtr raw_cam_cal_pub_;
    rclcpp::Publisher<multisense_msgs::msg::DeviceInfo>::SharedPtr device_info_pub_;
    rclcpp::Publisher<multisense_msgs::msg::Histogram>::SharedPtr histogram_pub_;

    //
    // Store outgoing messages to avoid repeated allocations

    sensor_msgs::msg::Image         left_mono_image_;
    sensor_msgs::msg::Image         right_mono_image_;
    sensor_msgs::msg::Image         left_rect_image_;
    sensor_msgs::msg::Image         right_rect_image_;
    sensor_msgs::msg::Image         depth_image_;
    sensor_msgs::msg::Image         ni_depth_image_;
    sensor_msgs::msg::PointCloud2   luma_point_cloud_;
    sensor_msgs::msg::PointCloud2   color_point_cloud_;
    sensor_msgs::msg::PointCloud2   luma_organized_point_cloud_;
    sensor_msgs::msg::PointCloud2   color_organized_point_cloud_;

    sensor_msgs::msg::Image         aux_mono_image_;
    sensor_msgs::msg::Image         left_rgb_image_;
    sensor_msgs::msg::Image         aux_rgb_image_;
    sensor_msgs::msg::Image         left_rgb_rect_image_;
    sensor_msgs::msg::Image         aux_rect_image_;
    sensor_msgs::msg::Image         aux_rect_rgb_image_;

    sensor_msgs::msg::Image         left_disparity_image_;
    sensor_msgs::msg::Image         left_disparity_cost_image_;
    sensor_msgs::msg::Image         right_disparity_image_;

    stereo_msgs::msg::DisparityImage left_stereo_disparity_;
    stereo_msgs::msg::DisparityImage right_stereo_disparity_;

    multisense_msgs::msg::RawCamData raw_cam_data_;

    std::vector<uint8_t> pointcloud_color_buffer_;
    std::vector<uint8_t> pointcloud_rect_color_buffer_;

    //
    // Calibration from sensor

    crl::multisense::system::VersionInfo version_info_;
    crl::multisense::system::DeviceInfo  device_info_;
    std::vector<crl::multisense::system::DeviceMode> device_modes_;

    //
    // Calibration manager

    StereoCalibrationManger::SharedPtr stereo_calibration_manager_;

    //
    // The frame IDs

    const std::string frame_id_left_;
    const std::string frame_id_right_;
    const std::string frame_id_aux_;
    const std::string frame_id_rectified_left_;
    const std::string frame_id_rectified_right_;
    const std::string frame_id_rectified_aux_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    //
    // For pointcloud generation

    double pointcloud_max_range_;

    //
    // Stream subscriptions

    crl::multisense::DataSource active_streams_;

    //
    // Histogram tracking

    int64_t last_frame_id_ = -1;

    //
    // The mask used to perform the border clipping of the disparity image

    BorderClip border_clip_type_;
    double border_clip_value_ = 0.0;

    //
    // Storage of images which we use for pointcloud colorizing

    std::unordered_map<crl::multisense::DataSource, std::shared_ptr<BufferWrapper<crl::multisense::image::Header>>> image_buffers_;

    //
    // Has a 3rd aux color camera

    bool has_aux_camera_ = false;
    bool aux_control_supported_ = false;

    //
    // Supports color images (either left color, right color, or aux color)

    bool supports_color_ = false;

    //
    // Contains the next-gen stereo hardware (i.e S30/S27 etc)

    bool next_gen_camera_ = false;

    //
    // ROI control

    bool enable_roi_auto_exposure_control_ = false;
    bool enable_aux_roi_auto_exposure_control_ = false;

    RegionOfIntrest auto_exposure_roi_;
    RegionOfIntrest aux_auto_exposure_roi_;

};

}// namespace

#endif
