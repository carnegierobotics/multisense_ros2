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

#pragma once

#include <condition_variable>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <multisense_msgs/msg/histogram.hpp>
#include <multisense_msgs/msg/info.hpp>
#include <multisense_msgs/msg/status.hpp>

#include "multisense_ros/publisher_utilities.h"
#include <multisense_ros/multisense_parameters.hpp>

#include <MultiSense/MultiSenseChannel.hh>

namespace multisense_ros {

enum class TimestampSource
{
    CAMERA,
    SYSTEM,
    PTP
};

template <typename T>
class FrameNotifier
{
public:

    FrameNotifier() = default;

    ~FrameNotifier()
    {
        cv_.notify_all();
    }

    void notify_all()
    {
        cv_.notify_all();
    };

    ///
    /// @brief Copy a frame into the local storage, and notify all waiters that the frame is valid
    ///
    void set_and_notify(const T &in_frame)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        frame_ = in_frame;
        cv_.notify_all();
    }

    ///
    /// @brief Wait for the notifier to be valid. If the timeout is invalid, will wait forever
    ///
    template <class Rep, class Period>
    std::optional<T> wait(const std::optional<std::chrono::duration<Rep, Period>>& timeout)
    {
        std::unique_lock<std::mutex> lock(mutex_);

        std::optional<T> output_frame = std::nullopt;
        if (timeout)
        {
            if (std::cv_status::no_timeout == cv_.wait_for(lock, timeout.value()))
            {
                output_frame = frame_;
            }
        }
        else
        {
            cv_.wait(lock);
            output_frame = frame_;
        }

        return output_frame;
    }

    std::optional<T> wait()
    {
        const std::optional<std::chrono::milliseconds> timeout = std::nullopt;
        return wait(timeout);
    }

private:

    std::mutex mutex_;
    std::condition_variable cv_;
    std::optional<T> frame_;
};

class MultiSense : public rclcpp::Node
{
public:
    MultiSense(const std::string& node_name,
           const rclcpp::NodeOptions& options,
           std::unique_ptr<multisense::Channel> channel,
           const std::string& tf_prefix,
           bool use_image_transport,
           bool use_sensor_qos);

    ~MultiSense();

private:

    //
    // Node names

    static constexpr char LEFT[] = "left";
    static constexpr char RIGHT[] = "right";
    static constexpr char AUX[] = "aux";
    static constexpr char IMU[] = "imu";

    //
    // Frames

    static constexpr char LEFT_CAMERA_FRAME[] = "/left_camera_frame";
    static constexpr char LEFT_RECTIFIED_FRAME[] = "/left_camera_optical_frame";
    static constexpr char RIGHT_CAMERA_FRAME[] = "/right_camera_frame";
    static constexpr char RIGHT_RECTIFIED_FRAME[] = "/right_camera_optical_frame";
    static constexpr char AUX_CAMERA_FRAME[] = "/aux_camera_frame";
    static constexpr char AUX_RECTIFIED_FRAME[] = "/aux_camera_optical_frame";
    static constexpr char IMU_FRAME[] = "/imu_frame";

    //
    // Topic names

    static constexpr char INFO_TOPIC[] = "info";
    static constexpr char STATUS_TOPIC[] = "status";
    static constexpr char HISTOGRAM_TOPIC[] = "histogram";
    static constexpr char RAW_CONFIG_TOPIC[] = "raw_config";
    static constexpr char MONO_TOPIC[] = "image_mono";
    static constexpr char RECT_TOPIC[] = "image_rect";
    static constexpr char DISPARITY_TOPIC[] = "disparity";
    static constexpr char DISPARITY_IMAGE_TOPIC[] = "disparity_image";
    static constexpr char DEPTH_TOPIC[] = "depth";
    static constexpr char OPENNI_DEPTH_TOPIC[] = "openni_depth";
    static constexpr char COST_TOPIC[] = "cost";
    static constexpr char COLOR_TOPIC[] = "image_color";
    static constexpr char RECT_COLOR_TOPIC[] = "image_rect_color";
    static constexpr char POINTCLOUD_TOPIC[] = "points2";
    static constexpr char LUMA_POINTCLOUD_TOPIC[] = "image_points2";
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

    static constexpr char IMU_TOPIC[] = "imu_data";

    //
    // Device stream control

    size_t num_subscribers(const rclcpp::Node::SharedPtr &node, const std::string &topic) const;
    size_t num_subscribers(const rclcpp::Node* node, const std::string &topic) const;
    void stop();

    //
    // Create publisher options which enable disable sources on active subscriptions

    rclcpp::PublisherOptions create_publisher_options(const std::vector<multisense::DataSource> &sources,
                                                      const std::string &topic);

    //
    // Publish static transforms for convenience

    void publish_static_tf(const multisense::StereoCalibration &stereo_calibration);

    //
    // Function to publish device info for convenience

    void publish_info(const multisense::MultiSenseInfo &info);

    //
    // Function to publish config for convenience

    void publish_config(const multisense::MultiSenseConfig &config);

    //
    // Function to publish status for convenience

    void publish_status(const multisense::MultiSenseStatus &status);

    //
    // Function which waits for image frames from the camera, and publishes images if there is
    // an active subscription to the corresponding image topic

    void image_publisher();

    //
    // Function which waits for image frames from the camera, and publishes depth images if there is
    // an active subscription to the corresponding depth topic

    void depth_publisher();

    //
    // Function which waits for image frames from the camera, and publishes point clouds if there is
    // an active subscription to the corresponding point cloud topic

    void point_cloud_publisher();

    //
    // Function which waits for image frames from the camera, and publishes color images if there is
    // an active subscription to the corresponding color image topic

    void color_publisher();

    //
    // Function which waits for imu frames from the camera, and publishes imu data  if there is
    // an active subscription to the corresponding imu data topic

    void imu_publisher();

    //
    // Helper function to setup the nodes configuration parameters. This setups parameters which were were unable
    // to configure statically via the generate__parameter_library

    void initialize_parameters(const multisense::MultiSenseConfig &config, const multisense::MultiSenseInfo& info);

    //
    // Helper function to initialize IMU rate/range parameters

    void create_imu_parameters(const multisense::MultiSenseInfo::ImuInfo::Source &imu_source,
                               const multisense::MultiSenseConfig::ImuConfig::OperatingMode &operating_mode,
                               const std::string &parameter_base);

    //
    // Helper function to udpate the config from parameters

    multisense::MultiSenseConfig update_config(multisense::MultiSenseConfig config);

    //
    // Parameter management

    OnSetParametersCallbackHandle::SharedPtr paramter_handle_ = nullptr;
    std::shared_ptr<multisense::ParamListener> param_listener_ = nullptr;

    rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter>& parameters);

    std::atomic_bool shutdown_ = false;

    //
    // CRL sensor API

    std::unique_ptr<multisense::Channel> channel_ = nullptr;

    //
    // Sub nodes

    rclcpp::Node::SharedPtr left_node_ = nullptr;
    rclcpp::Node::SharedPtr right_node_ = nullptr;
    rclcpp::Node::SharedPtr aux_node_ = nullptr;
    rclcpp::Node::SharedPtr imu_node_ = nullptr;

    //
    // Timer callback object for publishing status

    rclcpp::TimerBase::SharedPtr status_timer_ = nullptr;

    //
    // Data publishers

    std::shared_ptr<ImagePublisher> left_mono_cam_pub_ = nullptr;
    std::shared_ptr<ImagePublisher> right_mono_cam_pub_ = nullptr;
    std::shared_ptr<ImagePublisher> left_rect_cam_pub_ = nullptr;
    std::shared_ptr<ImagePublisher> right_rect_cam_pub_ = nullptr;
    std::shared_ptr<ImagePublisher> depth_cam_pub_ = nullptr;
    std::shared_ptr<ImagePublisher> ni_depth_cam_pub_ = nullptr;
    std::shared_ptr<ImagePublisher> aux_rgb_cam_pub_ = nullptr;
    std::shared_ptr<ImagePublisher> aux_mono_cam_pub_ = nullptr;
    std::shared_ptr<ImagePublisher> aux_rect_cam_pub_ = nullptr;
    std::shared_ptr<ImagePublisher> aux_rgb_rect_cam_pub_ = nullptr;
    std::shared_ptr<ImagePublisher> left_disparity_pub_ = nullptr;
    std::shared_ptr<ImagePublisher> left_disparity_cost_pub_ = nullptr;

    rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr left_stereo_disparity_pub_ = nullptr;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr luma_point_cloud_pub_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr color_point_cloud_pub_ = nullptr;

    rclcpp::Publisher<multisense_msgs::msg::Histogram>::SharedPtr histogram_pub_ = nullptr;
    rclcpp::Publisher<multisense_msgs::msg::Info>::SharedPtr info_pub_ = nullptr;
    rclcpp::Publisher<multisense_msgs::msg::Status>::SharedPtr status_pub_ = nullptr;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr config_pub_ = nullptr;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_ = nullptr;

    //
    // Store outgoing messages to avoid repeated allocations

    sensor_msgs::msg::Image         left_mono_image_{};
    sensor_msgs::msg::Image         right_mono_image_{};
    sensor_msgs::msg::Image         left_rect_image_{};
    sensor_msgs::msg::Image         right_rect_image_{};
    sensor_msgs::msg::Image         depth_image_{};
    sensor_msgs::msg::Image         ni_depth_image_{};
    sensor_msgs::msg::PointCloud2   point_cloud_{};
    sensor_msgs::msg::PointCloud2   luma_point_cloud_{};
    sensor_msgs::msg::PointCloud2   color_point_cloud_{};
    sensor_msgs::msg::PointCloud2   luma_organized_point_cloud_{};
    sensor_msgs::msg::PointCloud2   color_organized_point_cloud_{};

    sensor_msgs::msg::Image         aux_mono_image_{};
    sensor_msgs::msg::Image         aux_rgb_image_{};
    sensor_msgs::msg::Image         aux_rect_image_{};
    sensor_msgs::msg::Image         aux_rect_rgb_image_{};

    sensor_msgs::msg::Image         left_disparity_image_{};
    sensor_msgs::msg::Image         left_disparity_cost_image_{};
    sensor_msgs::msg::Image         right_disparity_image_{};

    stereo_msgs::msg::DisparityImage left_stereo_disparity_{};

    multisense::MultiSenseInfo info_{};
    multisense::MultiSenseConfig current_config_{};

    //
    // The frame IDs

    const std::string frame_id_left_{};
    const std::string frame_id_right_{};
    const std::string frame_id_aux_{};
    const std::string frame_id_rectified_left_{};
    const std::string frame_id_rectified_right_{};
    const std::string frame_id_rectified_aux_{};
    const std::string frame_id_imu_{};

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    //
    // Used to signal threads waiting to process images

    FrameNotifier<multisense::ImageFrame> image_frame_notifier_{};

    //
    // Used to signal threads waiting to process imu data

    FrameNotifier<multisense::ImuFrame> imu_frame_notifier_{};

    //
    // Processing threads which are used to convert MultiSense types to ROS messages and publish them

    std::vector<std::thread> procesing_threads_{};

    //
    // For pointcloud generation

    double pointcloud_max_range_ = 50.0;

    //
    // Active streams

    std::mutex stream_mutex_{};
    std::map<multisense::DataSource, int> active_streams_{};
    std::unordered_map<std::string, int> active_topics_{};

    //
    // Has a 3rd aux color camera

    bool has_aux_camera_ = false;
    bool aux_control_supported_ = false;

    //
    // Handle stamping messages with different time sources

    std::atomic<TimestampSource> timestamp_source_ {TimestampSource::SYSTEM};

    size_t time_offset_buffer_size_ = 8;
    std::optional<std::chrono::nanoseconds> camera_host_time_offset_{std::nullopt};
};

}// namespace
