/**
 * @file thermal_publisher.h
 *
 * Copyright 2026
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * All rights reserved.
 **/

#pragma once

#include <array>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <MultiSense/MultiSenseChannel.hh>
#include <MultiSense/MultiSenseSecondaryApplication.hh>

#include "multisense_ros/publisher_utilities.h"
#include <multisense_ros/thermal_parameters.hpp>

namespace multisense_ros
{

///
/// @brief Publishes the images produced by the STT6 thermal secondary application.
///
/// This class deliberately owns all thermal-specific parameters, initialization,
/// decoding, and ROS publishers so the stereo driver does not need to model six
/// additional cameras directly.
///
class ThermalPublisher : public std::enable_shared_from_this<ThermalPublisher>
{
public:
    using FrameGroup = multisense::secondary_application::thermal::FrameGroup;
    using PublisherOptionsFactory = std::function<rclcpp::PublisherOptions(
        const std::vector<multisense::DataSource> &, const std::string &)>;
    using TimestampFactory = std::function<std::optional<rclcpp::Time>(const FrameGroup &)>;

    static std::shared_ptr<ThermalPublisher> create(
        rclcpp::Node *node,
        multisense::Channel &channel,
        const std::string &tf_prefix,
        const rclcpp::QoS &qos,
        bool use_image_transport,
        PublisherOptionsFactory publisher_options_factory,
        TimestampFactory timestamp_factory);

    ~ThermalPublisher();

    void shutdown();

    ThermalPublisher(const ThermalPublisher &) = delete;
    ThermalPublisher &operator=(const ThermalPublisher &) = delete;

private:
    static constexpr size_t MAX_IMAGERS = 6;

    struct ImagerPublisher
    {
        std::shared_ptr<ImagePublisher> publisher;
        sensor_msgs::msg::CameraInfo camera_info;
        sensor_msgs::msg::Image image;
        std::string frame_id;
    };

    ThermalPublisher(rclcpp::Node *node,
                     multisense::Channel &channel,
                     std::string tf_prefix,
                     rclcpp::QoS qos,
                     bool use_image_transport,
                     PublisherOptionsFactory publisher_options_factory,
                     TimestampFactory timestamp_factory);

    void initialize();
    void handle_packet(const multisense::SecondaryApplicationData &packet);
    void run();

    rclcpp::Node *node_;
    multisense::Channel &channel_;
    std::string tf_prefix_;
    rclcpp::QoS qos_;
    bool use_image_transport_;
    PublisherOptionsFactory publisher_options_factory_;
    TimestampFactory timestamp_factory_;

    std::shared_ptr<multisense_thermal::ParamListener> parameter_listener_;
    std::array<std::optional<ImagerPublisher>, MAX_IMAGERS> imagers_{};

    std::mutex frame_mutex_;
    std::condition_variable frame_condition_;
    std::optional<FrameGroup> pending_frame_;
    bool shutdown_ = false;
    bool stopped_ = false;
    std::thread processing_thread_;
};

} // namespace multisense_ros
