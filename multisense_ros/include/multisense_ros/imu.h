/**
 * @file imu.h
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

#ifndef MULTISENSE_ROS_IMU_H
#define MULTISENSE_ROS_IMU_H

#include <optional>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <multisense_msgs/msg/raw_imu_data.hpp>
#include <MultiSense/MultiSenseChannel.hh>

namespace multisense_ros {

class Imu : public rclcpp::Node
{
public:

    Imu(const std::string& node_name,
        const rclcpp::NodeOptions& options,
        crl::multisense::Channel* driver,
        const std::string& tf_prefix);

    ~Imu();

    void imuCallback(const crl::multisense::imu::Header& header);

private:
    static constexpr char RAW_ACCEL_TOPIC[] = "accelerometer";
    static constexpr char RAW_GYRO_TOPIC[] = "gyroscope";
    static constexpr char RAW_MAG_TOPIC[] = "magnetometer";
    static constexpr char IMU_TOPIC[] = "imu_data";
    static constexpr char ACCEL_VECTOR_TOPIC[] = "accelerometer_vector";
    static constexpr char GYRO_VECTOR_TOPIC[] = "gyroscope_vector";
    static constexpr char MAG_VECTOR_TOPIC[] = "magnetometer_vector";

    //
    // CRL sensor API

    crl::multisense::Channel* driver_;

    //
    // ROS2 timer for checking publisher status

    rclcpp::TimerBase::SharedPtr timer_;

    //
    // multisense_ros/RawImuData publishers

    rclcpp::Publisher<multisense_msgs::msg::RawImuData>::SharedPtr accelerometer_pub_;
    rclcpp::Publisher<multisense_msgs::msg::RawImuData>::SharedPtr gyroscope_pub_;
    rclcpp::Publisher<multisense_msgs::msg::RawImuData>::SharedPtr magnetometer_pub_;

    //
    // sensor_msgs/Imu publisher

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    //
    // geometry_msgs/Vector3Stamped publishers

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr accelerometer_vector_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gyroscope_vector_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr magnetometer_vector_pub_;

    //
    // Imu message to publish. Non-dynamic data (i.e. covariance values) are cached here

    sensor_msgs::msg::Imu imu_message_;

    //
    // Helper function to setup the nodes configuration parameters. In ROS2 this takes the place of dynamic
    // reconfigure

    void initalizeParameters();

    //
    // Get the total subscribers on all the IMU topics

    size_t getNumSubscribers();

    //
    // Callback to check subscriptions to our publishers. This duplicates the behavior of the ROS1
    // SubscriberStatusCallback in a much less elegant manner. Until that functionality is added to ROS2 we will
    // poll to enable our lazy publishing scheme.

    void timerCallback();

    //
    // TF frame ID's

    std::string accel_frameId_;
    std::string gyro_frameId_;
    std::string mag_frameId_;

    //
    // Parameter management

    OnSetParametersCallbackHandle::SharedPtr paramter_handle_;

    rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter>& parameters);

    //
    // IMU configuration

    uint32_t imu_samples_per_message_;
    std::optional<crl::multisense::imu::Config> accelerometer_config_;
    std::optional<crl::multisense::imu::Config> gyroscope_config_;
    std::optional<crl::multisense::imu::Config> magnetometer_config_;

    //
    // Active streams

    crl::multisense::DataSource active_streams_;

    bool next_gen_camera_;
};

}

#endif
