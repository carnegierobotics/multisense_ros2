/**
 * @file laser.h
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

#ifndef MULTISENSE_ROS_LASER_H
#define MULTISENSE_ROS_LASER_H

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <MultiSense/MultiSenseChannel.hh>
#include <multisense_msgs/msg/raw_lidar_data.hpp>
#include <multisense_msgs/msg/raw_lidar_cal.hpp>


namespace multisense_ros {

class Laser : public rclcpp::Node
{
public:
    Laser(const std::string& node_name,
          const rclcpp::NodeOptions& options,
          crl::multisense::Channel* driver,
          const std::string& tf_prefix);

    ~Laser();

    void scanCallback(const crl::multisense::lidar::Header& header);
    void pointCloudCallback(const crl::multisense::lidar::Header& header);


private:
    static constexpr char LASER_SCAN_TOPIC[] = "lidar_scan";
    static constexpr char LASER_POINTCLOUD_TOPIC[] = "lidar_points2";
    static constexpr char RAW_LASER_CAL_TOPIC[] = "calibration/raw_lidar_cal";
    static constexpr char RAW_LASER_DATA_TOPIC[] = "calibration/raw_lidar_data";
    static constexpr char JOINT_STATE_TOPIC[] = "joint_states";

    //
    // Device stream control

    crl::multisense::Channel *driver_;
    void stop();

    //
    // Transform boadcasting

    void publishSpindleTransform(const float spindle_angle, const float velocity, const rclcpp::Time& time);

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    void tfPublisher();

    //
    // Callback to check subscriptions to our publishers. This duplicates the behavior of the ROS1
    // SubscriberStatusCallback in a much less elegant manner. Untill that functionality is added to ROS2 we will
    // poll to enable our lazy publishing scheme.

    void streamControl();

    //
    // Determine how many subscribers we have to our laser topics

    size_t getNumSubscribers();

    //
    // Query transforms

    tf2::Transform getSpindleTransform(float spindle_angle);

    //
    // Calibration from sensor

    crl::multisense::lidar::Calibration lidar_cal_;

    tf2::Transform motor_to_camera_;
    tf2::Transform laser_to_spindle_;

    //
    // Frames to Publish

    std::string frame_id_;
    std::string left_camera_optical_;
    std::string motor_;
    std::string spindle_;
    std::string hokuyo_;

    //
    // Scan publishing

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

    //
    // Raw data publishing

    rclcpp::Publisher<multisense_msgs::msg::RawLidarData>::SharedPtr raw_lidar_data_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Publisher<multisense_msgs::msg::RawLidarCal>::SharedPtr raw_lidar_cal_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;

    //
    // Keep around for efficiency

    sensor_msgs::msg::LaserScan   laser_msg_;
    sensor_msgs::msg::PointCloud2 point_cloud_;
    sensor_msgs::msg::JointState  joint_states_;

    //
    // Timer used to publish the default laser transforms

    rclcpp::TimerBase::SharedPtr timer_;

    //
    // ROS2 timer for checking publisher status

    rclcpp::TimerBase::SharedPtr publisher_timer_;

    //
    // Spindle angle used when publishing the default transforms

    double spindle_angle_;

    //
    // Track publishing rates

    rclcpp::Time previous_scan_time_;

    //
    // Parameter management

    OnSetParametersCallbackHandle::SharedPtr paramter_handle_;

    rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter>&  parameters);

    //
    // Active streams

    crl::multisense::DataSource active_streams_;

}; // class

}// namespace


#endif
