/**
 * @file ros_driver.cpp
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

#include <rclcpp/rclcpp.hpp>

#include <multisense_ros/camera.h>

#include <MultiSense/MultiSenseChannel.hh>

namespace lms = multisense;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto initialize_node = std::make_shared<rclcpp::Node>("multisense_initialization");

    rcl_interfaces::msg::ParameterDescriptor ip_description;
    ip_description.set__name("sensor_ip")
                  .set__read_only(true)
                  .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
                  .set__description("multisense ip address");
    initialize_node->declare_parameter("sensor_ip", "10.66.171.21", ip_description);

    rcl_interfaces::msg::ParameterDescriptor mtu_description;
    mtu_description.set__name("sensor_mtu")
                   .set__read_only(true)
                   .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                   .set__description("multisense mtu");
    initialize_node->declare_parameter("sensor_mtu", 1500, mtu_description);

    rcl_interfaces::msg::ParameterDescriptor tf_prefix_description;
    tf_prefix_description.set__name("tf_prefix")
                         .set__read_only(true)
                         .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
                         .set__description("tf2 transform prefix");
    initialize_node->declare_parameter("tf_prefix", "multisense", tf_prefix_description);

    rclcpp::Parameter sensor_ip;
    rclcpp::Parameter sensor_mtu;
    rclcpp::Parameter tf_prefix;

    if(!initialize_node->get_parameter("sensor_ip", sensor_ip))
        RCLCPP_ERROR(initialize_node->get_logger(), "multisense_ros: sensor ip address not specified");

    if(!initialize_node->get_parameter("sensor_mtu", sensor_mtu))
        RCLCPP_ERROR(initialize_node->get_logger(), "multisense_ros: sensor mtu not specified");

    if(!initialize_node->get_parameter("tf_prefix", tf_prefix))
        RCLCPP_ERROR(initialize_node->get_logger(), "multisense_ros: tf prefix not specified");

    try
    {
        auto channel = lms::Channel::create(lms::Channel::Config{sensor_ip.as_string(), sensor_mtu.as_int()});
	    if (nullptr == channel)
        {
            RCLCPP_ERROR(initialize_node->get_logger(), "multisense_ros: failed to create communication channel to sensor @ \"%s\"",
                         sensor_ip.as_string().c_str());
            return EXIT_FAILURE;
        }

        rclcpp::spin(std::make_shared<multisense_ros::Camera>("camera", rclcpp::NodeOptions{}, std::move(channel), tf_prefix.as_string()));
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(initialize_node->get_logger(), "multisense_ros: caught exception: %s", e.what());
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
