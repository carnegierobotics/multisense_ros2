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

#include <multisense_ros/multisense.h>
#include <multisense_ros/multisense_init_parameters.hpp>

#include <MultiSense/MultiSenseChannel.hh>

namespace lms = multisense;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto initialize_node = std::make_shared<rclcpp::Node>("multisense_initialization");

    auto param_listener = std::make_shared<initialization::ParamListener>(initialize_node);
    auto params = param_listener->get_params();

    const std::string sensor_ip = params.sensor_ip;

    try
    {
        auto channel = lms::Channel::create(lms::Channel::Config{sensor_ip, params.sensor_mtu});
	    if (nullptr == channel)
        {
            RCLCPP_ERROR(initialize_node->get_logger(), "multisense_ros: failed to create communication channel to sensor @ \"%s\"",
                         sensor_ip.c_str());
            return EXIT_FAILURE;
        }

        auto sensor = std::make_shared<multisense_ros::MultiSense>("sensor",
                                                                   rclcpp::NodeOptions{},
                                                                   std::move(channel),
                                                                   params.tf_prefix,
                                                                   params.use_image_transport,
                                                                   params.use_sensor_qos);

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(sensor);
        executor.add_node(initialize_node);
        executor.spin();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(initialize_node->get_logger(), "multisense_ros: caught exception: %s", e.what());
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
