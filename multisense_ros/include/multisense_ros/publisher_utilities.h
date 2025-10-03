/**
 * @file publisher_utilities.h
 *
 * Copyright 2025
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * https://www.carnegierobotics.com
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

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace multisense_ros {

std::string get_full_topic_name(const rclcpp::Node::SharedPtr node, const std::string &topic_name);

///
/// @brief A custom Image publisher which can optionally publish via image transport or standard rclcpp
///        image publishers. Publishing of CameraInfo is also managed
///
class ImagePublisher
{
public:
    ImagePublisher(rclcpp::Node::SharedPtr node,
                   const std::string &topic_name,
                   const sensor_msgs::msg::CameraInfo &initial_camera_info,
                   const rclcpp::QoS &qos,
                   const rclcpp::PublisherOptions &options,
                   bool image_transport)
    {
        if (image_transport)
        {
            image_transport_pub_ =
                std::make_unique<image_transport::Publisher>(image_transport::create_publisher(node.get(),
                                                             get_full_topic_name(node, topic_name),
                                                             qos.get_rmw_qos_profile(),
                                                             options));
        }
        else
        {
            image_pub_ = node->create_publisher<sensor_msgs::msg::Image>(topic_name, qos, options);
        }

        const auto camera_info_qos = rclcpp::QoS(1).transient_local();

        camera_info_pub_ = node->create_publisher<sensor_msgs::msg::CameraInfo>(topic_name + "/camera_info", camera_info_qos);

        camera_info_pub_->publish(std::make_unique<sensor_msgs::msg::CameraInfo>(initial_camera_info));
    }

    ~ImagePublisher() = default;

    void publish(std::unique_ptr<sensor_msgs::msg::Image> image,
                 std::unique_ptr<sensor_msgs::msg::CameraInfo> camera_info)
    {
        if (image_transport_pub_)
        {
            image_transport_pub_->publish(std::move(image));
        }
        else if (image_pub_)
        {
            image_pub_->publish(std::move(image));
        }

        camera_info_pub_->publish(std::move(camera_info));
    }

private:

    std::unique_ptr<image_transport::Publisher> image_transport_pub_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_ = nullptr;
};

}
