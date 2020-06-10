/**
 * @file pps.cpp
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


#include <multisense_ros/pps.h>

using namespace crl::multisense;

namespace multisense_ros {

namespace { // anonymous

//
// Shim for C-style driver callbacks

void ppsCB(const pps::Header& header, void* userDataP)
{ reinterpret_cast<Pps*>(userDataP)->ppsCallback(header); }


} // anonymous

Pps::Pps(const std::string& node_name, Channel* driver) :
    Node(node_name),
    driver_(driver)
{
    pps_pub_ = create_publisher<builtin_interfaces::msg::Time>(PPS_TOPIC, rclcpp::SensorDataQoS());
    stamped_pps_pub_ = create_publisher<multisense_msgs::msg::StampedPps>(STAMPED_PPS_TOPIC, rclcpp::SensorDataQoS());

    driver_->addIsolatedCallback(ppsCB, this);
}

Pps::~Pps()
{
    driver_->removeIsolatedCallback(ppsCB);
}

void Pps::ppsCallback(const pps::Header& header)
{
    if (count_subscribers(PPS_TOPIC) <= 0 && count_subscribers(STAMPED_PPS_TOPIC) <= 0)
        return;

    multisense_msgs::msg::StampedPps stamped_pps_msg;

    builtin_interfaces::msg::Time pps_msg = rclcpp::Time(header.sensorTime / 1000000000ll,
                                                         pps_msg.nanosec = header.sensorTime % 1000000000ll);

    stamped_pps_msg.host_time = rclcpp::Time(header.timeSeconds, 1000 * header.timeMicroSeconds);
    stamped_pps_msg.data = pps_msg;

    pps_pub_->publish(pps_msg);

    stamped_pps_pub_->publish(stamped_pps_msg);
}

} // namespace
