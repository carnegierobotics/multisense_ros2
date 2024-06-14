/**
 * @file status.cpp
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

#include <chrono>

#include <multisense_ros/status.h>

using namespace std::chrono_literals;

namespace multisense_ros {

Status::Status(const std::string& node_name, crl::multisense::Channel* driver):
    Node(node_name),
    driver_(driver)
{
    if (nullptr == driver_)
    {
        return;
    }

    crl::multisense::system::VersionInfo version_info;
    if (const auto status = driver_->getVersionInfo(version_info); status != crl::multisense::Status_Ok )
    {
        RCLCPP_ERROR(get_logger(), "Status: failed to query version info: %s", crl::multisense::Channel::statusString(status));
        return;
    }

    status_pub_ = create_publisher<multisense_msgs::msg::DeviceStatus>(STATUS_TOPIC, rclcpp::SensorDataQoS());

    if (version_info.sensorFirmwareVersion >= 0x060A)
    {
        ptp_status_supported_ = true;
        ptp_status_pub_ = create_publisher<multisense_msgs::msg::PtpStatus>(PTP_STATUS_TOPIC, rclcpp::SensorDataQoS());
    }

    timer_ = create_wall_timer(500ms, std::bind(&Status::queryStatus, this));
}

Status::~Status()
{
}

void Status::queryStatus()
{
    if (count_subscribers(STATUS_TOPIC) > 0)
    {
        crl::multisense::system::StatusMessage statusMessage;

        if (const auto status = driver_->getDeviceStatus(statusMessage) != crl::multisense::Status_Ok)
        {
            RCLCPP_WARN(get_logger(), "Status: failed to query status: %s", crl::multisense::Channel::statusString(status));
        }
        else
        {
            multisense_msgs::msg::DeviceStatus deviceStatus;

            std::chrono::nanoseconds uptime(static_cast<int64_t>(statusMessage.uptime * 1e9));

            deviceStatus.time_stamp = rclcpp::Clock().now();
            deviceStatus.uptime = rclcpp::Time(uptime.count());
            deviceStatus.system_ok = statusMessage.systemOk;
            deviceStatus.laser_ok = statusMessage.laserOk;
            deviceStatus.laser_motor_ok = statusMessage.laserMotorOk;
            deviceStatus.cameras_ok = statusMessage.camerasOk;
            deviceStatus.imu_ok = statusMessage.imuOk;
            deviceStatus.external_leds_ok = statusMessage.externalLedsOk;
            deviceStatus.processing_pipeline_ok = statusMessage.processingPipelineOk;
            deviceStatus.power_supply_temp = statusMessage.powerSupplyTemperature;
            deviceStatus.fpga_temp = statusMessage.fpgaTemperature;
            deviceStatus.left_imager_temp = statusMessage.leftImagerTemperature;
            deviceStatus.right_imager_temp = statusMessage.rightImagerTemperature;
            deviceStatus.input_voltage = statusMessage.inputVoltage;
            deviceStatus.input_current = statusMessage.inputCurrent;
            deviceStatus.fpga_power = statusMessage.fpgaPower;
            deviceStatus.logic_power = statusMessage.logicPower;
            deviceStatus.imager_power = statusMessage.imagerPower;

            status_pub_->publish(deviceStatus);
        }
    }

    if (ptp_status_supported_ && count_subscribers(PTP_STATUS_TOPIC) > 0)
    {
        crl::multisense::system::PtpStatus ptpStatusMessage;

        if (crl::multisense::Status_Ok == driver_->getPtpStatus(ptpStatusMessage))
        {
            multisense_msgs::msg::PtpStatus ptpStatus;

            ptpStatus.gm_present = ptpStatusMessage.gm_present != 0;
            memcpy(ptpStatus.gm_id.data(), ptpStatusMessage.gm_id, ptpStatus.gm_id.size());
            ptpStatus.gm_offset = ptpStatusMessage.gm_offset;
            ptpStatus.path_delay = ptpStatusMessage.path_delay;
            ptpStatus.steps_removed = ptpStatusMessage.steps_removed;

            ptp_status_pub_->publish(ptpStatus);
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Status: failed to query ptp status");
        }
    }
}

}
