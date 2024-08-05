/**
 * @file config.cpp
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

#include <multisense_ros/config.h>
#include <multisense_ros/parameter_utilities.h>

using namespace crl::multisense;

namespace multisense_ros {

Config::Config(const std::string& node_name, crl::multisense::Channel* driver):
    Node(node_name),
    driver_(driver)
{
    if (const auto status = driver_->getDeviceInfo(device_info_); status != crl::multisense::Status_Ok)
    {
        RCLCPP_ERROR(get_logger(), "Config: failed to query device info: %s",
                     crl::multisense::Channel::statusString(status));
        return;
    }

    lighting_supported_ = device_info_.lightingType != 0 ||
                          device_info_.hardwareRevision == system::DeviceInfo::HARDWARE_REV_MULTISENSE_KS21;

    if (lighting_supported_)
    {
        if (const auto status = driver_->getLightingConfig(lighting_config_); status != crl::multisense::Status_Ok)
        {
            RCLCPP_ERROR(get_logger(), "Config: failed to query lighting info: %s",
                         crl::multisense::Channel::statusString(status));
            return;
        }
    }

    ptp_supported_ = (device_info_.hardwareRevision == system::DeviceInfo::HARDWARE_REV_MULTISENSE_C6S2_S27 ||
                      device_info_.hardwareRevision == system::DeviceInfo::HARDWARE_REV_MULTISENSE_S30 ||
                      device_info_.hardwareRevision == system::DeviceInfo::HARDWARE_REV_MULTISENSE_KS21 ||
                      device_info_.hardwareRevision == system::DeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_VPB ||
                      device_info_.hardwareRevision == system::DeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_STEREO ||
                      device_info_.hardwareRevision == system::DeviceInfo::HARDWARE_REV_MULTISENSE_REMOTE_HEAD_MONOCAM);

    if (const auto status = driver_->getTransmitDelay(transmit_delay_); status != crl::multisense::Status_Ok)
    {
            RCLCPP_ERROR(get_logger(), "Config: failed to query transmit delay: %s",
                         crl::multisense::Channel::statusString(status));
            return;
    }

    paramter_handle_ = add_on_set_parameters_callback(std::bind(&Config::parameterCallback, this, std::placeholders::_1));

    //
    // Transmit delay

    rcl_interfaces::msg::IntegerRange transmit_delay_range;
    transmit_delay_range.set__from_value(0)
                        .set__to_value(500)
                        .set__step(1);

    rcl_interfaces::msg::ParameterDescriptor transmit_delay_desc;
    transmit_delay_desc.set__name("desired_transmit_delay")
                       .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                       .set__description("delay in ms before multisense sends data")
                       .set__integer_range({transmit_delay_range});
    declare_parameter("desired_transmit_delay", 0, transmit_delay_desc);

    if (device_info_.hardwareRevision != crl::multisense::system::DeviceInfo::HARDWARE_REV_MULTISENSE_ST21)
    {
        if (lighting_supported_)
        {
            //
            // Lighting

            rcl_interfaces::msg::ParameterDescriptor lighting_desc;
            lighting_desc.set__name("lighting")
                         .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
                         .set__description("enable external lights");
            declare_parameter("lighting", false, lighting_desc);

            //
            // Flash

            rcl_interfaces::msg::ParameterDescriptor flash_desc;
            flash_desc.set__name("flash")
                         .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
                         .set__description("strobe lights with each image exposure");
            declare_parameter("flash", false, flash_desc);

            //
            // LED duty cycle

            rcl_interfaces::msg::FloatingPointRange led_duty_cycle_range;
            led_duty_cycle_range.set__from_value(0.0)
                                .set__to_value(1.0)
                                .set__step(0.01);

            rcl_interfaces::msg::ParameterDescriptor led_duty_cycle_desc;
            led_duty_cycle_desc.set__name("led_duty_cycle")
                               .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                               .set__description("LED duty cycle when lights enabled")
                               .set__floating_point_range({led_duty_cycle_range});
            declare_parameter("led_duty_cycle", 0.0, led_duty_cycle_desc);
        }

        //
        // Network time sync

        rcl_interfaces::msg::ParameterDescriptor network_time_sync_desc;
        network_time_sync_desc.set__name("network_time_sync")
                              .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
                              .set__description("run basic time sync between MultiSense and host");
        declare_parameter("network_time_sync", true, network_time_sync_desc);

        if (ptp_supported_)
        {
            //
            // PTP time sync

            rcl_interfaces::msg::ParameterDescriptor ptp_time_sync_desc;
            ptp_time_sync_desc.set__name("ptp_time_sync")
                                  .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
                                  .set__description("run basic time sync between MultiSense and host");
            declare_parameter("ptp_time_sync", false, ptp_time_sync_desc);
        }
    }

}

Config::~Config()
{
}

rcl_interfaces::msg::SetParametersResult Config::parameterCallback(const std::vector<rclcpp::Parameter>&  parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.set__successful(true);

    bool update_lighting = false;

    for (const auto &parameter : parameters)
    {
        const auto type = parameter.get_type();
        if (type == rclcpp::ParameterType::PARAMETER_NOT_SET)
        {
            continue;
        }

        const auto name = parameter.get_name();

        if (name == "desired_transmit_delay")
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid transmission delay type");
            }

            const auto value = get_as_number<int>(parameter);
            if (transmit_delay_.delay != value)
            {
                auto delay = transmit_delay_;
                delay.delay = value;
                if (const auto status = driver_->setTransmitDelay(delay); status != crl::multisense::Status_Ok)
                {
                    return result.set__successful(false).set__reason(crl::multisense::Channel::statusString(status));
                }

                transmit_delay_ = std::move(delay);
            }
        }
        else if (name == "lighting")
        {
            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
            {
                return result.set__successful(false).set__reason("invalid lighting type");
            }

            const auto value = parameter.as_bool();
            if (lighting_enabled_ != value)
            {
                lighting_enabled_ = value;
                update_lighting = true;
            }
        }
        else if(name == "flash")
        {
            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
            {
                return result.set__successful(false).set__reason("invalid flash type");
            }

            const auto value = parameter.as_bool();
            if (lighting_config_.getFlash() != value)
            {
                lighting_config_.setFlash(value);
                update_lighting = true;
            }
        }
        else if(name == "led_duty_cycle")
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid led duty cycle type");
            }

            const auto value = get_as_number<double>(parameter);
            if (lighting_config_.getDutyCycle(0) != value)
            {
                lighting_config_.setDutyCycle(value);
                update_lighting = true;
            }
        }
        else if(name == "network_time_sync")
        {
            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
            {
                return result.set__successful(false).set__reason("invalid network time sync type");
            }

            if (const auto status = driver_->networkTimeSynchronization(parameter.as_bool());
                    status != crl::multisense::Status_Ok)
            {
                return result.set__successful(false).set__reason(crl::multisense::Channel::statusString(status));
            }
        }
        else if(name == "ptp_time_sync")
        {
            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
            {
                return result.set__successful(false).set__reason("invalid ptp time sync type");
            }

            if (!ptp_supported_)
            {
                return result.set__successful(false).set__reason("ptp time sync not supported");
            }

            if (const auto status = driver_->ptpTimeSynchronization(parameter.as_bool());
                    status != crl::multisense::Status_Ok)
            {
                ptp_enabled_ = parameter.as_bool();
                return result.set__successful(false).set__reason(crl::multisense::Channel::statusString(status));
            }
        }
    }

    if (update_lighting)
    {
        auto config = lighting_config_;

        config.setFlash(lighting_enabled_ ? config.getFlash() : false);
        config.setDutyCycle(lighting_enabled_ ? config.getDutyCycle(0) : 0.0);

        if (const auto status = driver_->setLightingConfig(config); status != crl::multisense::Status_Ok)
        {
            if (status == crl::multisense::Status_Unsupported)
            {
                return result.set__successful(false).set__reason("lighting not supported");
            }
            else
            {
                return result.set__successful(false).set__reason(crl::multisense::Channel::statusString(status));
            }
        }
    }

    return result;
}

}
