/**
 * @file imu.cpp
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
#include <cmath>

#include <multisense_ros/imu.h>
#include <multisense_ros/parameter_utilities.h>

using namespace crl::multisense;
using namespace std::chrono_literals;

namespace multisense_ros {

namespace { // anonymous

//
// Shim for C-style driver callbacks

void imuCB(const imu::Header& header, void* userDataP)
{ reinterpret_cast<Imu*>(userDataP)->imuCallback(header); }


} // anonymous

Imu::Imu(const std::string& node_name,
         const rclcpp::NodeOptions& options,
         Channel* driver,
         const std::string& tf_prefix) :
    Node(node_name, options),
    driver_(driver),
    accelerometer_pub_(nullptr),
    gyroscope_pub_(nullptr),
    magnetometer_pub_(nullptr),
    accelerometer_vector_pub_(nullptr),
    gyroscope_vector_pub_(nullptr),
    magnetometer_vector_pub_(nullptr),
    accel_frameId_(tf_prefix + "/accel"),
    gyro_frameId_(tf_prefix + "/gyro"),
    mag_frameId_(tf_prefix + "/mag"),
    imu_samples_per_message_(30),
    active_streams_(Source_Unknown),
    next_gen_camera_(false)
{
    //
    // Get version info

    system::VersionInfo  version_info;
    if (const auto status = driver_->getVersionInfo(version_info); status != Status_Ok)
    {
        RCLCPP_ERROR(get_logger(), "IMU: failed to query version info: %s",
                  Channel::statusString(status));
        return;
    }

    //
    // All cameras running firmware greater than version 4.3 are next gen cameras

    next_gen_camera_ = version_info.sensorFirmwareVersion > 0x0403;

    if (next_gen_camera_)
    {
        accel_frameId_ = tf_prefix + "/imu";
        gyro_frameId_ = tf_prefix + "/imu";
    }

    //
    // Initialize the sensor_msgs::Imu topic
    // We will publish the data in the accelerometer frame applying the
    // transform from the /gyro to the /accel frame to the gyroscope data

    imu_message_.header.frame_id = accel_frameId_;

    //
    // Covariance matrix for linear acceleration and angular velocity were
    // generated using 2 minutes of logged data with the default imu
    // settings. Note the angular velocity covariance has the nominal gyro
    // to accelerometer transform applied to it

    imu_message_.linear_acceleration_covariance[0] = 6.98179077e-04;
    imu_message_.linear_acceleration_covariance[1] = 2.46789341e-06;
    imu_message_.linear_acceleration_covariance[2] =-2.50549745e-06;
    imu_message_.linear_acceleration_covariance[3] = 2.46789341e-06;
    imu_message_.linear_acceleration_covariance[4] = 5.02177646e-04;
    imu_message_.linear_acceleration_covariance[5] = 5.26265558e-05;
    imu_message_.linear_acceleration_covariance[6] =-2.50549745e-06;
    imu_message_.linear_acceleration_covariance[7] = 5.26265558e-05;
    imu_message_.linear_acceleration_covariance[8] = 9.22796937e-04;

    imu_message_.angular_velocity_covariance[0] = 8.79376936e-06;
    imu_message_.angular_velocity_covariance[1] = -3.56007627e-07;
    imu_message_.angular_velocity_covariance[2] = 2.22611968e-07;
    imu_message_.angular_velocity_covariance[3] = -3.56007627e-07;
    imu_message_.angular_velocity_covariance[4] = 8.78939245e-06;
    imu_message_.angular_velocity_covariance[5] = -8.08367486e-07;
    imu_message_.angular_velocity_covariance[6] = 1.33981577e-05;
    imu_message_.angular_velocity_covariance[7] = 2.22611968e-07;
    imu_message_.angular_velocity_covariance[8] = -8.08367486e-07;

    std::vector<imu::Config> imu_configs;
    if (const auto status = driver_->getImuConfig(imu_samples_per_message_, imu_configs); status != Status_Ok)
    {
        RCLCPP_ERROR(get_logger(), "IMU: failed to query IMU config: %s", Channel::statusString(status));
        return;
    }

    if (imu_configs.empty())
    {
        RCLCPP_INFO(get_logger(), "IMU: hardware does not support a IMU");
        return;
    }

    auto accel_config = std::find_if(std::begin(imu_configs), std::end(imu_configs),
                                     [](const imu::Config &c) { return c.name == "accelerometer"; });
    accelerometer_config_ = accel_config == std::end(imu_configs) ? std::nullopt : std::make_optional(*accel_config);

    auto gyro_config = std::find_if(std::begin(imu_configs), std::end(imu_configs),
                                    [](const imu::Config &c) { return c.name == "gyroscope"; });
    gyroscope_config_ = gyro_config == std::end(imu_configs) ? std::nullopt : std::make_optional(*gyro_config);

    driver_->stopStreams(Source_Imu);

    accelerometer_pub_ = create_publisher<multisense_msgs::msg::RawImuData>(RAW_ACCEL_TOPIC, rclcpp::SensorDataQoS());
    gyroscope_pub_ = create_publisher<multisense_msgs::msg::RawImuData>(RAW_GYRO_TOPIC, rclcpp::SensorDataQoS());

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(IMU_TOPIC, rclcpp::SensorDataQoS());

    accelerometer_vector_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(ACCEL_VECTOR_TOPIC, rclcpp::SensorDataQoS());
    gyroscope_vector_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(GYRO_VECTOR_TOPIC, rclcpp::SensorDataQoS());

    if (!next_gen_camera_)
    {
        auto mag_config = std::find_if(std::begin(imu_configs), std::end(imu_configs),
                                       [](const imu::Config &c) { return c.name == "magnetometer"; });
        magnetometer_config_ = mag_config == std::end(imu_configs) ? std::nullopt : std::make_optional(*mag_config);

        magnetometer_pub_ = create_publisher<multisense_msgs::msg::RawImuData>(RAW_MAG_TOPIC, rclcpp::SensorDataQoS());

        magnetometer_vector_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(MAG_VECTOR_TOPIC, rclcpp::SensorDataQoS());
    }

    driver_->addIsolatedCallback(imuCB, this);

    timer_ = create_wall_timer(500ms, std::bind(&Imu::timerCallback, this));

    paramter_handle_ = add_on_set_parameters_callback(std::bind(&Imu::parameterCallback, this, std::placeholders::_1));

    initalizeParameters();
}

Imu::~Imu()
{
    driver_->stopStreams(Source_Imu);
    driver_->removeIsolatedCallback(imuCB);
}

void Imu::imuCallback(const imu::Header& header)
{
    const size_t accel_subscribers = count_subscribers(RAW_ACCEL_TOPIC);
    const size_t gyro_subscribers = count_subscribers(RAW_GYRO_TOPIC);
    const size_t mag_subscribers = next_gen_camera_ ? 0 : count_subscribers(RAW_MAG_TOPIC);
    const size_t imu_subscribers = count_subscribers(IMU_TOPIC);
    const size_t accel_vector_subscribers = count_subscribers(ACCEL_VECTOR_TOPIC);
    const size_t gyro_vector_subscribers = count_subscribers(GYRO_VECTOR_TOPIC);
    const size_t mag_vector_subscribers = next_gen_camera_ ? 0 : count_subscribers(MAG_VECTOR_TOPIC);

    for (const auto& s : header.samples)
    {
        multisense_msgs::msg::RawImuData msg;
        geometry_msgs::msg::Vector3Stamped vector_msg;

        msg.time_stamp = rclcpp::Time(s.timeSeconds, 1000 * s.timeMicroSeconds);

        msg.x = s.x;
        msg.y = s.y;
        msg.z = s.z;

        vector_msg.header.stamp = msg.time_stamp;
        vector_msg.vector.x = s.x;
        vector_msg.vector.y = s.y;
        vector_msg.vector.z = s.z;

        //
        // There are cases where the accel and gyro data are published with the same timestamps. Instead of
        // publishing two IMU messages for each, publish a new IMU message once we know for sure we have all the
        // samples from the same timestamp

        const bool publish_previous_imu_message = (s.type == imu::Sample::Type_Accelerometer ||
                                                   s.type == imu::Sample::Type_Gyroscope) &&
                                                  imu_message_.header.stamp != msg.time_stamp;

        imu_message_.header.stamp = msg.time_stamp;

        if (publish_previous_imu_message && imu_subscribers > 0)
        {
            imu_pub_->publish(imu_message_);
        }

        switch(s.type)
        {
            case imu::Sample::Type_Accelerometer:
            {
                //
                // Convert from g to m/s^2

                imu_message_.linear_acceleration.x = s.x * 9.80665;
                imu_message_.linear_acceleration.y = s.y * 9.80665;
                imu_message_.linear_acceleration.z = s.z * 9.80665;


                if (accel_subscribers > 0 && accelerometer_pub_)
                {
                    accelerometer_pub_->publish(msg);
                }

                if (accel_vector_subscribers > 0 && accelerometer_vector_pub_)
                {
                    vector_msg.header.frame_id = accel_frameId_;
                    accelerometer_vector_pub_->publish(vector_msg);
                }

                break;
            }
            case imu::Sample::Type_Gyroscope:
            {
                //
                // Convert from deg/sec to rad/sec and apply the nominal
                // calibration from the gyro to the accelerometer. Since all points
                // on a rigid body have the same angular velocity only the rotation
                // about the z axis of 90 degrees needs to be applied. (i.e.
                // new_x = orig_y ; new_y = -orig_x). Note this only applies for older cameras where
                // the gyro and accelerometer are on independent ICs.

                if (next_gen_camera_)
                {
                    imu_message_.angular_velocity.x = s.x * M_PI/180.;
                    imu_message_.angular_velocity.y = s.y * M_PI/180.;
                    imu_message_.angular_velocity.z = s.z * M_PI/180.;
                }
                else
                {
                    imu_message_.angular_velocity.x = s.y * M_PI/180.;
                    imu_message_.angular_velocity.y = -s.x * M_PI/180.;
                    imu_message_.angular_velocity.z = s.z * M_PI/180.;
                }


                if (gyro_subscribers > 0 && gyroscope_pub_)
                {
                    gyroscope_pub_->publish(msg);
                }

                if (gyro_vector_subscribers > 0 && gyroscope_vector_pub_)
                {
                    vector_msg.header.frame_id = gyro_frameId_;
                    gyroscope_vector_pub_->publish(vector_msg);
                }

                break;
            }
            case imu::Sample::Type_Magnetometer:
            {
                if (!next_gen_camera_)
                {
                    if (mag_subscribers > 0 && magnetometer_pub_)
                    {
                        magnetometer_pub_->publish(msg);
                    }

                    if (mag_vector_subscribers > 0 && magnetometer_vector_pub_)
                    {
                        vector_msg.header.frame_id = mag_frameId_;
                        magnetometer_vector_pub_->publish(vector_msg);
                    }
                }

                break;
            }
        }
    }
}

size_t Imu::getNumSubscribers()
{
    return count_subscribers(RAW_ACCEL_TOPIC) +
           count_subscribers(RAW_GYRO_TOPIC) +
           count_subscribers(RAW_MAG_TOPIC) +
           count_subscribers(IMU_TOPIC) +
           count_subscribers(ACCEL_VECTOR_TOPIC) +
           count_subscribers(GYRO_VECTOR_TOPIC) +
           count_subscribers(MAG_VECTOR_TOPIC);
}

void Imu::timerCallback()
{
    const auto num_subscribers = getNumSubscribers();

    if (num_subscribers > 0 && (active_streams_ & Source_Imu) == 0)
    {
        if (const auto status = driver_->startStreams(Source_Imu); status != Status_Ok)
        {
            RCLCPP_ERROR(get_logger(), "IMU: failed to start streams: %s",
                      Channel::statusString(status));
            return;
        }

        active_streams_ = Source_Imu;
    }
    else if (num_subscribers <= 0 && (active_streams_ & Source_Imu) == Source_Imu)
    {
        if (const auto status = driver_->stopStreams(Source_Imu); status != Status_Ok)
        {
            RCLCPP_ERROR(get_logger(), "IMU: failed to stop streams: %s",
                      Channel::statusString(status));
            return;
        }

        active_streams_ = Source_Unknown;
    }
}

void Imu::initalizeParameters()
{
    //
    // Samples per message

    rcl_interfaces::msg::IntegerRange samples_per_message_range;
    samples_per_message_range.set__from_value(1)
                             .set__to_value(300)
                             .set__step(1);

    rcl_interfaces::msg::ParameterDescriptor samples_per_message_desc;
    samples_per_message_desc.set__name("imu_samples_per_message")
                            .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                            .set__description("imu samples per message")
                            .set__integer_range({samples_per_message_range});
    declare_parameter("imu_samples_per_message", static_cast<int>(imu_samples_per_message_), samples_per_message_desc);

    //
    // Accelerometer enable

    rcl_interfaces::msg::ParameterDescriptor accel_enabled_desc;
    accel_enabled_desc.set__name("accelerometer_enabled")
                      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
                      .set__description("enable multisense accelerometer");
    declare_parameter("accelerometer_enabled", true, accel_enabled_desc);

    //
    // Accelerometer rate

    rcl_interfaces::msg::IntegerRange accel_rate_range;
    accel_rate_range.set__from_value(0)
                    .set__to_value(6)
                    .set__step(1);

    rcl_interfaces::msg::ParameterDescriptor accel_rate_desc;
    accel_rate_desc.set__name("accelerometer_rate")
                      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                      .set__description("acelerometer rate\n0: 10Hz__1HzCutoff\n"
                                                            "1: 25Hz__3HzCutoff\n"
                                                            "2: 50Hz__6HzCutoff\n"
                                                            "3: 100Hz__11HzCutoff\n"
                                                            "4: 200Hz__22HzCutoff\n"
                                                            "5: 400Hz__44HzCutoff\n"
                                                            "6: 1344Hz__150HzCutoff")
                      .set__integer_range({accel_rate_range});
    declare_parameter("accelerometer_rate", 3, accel_rate_desc);

    //
    // Accelerometer range

    rcl_interfaces::msg::IntegerRange accel_range_range;
    accel_range_range.set__from_value(0)
                     .set__to_value(3)
                     .set__step(1);

    rcl_interfaces::msg::ParameterDescriptor accel_range_desc;
    accel_range_desc.set__name("accelerometer_range")
                    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                    .set__description("acelerometer range\n0: 2g__1mg_per_lsb\n"
                                                           "1: 4g__2mg_per_lsb\n"
                                                           "2: 8g__4mg_per_lsb\n"
                                                           "3: 16g__12mg_per_lsb")
                    .set__integer_range({accel_range_range});
    declare_parameter("accelerometer_range", 0, accel_range_desc);


    //
    // Gyroscope enable

    rcl_interfaces::msg::ParameterDescriptor gyro_enabled_desc;
    gyro_enabled_desc.set__name("gyroscope_enabled")
                      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
                      .set__description("enable multisense gyroscop");
    declare_parameter("gyroscope_enabled", true, gyro_enabled_desc);

    //
    // Gyroscope rate

    rcl_interfaces::msg::IntegerRange gyro_rate_range;
    gyro_rate_range.set__from_value(0)
                    .set__to_value(6)
                    .set__step(1);

    rcl_interfaces::msg::ParameterDescriptor gyro_rate_desc;
    gyro_rate_desc.set__name("gyroscope_rate")
                      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                      .set__description("gyroscope rate\n0: 100Hz__13HzCutoff\n"
                                                         "1: 200Hz__13HzCutoff\n"
                                                         "2: 200Hz__25HzCutoff\n"
                                                         "3: 400Hz__25HzCutoff\n"
                                                         "4: 400Hz__50HzCutoff\n"
                                                         "5: 800Hz__50HzCutoff\n"
                                                         "6: 800Hz__110HzCutoff")
                      .set__integer_range({gyro_rate_range});
    declare_parameter("gyroscope_rate", 3, gyro_rate_desc);

    //
    // Gyroscope range

    rcl_interfaces::msg::IntegerRange gyro_range_range;
    gyro_range_range.set__from_value(0)
                     .set__to_value(2)
                     .set__step(1);

    rcl_interfaces::msg::ParameterDescriptor gyro_range_desc;
    gyro_range_desc.set__name("gyroscope_range")
                    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                    .set__description("gyroscope range\n0: 250dps__9mdps_per_lsb\n"
                                                        "1: 500dps__18mdps_per_lsb\n"
                                                        "2: 2000dps__70mdps_per_lsb")
                    .set__integer_range({gyro_range_range});
    declare_parameter("gyroscope_range", 0, gyro_range_desc);

    //
    // Magnetometer enable

    rcl_interfaces::msg::ParameterDescriptor magnetometer_enabled_desc;
    magnetometer_enabled_desc.set__name("magnetometer_enabled")
                      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
                      .set__description("enable multisense gyroscope");
    declare_parameter("magnetometer_enabled", true, magnetometer_enabled_desc);

    //
    // Magnetometer rate

    rcl_interfaces::msg::IntegerRange magnetometer_rate_range;
    magnetometer_rate_range.set__from_value(0)
                    .set__to_value(3)
                    .set__step(1);

    rcl_interfaces::msg::ParameterDescriptor magnetometer_rate_desc;
    magnetometer_rate_desc.set__name("magnetometer_rate")
                      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                      .set__description("magnetometer rate\n0: 10Hz\n"
                                                            "1: 25Hz\n"
                                                            "2: 50Hz\n"
                                                            "3: 100Hz")
                      .set__integer_range({magnetometer_rate_range});
    declare_parameter("magnetometer_rate", 0, magnetometer_rate_desc);

    //
    // Magnetometer range

    rcl_interfaces::msg::IntegerRange magnetometer_range_range;
    magnetometer_range_range.set__from_value(0)
                     .set__to_value(6)
                     .set__step(1);

    rcl_interfaces::msg::ParameterDescriptor magnetometer_range_desc;
    magnetometer_range_desc.set__name("magnetometer_range")
                    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
                    .set__description("magnetometer range\n0: 1p3gauss__1020ugauss_per_lsb\n"
                                                           "1: 1p9gauss__1316ugauss_per_lsb\n"
                                                           "2: 2p5gauss__1667ugauss_per_lsb\n"
                                                           "3: 4p0gauss__2500ugauss_per_lsb\n"
                                                           "4: 5p6gauss__3390ugauss_per_lsb\n"
                                                           "5: 5p6gauss__3390ugauss_per_lsb\n"
                                                           "6: 8p1gauss__4878ugauss_per_lsb")
                    .set__integer_range({magnetometer_range_range});
    declare_parameter("magnetometer_range", 0, magnetometer_range_desc);
}

rcl_interfaces::msg::SetParametersResult Imu::parameterCallback(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.set__successful(true);

    uint32_t imu_samples_per_message = imu_samples_per_message_;
    bool update_accelerometer = false;
    bool update_gyroscope = false;
    bool update_magnetometer = false;

    auto accelerometer_config = accelerometer_config_;
    auto gyroscope_config = gyroscope_config_;
    auto magnetometer_config = magnetometer_config_;

    for (const auto &parameter : parameters)
    {
        const auto type = parameter.get_type();
        if (type == rclcpp::ParameterType::PARAMETER_NOT_SET)
        {
            continue;
        }

        const auto name = parameter.get_name();

        if (name == "imu_samples_per_message")
        {
            if (type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid imu samples per message type");
            }

            imu_samples_per_message = get_as_number<int>(parameter);
        }
        else if (name == "accelerometer_enabled")
        {
            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
            {
                return result.set__successful(false).set__reason("invalid accelerometer enabled type");
            }

            const auto value = parameter.as_bool();
            if (accelerometer_config && accelerometer_config->enabled != value)
            {
                accelerometer_config->enabled = value;
                update_accelerometer = true;
            }
        }
        else if (name == "accelerometer_rate")
        {
            if (type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid accelerometer rate type");
            }

            const auto value = static_cast<uint32_t>(get_as_number<int>(parameter));
            if (accelerometer_config && accelerometer_config->rateTableIndex != value)
            {
                accelerometer_config->rateTableIndex = value;
                update_accelerometer = true;
            }
        }
        else if (name == "accelerometer_range")
        {
            if (type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid accelerometer range type");
            }

            const auto value = static_cast<uint32_t>(get_as_number<int>(parameter));
            if (accelerometer_config && accelerometer_config->rangeTableIndex != value)
            {
                accelerometer_config->rangeTableIndex = value;
                update_accelerometer = true;
            }
        }
        else if (name == "gyroscope_enabled")
        {
            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
            {
                return result.set__successful(false).set__reason("invalid gyroscope enabled type");
            }

            const auto value = parameter.as_bool();
            if (gyroscope_config && gyroscope_config->enabled != value)
            {
                gyroscope_config->enabled = value;
                update_gyroscope = true;
            }
        }
        else if (name == "gyroscope_rate")
        {
            if (type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid gyroscope rate type");
            }

            const auto value = static_cast<uint32_t>(get_as_number<int>(parameter));
            if (gyroscope_config && gyroscope_config->rateTableIndex != value)
            {
                gyroscope_config->rateTableIndex = value;
                update_gyroscope = true;
            }
        }
        else if (name == "gyroscope_range")
        {
            if (type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid gyroscope range type");
            }

            const auto value = static_cast<uint32_t>(get_as_number<int>(parameter));
            if (gyroscope_config && gyroscope_config->rangeTableIndex != value)
            {
                gyroscope_config->rangeTableIndex = value;
                update_gyroscope = true;
            }
        }
        else if (name == "magnetometer_enabled")
        {
            if (type != rclcpp::ParameterType::PARAMETER_BOOL)
            {
                return result.set__successful(false).set__reason("invalid magnetometer enabled type");
            }

            const auto value = parameter.as_bool();
            if (magnetometer_config && magnetometer_config->enabled != value)
            {
                magnetometer_config->enabled = value;
                update_magnetometer = true;
            }
        }
        else if (name == "magnetometer_rate")
        {
            if (type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid magnetometer range type");
            }

            const auto value = static_cast<uint32_t>(get_as_number<int>(parameter));
            if (magnetometer_config && magnetometer_config->rateTableIndex != value)
            {
                magnetometer_config->rateTableIndex = value;
                update_magnetometer = true;
            }
        }
        else if (name == "magnetometer_range")
        {
            if (type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid magnetometer range type");
            }

            const auto value = static_cast<uint32_t>(get_as_number<int>(parameter));
            if (magnetometer_config && magnetometer_config->rangeTableIndex != value)
            {
                magnetometer_config->rangeTableIndex = value;
                update_magnetometer = true;
            }
        }
    }

    std::vector<imu::Config> changed_configs;
    if (update_accelerometer && accelerometer_config)
    {
        changed_configs.push_back(accelerometer_config.value());
    }

    if (update_gyroscope && gyroscope_config)
    {
        changed_configs.push_back(gyroscope_config.value());
    }

    if (update_magnetometer && magnetometer_config)
    {
        changed_configs.push_back(magnetometer_config.value());
    }

    if (!changed_configs.empty() || imu_samples_per_message_ != imu_samples_per_message)
    {
        RCLCPP_WARN(get_logger(), "IMU: IMU configuration changes will take effect after all IMU "
                    "topic subscriptions have been closed.");

        imu_samples_per_message_ = imu_samples_per_message;

        if (const auto status = driver_->setImuConfig(false, imu_samples_per_message_, changed_configs); status != Status_Ok)
        {
            return result.set__successful(false).set__reason(Channel::statusString(status));
        }

        //
        // Only update our cached configs if the unit accepted the new configs

        accelerometer_config_ = (update_accelerometer && accelerometer_config) ? accelerometer_config : accelerometer_config_;
        gyroscope_config_ = (update_gyroscope && gyroscope_config) ? gyroscope_config : gyroscope_config_;
        magnetometer_config_ = (update_magnetometer && magnetometer_config) ? magnetometer_config : magnetometer_config_;
    }

    return result;
}

} // namespace
