/**
 * @file laser.cpp
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

#include <arpa/inet.h>

#include <angles/angles.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#if defined(ROS_FOXY) || defined(ROS_GALACTIC)
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <multisense_ros/laser.h>
#include <multisense_ros/parameter_utilities.h>
#include <multisense_ros/point_cloud_utilities.h>

using namespace crl::multisense;
using namespace std::chrono_literals;

namespace { // anonymous

tf2::Transform makeTransform(const float T[4][4])
{
    //
    // Manually create the rotation matrix
    tf2::Matrix3x3 rot {T[0][0],
                        T[0][1],
                        T[0][2],
                        T[1][0],
                        T[1][1],
                        T[1][2],
                        T[2][0],
                        T[2][1],
                        T[2][2]};

    //
    // Maually create the translation vector
    tf2::Vector3 trans{T[0][3], T[1][3], T[2][3]};

    return tf2::Transform(rot, trans);
}

} // anonymous

namespace multisense_ros {

namespace { // anonymous

//
// Shims for c-style driver callbacks

void lCB(const lidar::Header&        header,
	 void*                       userDataP)
{
    reinterpret_cast<Laser*>(userDataP)->scanCallback(header);
}

void pCB(const lidar::Header&        header,
	 void*                       userDataP)
{
    reinterpret_cast<Laser*>(userDataP)->pointCloudCallback(header);
}

} // anonymous

Laser::Laser(const std::string& node_name,
             const rclcpp::NodeOptions& options,
             Channel* driver,
             const std::string& tf_prefix):
    Node(node_name, options),
    driver_(driver),
    static_tf_broadcaster_(std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this)),
    frame_id_(tf_prefix + "/head_hokuyo_frame"),
    left_camera_optical_ (tf_prefix + "/" + "left_camera_optical_frame"),
    motor_(tf_prefix + "/" + "motor"),
    spindle_(tf_prefix + "/" + "spindle"),
    hokuyo_(tf_prefix + "/" + "hokuyo_link"),
    spindle_angle_(0.0),
    previous_scan_time_(0),
    active_streams_(Source_Unknown)
{
    //
    // Get device info

    system::DeviceInfo  device_info;

    if (const auto status = driver_->getDeviceInfo(device_info); status != Status_Ok)
    {
        RCLCPP_ERROR(get_logger(), "Laser: failed to query device info: %s",
                  Channel::statusString(status));
        return;
    }

    switch(device_info.hardwareRevision)
    {
        case system::DeviceInfo::HARDWARE_REV_MULTISENSE_SL:
        {
            break;
        }
        default:
        {
            RCLCPP_INFO(get_logger(), "Laser: hardware does not support a laser");
            return;
        }
    }

    //
    // Stop lidar stream

    stop();

    //
    // Query calibration from sensor

    if (const auto status = driver_->getLidarCalibration(lidar_cal_); status != Status_Ok)
    {
        RCLCPP_WARN(get_logger(), "could not query lidar calibration (%s), using URDF defaults",
                 Channel::statusString(status));
    }
    else
    {
        //
        // Create two static transforms representing sensor calibration
        motor_to_camera_ = makeTransform(lidar_cal_.cameraToSpindleFixed);
        laser_to_spindle_ = makeTransform(lidar_cal_.laserToSpindle);
    }

    //
    // Create scan publisher

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(LASER_SCAN_TOPIC, rclcpp::SensorDataQoS());

    //
    // Initialize point cloud structure

    point_cloud_ = initialize_pointcloud<float>(true, tf_prefix + "/left_camera_optical_frame", "intensity");

    //
    // Create point cloud publisher

    point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(LASER_POINTCLOUD_TOPIC, rclcpp::SensorDataQoS());

    //
    // Create calibration publishers

    raw_lidar_cal_pub_ = create_publisher<multisense_msgs::msg::RawLidarCal>(RAW_LASER_CAL_TOPIC, rclcpp::SensorDataQoS());

    raw_lidar_data_pub_ = create_publisher<multisense_msgs::msg::RawLidarData>(RAW_LASER_DATA_TOPIC, rclcpp::SensorDataQoS());

    //
    // Publish calibration

    multisense_msgs::msg::RawLidarCal raw_lidar_cal;

    const float *calP = reinterpret_cast<const float*>(&(lidar_cal_.laserToSpindle[0][0]));
    for(size_t i=0; i<16; ++i)
    {
        raw_lidar_cal.laser_to_spindle[i] = calP[i];
    }

    calP = reinterpret_cast<const float*>(&(lidar_cal_.cameraToSpindleFixed[0][0]));
    for(size_t i=0; i<16; ++i)
    {
        raw_lidar_cal.camera_to_spindle_fixed[i] = calP[i];
    }

    raw_lidar_cal_pub_->publish(raw_lidar_cal);

    //
    // Populate the jointstates message for publishing the laser spindle
    // angle

    joint_states_.name.resize(1);
    joint_states_.position.resize(1);
    joint_states_.velocity.resize(1);
    joint_states_.effort.resize(1);
    joint_states_.name[0] = tf_prefix + "/motor_joint";
    joint_states_.position[0] = 0.0;
    joint_states_.velocity[0] = 0.0;
    joint_states_.effort[0] = 0.0;

    //
    // Create a joint state publisher

    joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>(JOINT_STATE_TOPIC, rclcpp::SensorDataQoS());

    //
    // Create a timer routine to publish the laser transform even when nothing
    // is subscribed to the laser topics. Publishing occurs at 1Hz

    timer_ = create_wall_timer(1s, std::bind(&Laser::tfPublisher, this));

    publisher_timer_ = create_wall_timer(500ms, std::bind(&Laser::streamControl, this));

    //
    // Register callbacks, driver creates dedicated background thread for each

    driver_->addIsolatedCallback(lCB, this);
    driver_->addIsolatedCallback(pCB, this);

    std::vector<geometry_msgs::msg::TransformStamped> stamped_transforms(2);
    stamped_transforms[0].header.stamp = rclcpp::Clock().now();
    stamped_transforms[0].header.frame_id = left_camera_optical_;
    stamped_transforms[0].child_frame_id = motor_;
    stamped_transforms[0].transform = tf2::toMsg(motor_to_camera_);

    stamped_transforms[1].header.stamp = rclcpp::Clock().now();
    stamped_transforms[1].header.frame_id = spindle_;
    stamped_transforms[1].child_frame_id = hokuyo_;
    stamped_transforms[1].transform = tf2::toMsg(laser_to_spindle_);

    static_tf_broadcaster_->sendTransform(stamped_transforms);

    paramter_handle_ = add_on_set_parameters_callback(std::bind(&Laser::parameterCallback, this, std::placeholders::_1));

    //
    // Motor speed

    rcl_interfaces::msg::FloatingPointRange motor_speed_range;
    motor_speed_range.set__from_value(0.0)
                     .set__to_value(5.2)
                     .set__step(0.1);

    rcl_interfaces::msg::ParameterDescriptor motor_speed_desc;
    motor_speed_desc.set__name("motor_speed")
                    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
                    .set__description("laser motor speed in rad/s")
                    .set__floating_point_range({motor_speed_range});
    declare_parameter("motor_speed", 0.0, motor_speed_desc);
}

Laser::~Laser()
{
    stop();
    driver_->removeIsolatedCallback(lCB);
    driver_->removeIsolatedCallback(pCB);
}


void Laser::pointCloudCallback(const lidar::Header& header)
{
    //
    // Get out if we have no work to do

    if (0 == count_subscribers(LASER_POINTCLOUD_TOPIC))
        return;

    point_cloud_.data.resize(point_cloud_.point_step * header.pointCount);
    point_cloud_.row_step     = header.pointCount * point_cloud_.point_step;
    point_cloud_.width        = header.pointCount;
    point_cloud_.header.stamp = rclcpp::Time(header.timeStartSeconds, 1000 * header.timeStartMicroSeconds);


    //
    // For convenience below

    uint8_t       *cloudP            = reinterpret_cast<uint8_t*>(&point_cloud_.data[0]);
    const double   arcRadians        = 1e-6 * static_cast<double>(header.scanArc);
    const double   mirrorThetaStart  = -arcRadians / 2.0;
    const double   spindleAngleStart = angles::normalize_angle(1e-6 * static_cast<double>(header.spindleAngleStart));
    const double   spindleAngleEnd   = angles::normalize_angle(1e-6 * static_cast<double>(header.spindleAngleEnd));
    const double   spindleAngleRange = angles::normalize_angle(spindleAngleEnd - spindleAngleStart);

    for( size_t i = 0; i < header.pointCount; ++i, cloudP += point_cloud_.point_step)
    {

        //
        // Percent through the scan arc

        const double percent = static_cast<double>(i) / static_cast<double>(header.pointCount - 1);

        //
        // The mirror angle for this point

        const double mirrorTheta = mirrorThetaStart + percent * arcRadians;

        //
        // The rotation about the spindle

        const double spindleTheta = spindleAngleStart + percent * spindleAngleRange;

        const auto spindle_to_motor = getSpindleTransform(spindleTheta);

        //
        // The coordinate in left optical frame

        const double rangeMeters = 1e-3 * static_cast<double>(header.rangesP[i]);  // from millimeters

        const tf2::Vector3 pointMotor  = (laser_to_spindle_ *
                                         tf2::Vector3(rangeMeters * std::sin(mirrorTheta),
                                                      0.0,
                                                      rangeMeters *  std::cos(mirrorTheta)));

        const tf2::Vector3 pointCamera = motor_to_camera_ * (spindle_to_motor * pointMotor);

        //
        // Copy data to point cloud structure
        float* cloudPf = reinterpret_cast<float*>(cloudP);

        cloudPf[0] = static_cast<float>(pointCamera.getX());
        cloudPf[1] = static_cast<float>(pointCamera.getY()),
        cloudPf[2] = static_cast<float>(pointCamera.getZ());
        cloudPf[3] = static_cast<float>(header.intensitiesP[i]); // in device units
    }

    point_cloud_pub_->publish(point_cloud_);
}

void Laser::scanCallback(const lidar::Header& header)
{

    const rclcpp::Time start_absolute_time(header.timeStartSeconds,
                                           1000 * header.timeStartMicroSeconds);
    const rclcpp::Time end_absolute_time(header.timeEndSeconds,
                                         1000 * header.timeEndMicroSeconds);
    const auto scan_time = end_absolute_time - start_absolute_time;

    const float angle_start = 1e-6 * static_cast<float>(header.spindleAngleStart);
    const float angle_end   = 1e-6 * static_cast<float>(header.spindleAngleEnd);

    //
    // Initialize the previous scan time to the start time if it has not
    // been previously set

    if (previous_scan_time_.seconds() == 0 && previous_scan_time_.nanoseconds() == 0)
    {
        previous_scan_time_ = start_absolute_time;
    }


    //
    // Compute the velocity between our last scan and the start of our current
    // scan

    float velocity = angles::normalize_angle((angle_start - spindle_angle_)) /
        ((start_absolute_time - previous_scan_time_).nanoseconds() / 1e9);

    publishSpindleTransform(angle_start, velocity, start_absolute_time);
    spindle_angle_ = angle_start;

    //
    // Compute the velocity for the spindle during the duration of our
    // laser scan

    velocity = angles::normalize_angle((angle_end - angle_start)) / (scan_time.nanoseconds() / 1e9);

    publishSpindleTransform(angle_end, velocity, end_absolute_time);
    spindle_angle_ = angle_end;
    previous_scan_time_ = end_absolute_time;

    if (count_subscribers(LASER_SCAN_TOPIC) > 0)
    {
        const double arcRadians = 1e-6 * static_cast<double>(header.scanArc);

        laser_msg_.header.frame_id = frame_id_;
        laser_msg_.header.stamp    = start_absolute_time;
        laser_msg_.scan_time       = scan_time.nanoseconds() / 1e9;
        laser_msg_.time_increment  = laser_msg_.scan_time / header.pointCount;
        laser_msg_.angle_min       = -arcRadians / 2.0;
        laser_msg_.angle_max       = arcRadians / 2.0;
        laser_msg_.angle_increment = arcRadians / (header.pointCount - 1);
        laser_msg_.range_min       = 0.0;
        laser_msg_.range_max       = static_cast<double>(header.maxRange) / 1000.0;

        laser_msg_.ranges.resize(header.pointCount);
        laser_msg_.intensities.resize(header.pointCount);

        for (size_t i=0; i<header.pointCount; i++)
        {
            laser_msg_.ranges[i]      = 1e-3 * static_cast<float>(header.rangesP[i]); // from millimeters
            laser_msg_.intensities[i] = static_cast<float>(header.intensitiesP[i]);   // in device units
        }

        scan_pub_->publish(laser_msg_);
    }

    if (count_subscribers(RAW_LASER_DATA_TOPIC) > 0)
    {
        multisense_msgs::msg::RawLidarData ros_msg;

        ros_msg.scan_count  = header.scanId;
        ros_msg.time_start  = start_absolute_time;
        ros_msg.time_end    = end_absolute_time;
        ros_msg.angle_start = header.spindleAngleStart;
        ros_msg.angle_end   = header.spindleAngleEnd;

        ros_msg.distance.resize(header.pointCount);
        memcpy(&(ros_msg.distance[0]),
               header.rangesP,
               header.pointCount * sizeof(uint32_t));

        ros_msg.intensity.resize(header.pointCount);
        memcpy(&(ros_msg.intensity[0]),
               header.intensitiesP,
               header.pointCount * sizeof(uint32_t));

        raw_lidar_data_pub_->publish(ros_msg);
    }
}

void Laser::publishSpindleTransform(const float spindle_angle, const float velocity, const rclcpp::Time& time)
{
    joint_states_.header.stamp = time;
    joint_states_.position[0] = spindle_angle;
    joint_states_.velocity[0] = velocity;
    joint_states_pub_->publish(joint_states_);
}

tf2::Transform Laser::getSpindleTransform(float spindle_angle)
{
    //
    // Spindle angle turns about the z-axis to create a transform where it adjusts
    // yaw
    tf2::Quaternion spindle_rot;
    spindle_rot.setRPY(0.0, 0.0, spindle_angle);
    tf2::Transform spindle_to_motor(spindle_rot);

    return spindle_to_motor;
}

void Laser::tfPublisher()
{
    //
    // If our message time is 0 or our message time is over 1 second old
    // we are not subscribed to a laser topic anymore. Publish the default
    // transform
    if ( (laser_msg_.header.stamp == rclcpp::Time(0) ||
         (rclcpp::Clock().now() - laser_msg_.header.stamp >= rclcpp::Duration(1, 0))) &&
         (point_cloud_.header.stamp == rclcpp::Time(0) ||
         (rclcpp::Clock().now() - point_cloud_.header.stamp >= rclcpp::Duration(1, 0))) )

    {
        publishSpindleTransform(spindle_angle_, 0.0, rclcpp::Clock().now());
    }
}

size_t Laser::getNumSubscribers()
{
    return count_subscribers(LASER_SCAN_TOPIC) +
           count_subscribers(LASER_POINTCLOUD_TOPIC) +
           count_subscribers(RAW_LASER_DATA_TOPIC);
}

void Laser::streamControl()
{
    const auto num_subscribers = getNumSubscribers();

    if (num_subscribers > 0 && (active_streams_ & Source_Lidar_Scan) == 0)
    {
        if (const auto status = driver_->startStreams(Source_Lidar_Scan); status != Status_Ok)
        {
            RCLCPP_ERROR(get_logger(), "Laser: failed to start laser stream: %s",
                      Channel::statusString(status));
            return;
        }

        active_streams_ = Source_Lidar_Scan;
    }
    else if (num_subscribers <= 0 && (active_streams_ & Source_Lidar_Scan) == Source_Lidar_Scan)
    {
        if (const auto status = driver_->stopStreams(Source_Lidar_Scan); status != Status_Ok)
        {
            RCLCPP_ERROR(get_logger(), "Laser: failed to stop laser stream: %s",
                      Channel::statusString(status));
            return;
        }

        active_streams_ = Source_Unknown;
    }
}

void Laser::stop()
{
    if (const auto status = driver_->stopStreams(Source_Lidar_Scan); status != Status_Ok)
    {
        RCLCPP_ERROR(get_logger(), "Laser: failed to stop laser stream: %s",
                  Channel::statusString(status));
    }
}

rcl_interfaces::msg::SetParametersResult Laser::parameterCallback(const std::vector<rclcpp::Parameter>&  parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.set__successful(true);

    constexpr double radians_per_second_to_rpm = 9.54929659643;

    for (const auto &parameter : parameters)
    {
        const auto type = parameter.get_type();
        if (type == rclcpp::ParameterType::PARAMETER_NOT_SET)
        {
            continue;
        }

        const auto name = parameter.get_name();

        if (name == "motor_speed")
        {
            if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                return result.set__successful(false).set__reason("invalid motor speed type");
            }

            const auto radians_per_second = get_as_number<double>(parameter);
            if (const auto status = driver_->setMotorSpeed(radians_per_second_to_rpm * radians_per_second); status != Status_Ok)
            {
                return result.set__successful(false).set__reason(Channel::statusString(status));
            }
        }
    }

    return result;
}

}
