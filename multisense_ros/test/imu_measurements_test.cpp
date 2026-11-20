/**
 * @file imu_measurements_test.cpp
 *
 * Copyright 2026
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * All rights reserved.
 **/

#include <gtest/gtest.h>

#include "multisense_ros/imu_measurements.h"

namespace multisense_ros {

namespace { // anonymous

using Measurement = multisense::ImuSample::Measurement;

multisense::ImuSample accel_only(float z)
{
    multisense::ImuSample sample{};
    sample.accelerometer = Measurement{0.0f, 0.0f, z};
    return sample;
}

multisense::ImuSample gyro_only(float z)
{
    multisense::ImuSample sample{};
    sample.gyroscope = Measurement{0.0f, 0.0f, z};
    return sample;
}

TEST(LatestImuMeasurements, NoOtherReadingLeavesSampleUnchanged)
{
    LatestImuMeasurements latest{};

    const auto output = latest.fill(gyro_only(30.0f));
    EXPECT_FALSE(output.accelerometer);
    ASSERT_TRUE(output.gyroscope);
    EXPECT_EQ(output.gyroscope->z, 30.0f);
}

TEST(LatestImuMeasurements, FillsEachHalfWithTheLatestOtherReading)
{
    LatestImuMeasurements latest{};

    latest.fill(accel_only(1.0f));

    const auto gyro = latest.fill(gyro_only(30.0f));
    ASSERT_TRUE(gyro.accelerometer);
    ASSERT_TRUE(gyro.gyroscope);
    EXPECT_EQ(gyro.accelerometer->z, 1.0f);
    EXPECT_EQ(gyro.gyroscope->z, 30.0f);

    const auto accel = latest.fill(accel_only(0.5f));
    ASSERT_TRUE(accel.accelerometer);
    ASSERT_TRUE(accel.gyroscope);
    EXPECT_EQ(accel.accelerometer->z, 0.5f);
    EXPECT_EQ(accel.gyroscope->z, 30.0f);
}

TEST(LatestImuMeasurements, CombinedAndMagnetometerOnlySamplesAreUnchanged)
{
    LatestImuMeasurements latest{};
    latest.fill(accel_only(1.0f));
    latest.fill(gyro_only(30.0f));

    multisense::ImuSample both{};
    both.accelerometer = Measurement{0.0f, 0.0f, 0.25f};
    both.gyroscope = Measurement{0.0f, 0.0f, 10.0f};
    const auto combined = latest.fill(both);
    EXPECT_EQ(combined.accelerometer->z, 0.25f);
    EXPECT_EQ(combined.gyroscope->z, 10.0f);

    multisense::ImuSample magnetometer{};
    magnetometer.magnetometer = Measurement{1.0f, 2.0f, 3.0f};
    const auto output = latest.fill(magnetometer);
    EXPECT_FALSE(output.accelerometer);
    EXPECT_FALSE(output.gyroscope);
    ASSERT_TRUE(output.magnetometer);
}

TEST(LatestImuMeasurements, ClearForgetsTheLatestReadings)
{
    LatestImuMeasurements latest{};
    latest.fill(accel_only(1.0f));
    latest.clear();

    EXPECT_FALSE(latest.fill(gyro_only(30.0f)).accelerometer);
}

} // namespace

} // namespace multisense_ros
