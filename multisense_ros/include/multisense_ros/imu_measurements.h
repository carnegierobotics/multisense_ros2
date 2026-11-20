/**
 * @file imu_measurements.h
 *
 * Copyright 2026
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * All rights reserved.
 **/

#pragma once

#include <optional>

#include <MultiSense/MultiSenseTypes.hh>

namespace multisense_ros {

//
// On cameras where the accelerometer and gyroscope are separate chips, each ImuSample has only one of the two
// measurements. Track the latest reading of each so the missing half of a sample can be filled in.
//
class LatestImuMeasurements
{
public:
    //
    // Record the measurements in the sample. If the sample has exactly one of accelerometer and gyroscope,
    // return it with the other filled from the latest reading of that sensor, when there is one. Other samples
    // are returned unchanged.
    //
    multisense::ImuSample fill(const multisense::ImuSample &sample)
    {
        if (sample.accelerometer)
        {
            accelerometer_ = sample.accelerometer;
        }

        if (sample.gyroscope)
        {
            gyroscope_ = sample.gyroscope;
        }

        auto output = sample;

        if (sample.accelerometer.has_value() != sample.gyroscope.has_value())
        {
            output.accelerometer = accelerometer_;
            output.gyroscope = gyroscope_;
        }

        return output;
    }

    void clear()
    {
        accelerometer_ = std::nullopt;
        gyroscope_ = std::nullopt;
    }

private:
    std::optional<multisense::ImuSample::Measurement> accelerometer_ = std::nullopt;
    std::optional<multisense::ImuSample::Measurement> gyroscope_ = std::nullopt;
};

} // namespace multisense_ros
