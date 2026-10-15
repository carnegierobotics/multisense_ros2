/**
 * @file publisher_streams_test.cpp
 *
 * Copyright 2026
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * All rights reserved.
 **/

#include <array>

#include <gtest/gtest.h>

#include "multisense_ros/publisher_streams.h"

namespace multisense_ros {

namespace { // anonymous

using ds = multisense::DataSource;

TEST(PublisherStreams, NoDemandRequiresNoStreams)
{
    EXPECT_TRUE(required_streams({}).empty());
    EXPECT_TRUE(required_streams({{{ds::IMU}, []() { return 0; }}}).empty());
    EXPECT_TRUE(stream_difference({}, {}).empty());
}

TEST(PublisherStreams, SubscriberChangesAreVisibleWithoutMatchEvents)
{
    size_t subscribers = 0;
    const std::vector<PublisherStreams> publishers{{{ds::LEFT_DISPARITY_RAW}, [&subscribers]() { return subscribers; }}};
    EXPECT_TRUE(required_streams(publishers).empty());

    subscribers = 3;
    const auto applied = required_streams(publishers);
    EXPECT_EQ(stream_difference(applied, {}), (std::vector<ds>{ds::LEFT_DISPARITY_RAW}));

    subscribers = 2;
    EXPECT_TRUE(stream_difference(required_streams(publishers), applied).empty());
    EXPECT_TRUE(stream_difference(applied, required_streams(publishers)).empty());

    subscribers = 0;
    EXPECT_EQ(stream_difference(applied, required_streams(publishers)), (std::vector<ds>{ds::LEFT_DISPARITY_RAW}));
}

TEST(PublisherStreams, DepthAndPointCloudShareDisparity)
{
    size_t depth_subscribers = 1;
    size_t cloud_subscribers = 1;
    const std::vector<PublisherStreams> publishers{
        {{ds::LEFT_DISPARITY_RAW}, [&depth_subscribers]() { return depth_subscribers; }},
        {{ds::LEFT_DISPARITY_RAW}, [&cloud_subscribers]() { return cloud_subscribers; }}};
    const auto applied = required_streams(publishers);

    depth_subscribers = 0;
    EXPECT_EQ(required_streams(publishers), applied);

    cloud_subscribers = 0;
    EXPECT_EQ(stream_difference(applied, required_streams(publishers)), (std::vector<ds>{ds::LEFT_DISPARITY_RAW}));
}

TEST(PublisherStreams, LumaPointCloudStartsOnlyTheAdditionalSource)
{
    const StreamSet applied{ds::LEFT_DISPARITY_RAW};
    const auto required = required_streams(
        {{{ds::LEFT_DISPARITY_RAW, ds::LEFT_RECTIFIED_RAW}, []() { return 1; }}});

    EXPECT_EQ(stream_difference(required, applied), (std::vector<ds>{ds::LEFT_RECTIFIED_RAW}));
    EXPECT_TRUE(stream_difference(applied, required).empty());
}

TEST(PublisherStreams, LastSubscriberStopsOnlyTheUnusedSource)
{
    const StreamSet applied{ds::LEFT_DISPARITY_RAW, ds::IMU};
    const auto required = required_streams(
        {{{ds::LEFT_DISPARITY_RAW}, []() { return 1; }}, {{ds::IMU}, []() { return 0; }}});

    EXPECT_EQ(stream_difference(applied, required), (std::vector<ds>{ds::IMU}));
    EXPECT_TRUE(stream_difference(required, applied).empty());
}

TEST(PublisherStreams, ThermalRemainsActiveWhileAnyImagerHasSubscribers)
{
    std::array<size_t, 6> subscribers{};
    std::vector<PublisherStreams> publishers;
    for (size_t id = 0; id < subscribers.size(); ++id)
    {
        publishers.push_back({{ds::THERMAL}, [&subscribers, id]() { return subscribers[id]; }});
    }

    subscribers[0] = 1;
    subscribers[5] = 3;
    const auto applied = required_streams(publishers);
    EXPECT_EQ(applied, (StreamSet{ds::THERMAL}));

    subscribers[0] = 0;
    EXPECT_EQ(required_streams(publishers), applied);

    subscribers[5] = 0;
    EXPECT_EQ(stream_difference(applied, required_streams(publishers)), (std::vector<ds>{ds::THERMAL}));
}

struct AuxSources
{
    ds mono;
    ds color;
    ds chroma;
};

class AuxPublisherStreams : public ::testing::TestWithParam<AuxSources>
{
};

TEST_P(AuxPublisherStreams, ColorDisconnectPreservesMono)
{
    const auto sources = GetParam();
    size_t color_subscribers = 1;
    const std::vector<PublisherStreams> publishers{
        {{sources.mono}, []() { return 1; }},
        {{sources.color}, [&color_subscribers]() { return color_subscribers; }}};
    const auto applied = required_streams(publishers);
    EXPECT_EQ(applied, (StreamSet{sources.mono, sources.chroma}));

    color_subscribers = 0;
    const auto required = required_streams(publishers);
    EXPECT_EQ(required, (StreamSet{sources.mono}));
    EXPECT_EQ(stream_difference(applied, required), (std::vector<ds>{sources.chroma}));
    EXPECT_TRUE(stream_difference(required, applied).empty());
}

TEST_P(AuxPublisherStreams, MonoDisconnectPreservesColor)
{
    const auto sources = GetParam();
    size_t mono_subscribers = 1;
    const std::vector<PublisherStreams> publishers{
        {{sources.mono}, [&mono_subscribers]() { return mono_subscribers; }},
        {{sources.color}, []() { return 1; }}};
    const auto applied = required_streams(publishers);

    mono_subscribers = 0;
    const auto required = required_streams(publishers);
    EXPECT_EQ(required, (StreamSet{sources.mono, sources.chroma}));
    EXPECT_TRUE(stream_difference(applied, required).empty());
    EXPECT_TRUE(stream_difference(required, applied).empty());
}

INSTANTIATE_TEST_SUITE_P(AuxImages, AuxPublisherStreams,
                        ::testing::Values(AuxSources{ds::AUX_LUMA_RAW, ds::AUX_RAW, ds::AUX_CHROMA_RAW},
                                          AuxSources{ds::AUX_LUMA_RECTIFIED_RAW, ds::AUX_RECTIFIED_RAW,
                                                     ds::AUX_CHROMA_RECTIFIED_RAW}));

TEST(PublisherStreams, ColorPointCloudDisconnectPreservesRectifiedMono)
{
    size_t cloud_subscribers = 1;
    const std::vector<PublisherStreams> publishers{
        {{ds::AUX_LUMA_RECTIFIED_RAW}, []() { return 1; }},
        {{ds::LEFT_DISPARITY_RAW, ds::AUX_RECTIFIED_RAW}, [&cloud_subscribers]() { return cloud_subscribers; }}};
    const auto applied = required_streams(publishers);

    cloud_subscribers = 0;
    const auto required = required_streams(publishers);
    EXPECT_EQ(required, (StreamSet{ds::AUX_LUMA_RECTIFIED_RAW}));
    EXPECT_EQ(stream_difference(applied, required), (std::vector<ds>{ds::LEFT_DISPARITY_RAW, ds::AUX_CHROMA_RECTIFIED_RAW}));
    EXPECT_TRUE(stream_difference(required, applied).empty());
}

} // anonymous

} // namespace multisense_ros
