/**
 * @file stream_demand.h
 *
 * Copyright 2026
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * All rights reserved.
 **/

#pragma once

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <type_traits>
#include <vector>

namespace multisense_ros {

template <typename Source>
struct TopicStreamDemand
{
    std::vector<Source> sources;
    size_t subscriber_count = 0;
};

template <typename Source>
struct StreamDemandUpdate
{
    std::vector<Source> start;
    std::vector<Source> stop;
    std::vector<Source> active;
};

template <typename Source>
bool stream_source_less(const Source &lhs, const Source &rhs)
{
    if constexpr (std::is_enum_v<Source>)
    {
        using Underlying = std::underlying_type_t<Source>;
        return static_cast<Underlying>(lhs) < static_cast<Underlying>(rhs);
    }
    else
    {
        return lhs < rhs;
    }
}

///
/// Desired hardware sources are the union of every source whose topic currently has at least one
/// subscriber. Counts are absolute, so duplicate or missed match events cannot drive demand negative.
/// Transitions are idempotent: a source already in the applied set is not restarted.
///
template <typename Source>
StreamDemandUpdate<Source> reconcile_stream_demand(const std::vector<TopicStreamDemand<Source>> &topics,
                                                   const std::vector<Source> &previously_active)
{
    std::vector<Source> desired;
    for (const auto &topic : topics)
    {
        if (topic.subscriber_count == 0)
        {
            continue;
        }
        desired.insert(desired.end(), topic.sources.begin(), topic.sources.end());
    }
    std::sort(desired.begin(), desired.end(), stream_source_less<Source>);
    desired.erase(std::unique(desired.begin(), desired.end()), desired.end());

    auto previous = previously_active;
    std::sort(previous.begin(), previous.end(), stream_source_less<Source>);
    previous.erase(std::unique(previous.begin(), previous.end()), previous.end());

    StreamDemandUpdate<Source> update;
    update.active = desired;
    std::set_difference(desired.begin(),
                        desired.end(),
                        previous.begin(),
                        previous.end(),
                        std::back_inserter(update.start),
                        stream_source_less<Source>);
    std::set_difference(previous.begin(),
                        previous.end(),
                        desired.begin(),
                        desired.end(),
                        std::back_inserter(update.stop),
                        stream_source_less<Source>);
    return update;
}

} // namespace multisense_ros
