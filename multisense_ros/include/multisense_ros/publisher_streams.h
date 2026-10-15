/**
 * @file publisher_streams.h
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
#include <functional>
#include <iterator>
#include <set>
#include <vector>

#include <MultiSense/MultiSenseTypes.hh>

namespace multisense_ros {

using StreamSet = std::set<multisense::DataSource>;

struct PublisherStreams
{
    std::vector<multisense::DataSource> sources;
    std::function<size_t()> subscriber_count;
};

//
// Read current subscriber counts so repeated or missed match events cannot corrupt demand.
// Each subscribed publisher contributes its required hardware streams.
//
inline StreamSet required_streams(const std::vector<PublisherStreams> &publishers)
{
    using ds = multisense::DataSource;
    StreamSet required;
    for (const auto &publisher : publishers)
    {
        if (publisher.subscriber_count() == 0)
        {
            continue;
        }

        for (const auto source : publisher.sources)
        {
            //
            // Color sources contain luma and chroma. Track them separately to preserve shared streams.
            //
            switch (source)
            {
                case ds::AUX_RAW:
                {
                    required.insert({ds::AUX_LUMA_RAW, ds::AUX_CHROMA_RAW});
                    break;
                }
                case ds::AUX_RECTIFIED_RAW:
                {
                    required.insert({ds::AUX_LUMA_RECTIFIED_RAW, ds::AUX_CHROMA_RECTIFIED_RAW});
                    break;
                }
                default:
                {
                    required.insert(source);
                    break;
                }
            }
        }
    }

    return required;
}

inline std::vector<multisense::DataSource> stream_difference(const StreamSet &lhs, const StreamSet &rhs)
{
    std::vector<multisense::DataSource> difference;
    std::set_difference(lhs.begin(), lhs.end(), rhs.begin(), rhs.end(), std::back_inserter(difference));
    return difference;
}

} // namespace multisense_ros
