#include "multisense_ros/stream_demand.h"

#include <cassert>
#include <vector>

namespace {

enum class Source
{
    DISPARITY = 1,
    AUX_RECTIFIED = 2,
    AUX_RAW = 3
};

using Demand = multisense_ros::TopicStreamDemand<Source>;

void expect_sources(const std::vector<Source> &actual, const std::vector<Source> &expected)
{
    assert(actual == expected);
}

void subscriber_starts_only_the_demanded_source()
{
    const auto update = multisense_ros::reconcile_stream_demand<Source>({Demand{{Source::AUX_RECTIFIED}, 1}}, {});

    expect_sources(update.start, {Source::AUX_RECTIFIED});
    expect_sources(update.stop, {});
    expect_sources(update.active, {Source::AUX_RECTIFIED});
}

void repeating_the_same_demand_does_not_restart_the_source()
{
    const std::vector<Source> active{Source::AUX_RECTIFIED};
    const auto update = multisense_ros::reconcile_stream_demand<Source>({Demand{{Source::AUX_RECTIFIED}, 2}}, active);

    expect_sources(update.start, {});
    expect_sources(update.stop, {});
    expect_sources(update.active, active);
}

void shared_source_stays_active_while_any_topic_has_a_subscriber()
{
    const auto update = multisense_ros::reconcile_stream_demand<Source>(
        {Demand{{Source::DISPARITY}, 0}, Demand{{Source::DISPARITY, Source::AUX_RECTIFIED}, 1}},
        {Source::DISPARITY});

    expect_sources(update.start, {Source::AUX_RECTIFIED});
    expect_sources(update.stop, {});
    expect_sources(update.active, {Source::DISPARITY, Source::AUX_RECTIFIED});
}

void losing_the_last_subscriber_stops_only_the_unused_source()
{
    const auto update = multisense_ros::reconcile_stream_demand<Source>(
        {Demand{{Source::DISPARITY}, 1}, Demand{{Source::AUX_RECTIFIED}, 0}},
        {Source::DISPARITY, Source::AUX_RECTIFIED});

    expect_sources(update.start, {});
    expect_sources(update.stop, {Source::AUX_RECTIFIED});
    expect_sources(update.active, {Source::DISPARITY});
}

void actual_subscriber_count_restarts_a_source_left_inactive()
{
    const auto update = multisense_ros::reconcile_stream_demand<Source>({Demand{{Source::AUX_RECTIFIED}, 3}}, {});

    expect_sources(update.start, {Source::AUX_RECTIFIED});
    expect_sources(update.active, {Source::AUX_RECTIFIED});
}

} // namespace

int main()
{
    subscriber_starts_only_the_demanded_source();
    repeating_the_same_demand_does_not_restart_the_source();
    shared_source_stays_active_while_any_topic_has_a_subscriber();
    losing_the_last_subscriber_stops_only_the_unused_source();
    actual_subscriber_count_restarts_a_source_left_inactive();
    return 0;
}
