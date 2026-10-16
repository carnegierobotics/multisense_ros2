// Copyright 2026 Carnegie Robotics, LLC
// SPDX-License-Identifier: BSD-3-Clause

#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>

namespace
{

class MultiSenseComponent : public ::testing::Test
{
protected:
    void SetUp() override
    {
        context_ = std::make_shared<rclcpp::Context>();
        context_->init(0, nullptr);
        options_.context(context_).use_global_arguments(false);
        manager_ = std::make_shared<rclcpp_components::ComponentManager>(
            std::weak_ptr<rclcpp::Executor>{}, "component_test", options_);
        factory_ = manager_->create_component_factory(
            {"multisense_ros::MultiSense", MULTISENSE_COMPONENT_LIBRARY});
        ASSERT_NE(factory_, nullptr);
    }

    void TearDown() override
    {
        factory_.reset();
        manager_.reset();
        context_->shutdown("test complete");
    }

    std::shared_ptr<rclcpp::Context> context_;
    rclcpp::NodeOptions options_;
    std::shared_ptr<rclcpp_components::ComponentManager> manager_;
    std::shared_ptr<rclcpp_components::NodeFactory> factory_;
};

TEST_F(MultiSenseComponent, RejectsReconnectWithoutShuttingDownContainer)
{
    options_.parameter_overrides({rclcpp::Parameter("reconnect", true)});
    EXPECT_THROW(factory_->create_node_instance(options_), std::invalid_argument);
    EXPECT_TRUE(context_->is_valid());
}

TEST_F(MultiSenseComponent, ValidatesInitializationParameterOverrides)
{
    options_.parameter_overrides({rclcpp::Parameter("sensor_mtu", 1499)});
    EXPECT_THROW(factory_->create_node_instance(options_),
                 rclcpp::exceptions::InvalidParameterValueException);
    EXPECT_TRUE(context_->is_valid());
}

TEST_F(MultiSenseComponent, FailedConnectionAllowsAnotherLoadAttempt)
{
    options_.parameter_overrides({rclcpp::Parameter("sensor_ip", "invalid-ip-address")});
    EXPECT_THROW(factory_->create_node_instance(options_), std::runtime_error);
    EXPECT_TRUE(context_->is_valid());

    options_.parameter_overrides({rclcpp::Parameter("reconnect", true)});
    EXPECT_THROW(factory_->create_node_instance(options_), std::invalid_argument);
    EXPECT_TRUE(context_->is_valid());
}

} // namespace
