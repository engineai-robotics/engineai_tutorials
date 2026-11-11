#pragma once

#include <stdexcept>
#include <rclcpp/rclcpp.hpp>

struct SensorIoOptions {
    rclcpp::CallbackGroup::SharedPtr group;
    rclcpp::SubscriptionOptions subscription;
    rclcpp::QoS imu_qos;
};

inline SensorIoOptions make_sensor_io_options(rclcpp::Node & node, bool concurrent, int64_t imu_depth)
{
    if (imu_depth < 10 || imu_depth > 2000)
        throw std::invalid_argument("common.imu_queue_size must lie in [10, 2000]");
    SensorIoOptions result{nullptr, {}, rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(imu_depth)))};
    if (concurrent) {
        // LiDAR preprocessing and IMU callbacks stay serialized with each other.
        // Filter, map and publication callbacks keep the node's default group.
        result.group = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        result.subscription.callback_group = result.group;
    }
    return result;
}
