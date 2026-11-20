#pragma once

#include <stdexcept>
#include <rclcpp/rclcpp.hpp>

struct SensorIoOptions {
    rclcpp::CallbackGroup::SharedPtr group;
    rclcpp::CallbackGroup::SharedPtr imu_group;
    rclcpp::SubscriptionOptions subscription;
    rclcpp::SubscriptionOptions imu_subscription;
    rclcpp::QoS imu_qos;
};

inline SensorIoOptions make_sensor_io_options(rclcpp::Node & node, bool concurrent, int64_t imu_depth)
{
    if (imu_depth < 10 || imu_depth > 2000)
        throw std::invalid_argument("common.imu_queue_size must lie in [10, 2000]");
    SensorIoOptions result{nullptr, nullptr, {}, {}, rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(imu_depth)))};
    if (concurrent) {
        // LiDAR preprocessing must not delay high-rate IMU reception. Each
        // stream remains ordered; their shared buffers use the queue mutex.
        result.group = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        result.imu_group = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        result.subscription.callback_group = result.group;
        result.imu_subscription.callback_group = result.imu_group;
    }
    return result;
}
