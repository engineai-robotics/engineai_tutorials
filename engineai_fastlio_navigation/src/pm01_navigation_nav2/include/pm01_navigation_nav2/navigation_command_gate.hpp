#pragma once

#include "rclcpp/rclcpp.hpp"

namespace pm01_navigation
{
std::shared_ptr<rclcpp::Node> make_navigation_command_gate(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
}  // namespace pm01_navigation
