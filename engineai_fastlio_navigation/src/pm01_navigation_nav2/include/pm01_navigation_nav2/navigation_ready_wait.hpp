#pragma once

#include <chrono>
#include <optional>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pm01_navigation
{
class NavigationReadyWait : public BT::DecoratorNode
{
public:
  NavigationReadyWait(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  void halt() override;

private:
  using Clock = std::chrono::steady_clock;
  void report(const std::string & state);
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  bool ready_ = false;
  bool failed_ = false;
  bool active_ = false;
  std::string reason_ = "waiting_for_readiness";
  std::string wait_id_;
  double timeout_ = 30.;
  double heartbeat_timeout_ = .5;
  std::optional<Clock::time_point> received_;
  std::optional<Clock::time_point> wait_started_;
  std::optional<Clock::time_point> last_report_;
};
}  // namespace pm01_navigation
