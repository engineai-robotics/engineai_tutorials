#include "pm01_navigation_nav2/navigation_ready_wait.hpp"
#include "pm01_navigation_nav2/navigation_readiness.hpp"

#include <cmath>
#include "behaviortree_cpp_v3/bt_factory.h"

namespace pm01_navigation
{
NavigationReadyWait::NavigationReadyWait(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::DecoratorNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  getInput("timeout", timeout_);
  getInput("heartbeat_timeout", heartbeat_timeout_);
  if (!std::isfinite(timeout_) || timeout_ <= 0. ||
    !std::isfinite(heartbeat_timeout_) || heartbeat_timeout_ <= 0.)
  {
    throw std::invalid_argument("Invalid navigation readiness wait timeout");
  }
  group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(group_, node_->get_node_base_interface());
  rclcpp::SubscriptionOptions options;
  options.callback_group = group_;
  subscription_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/pm01/navigation_readiness", rclcpp::QoS(1).transient_local(),
    [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
      ready_ = false;
      received_ = Clock::now();
      reason_ = "invalid_readiness_status";
      if (msg->status.size() == 1) {
        const auto & status = msg->status.front();
        reason_ = status.message;
        for (const auto & value : status.values) {
          if (value.key == "ready") {
            const auto fresh = statusStampFresh(node_->now().seconds(),
              rclcpp::Time(msg->header.stamp).seconds(), heartbeat_timeout_);
            ready_ = status.level == 0 && value.value == "true" && fresh;
            if (!fresh) {reason_ = "readiness_status_stale";}
          }
        }
      }
    }, options);
  publisher_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/pm01/navigation_wait_status", rclcpp::QoS(1).transient_local());
}

BT::PortsList NavigationReadyWait::providedPorts()
{
  return {BT::InputPort<double>("timeout", 30., "Maximum continuous observation outage in seconds"),
    BT::InputPort<double>("heartbeat_timeout", .5, "Readiness publisher heartbeat timeout")};
}

BT::NodeStatus NavigationReadyWait::tick()
{
  executor_.spin_some();
  if (failed_) {return BT::NodeStatus::FAILURE;}
  const auto new_goal = !active_;
  active_ = true;
  const auto now = Clock::now();
  const auto fresh = received_ &&
    std::chrono::duration<double>(now - *received_).count() <= heartbeat_timeout_;
  if (!fresh) {reason_ = "readiness_status_stale";}
  // The deadline remains authoritative even if readiness returns on this tick.
  if (wait_started_ && std::chrono::duration<double>(now - *wait_started_).count() >= timeout_) {
    haltChild();
    failed_ = true;
    report("timed_out");
    RCLCPP_ERROR(node_->get_logger(), "Navigation goal aborted after %.1f seconds waiting: %s",
      timeout_, reason_.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!fresh || !ready_) {
    if (!wait_started_) {
      wait_started_ = now;
      wait_id_ = name() + ":" + std::to_string(now.time_since_epoch().count());
      // Cancel planning, control AND recovery children, retaining the action goal.
      haltChild();
      report("waiting");
    } else if (!last_report_ || std::chrono::duration<double>(now - *last_report_).count() >= 1.) {
      report("waiting");
    }
    return BT::NodeStatus::RUNNING;
  }
  if (wait_started_) {
    report("resumed");
    wait_started_.reset();
  } else if (new_goal) {
    report("running");
  }
  return child_node_->executeTick();
}

void NavigationReadyWait::halt()
{
  if (wait_started_ && !failed_) {report("canceled");}
  wait_started_.reset();
  last_report_.reset();
  failed_ = false;
  active_ = false;
  wait_id_.clear();
  BT::DecoratorNode::halt();
  resetStatus();
}

void NavigationReadyWait::report(const std::string & state)
{
  last_report_ = Clock::now();
  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.header.stamp = node_->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "pm01/navigation_wait";
  status.level = state == "timed_out" ? 2 : (state == "waiting" ? 1 : 0);
  status.message = state;
  auto add = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key; item.value = value; status.values.push_back(item);
    };
  add("state", state);
  add("reason", reason_);
  add("wait_id", wait_id_);
  add("wait_elapsed_sec", std::to_string(wait_started_ ?
    std::chrono::duration<double>(Clock::now() - *wait_started_).count() : 0.));
  msg.status.push_back(status);
  publisher_->publish(msg);
}
}  // namespace pm01_navigation

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pm01_navigation::NavigationReadyWait>("NavigationReadyWait");
}
