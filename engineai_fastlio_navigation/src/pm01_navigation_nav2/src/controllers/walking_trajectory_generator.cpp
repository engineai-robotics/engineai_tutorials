// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <dwb_plugins/standard_traj_generator.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace pm01_fastlio_nav {

class WalkingTrajectoryGenerator : public dwb_plugins::StandardTrajectoryGenerator {
 public:
  void initialize(const nav2_util::LifecycleNode::SharedPtr & node,
                  const std::string & name) override {
    StandardTrajectoryGenerator::initialize(node, name);
    const auto parameter = name + ".walking_yaw_response_time";
    if (!node->has_parameter(parameter)) node->declare_parameter(parameter, 0.35);
    response_time_ = node->get_parameter(parameter).as_double();
    if (!std::isfinite(response_time_) || response_time_ < 0.0)
      throw std::invalid_argument(parameter + " must be a finite nonnegative time in seconds");
  }

 protected:
  nav_2d_msgs::msg::Twist2D computeNewVelocity(
      const nav_2d_msgs::msg::Twist2D & target,
      const nav_2d_msgs::msg::Twist2D & current, double dt) override {
    auto next = StandardTrajectoryGenerator::computeNewVelocity(target, current, dt);
    // Model a walking arc only. If DWB is configured to issue the first
    // simulated velocity as its command, preserve that upstream behavior.
    const bool stride = target.x != 0.0 || target.y != 0.0;
    const double yaw_change = target.theta - current.theta;
    const bool braking_yaw = current.theta * yaw_change < 0.0;
    if (!stride || !braking_yaw || response_time_ == 0.0 || limit_vel_cmd_in_traj_)
      return next;

    // DWB seeds each rollout with measured body velocity. Keep the residual
    // yaw when a candidate stride asks to straighten or reverse the turn.
    const double response_step = yaw_change * -std::expm1(-dt / response_time_);
    const double max_step = std::abs(next.theta - current.theta);
    next.theta = current.theta + std::clamp(response_step, -max_step, max_step);
    return next;
  }

 private:
  double response_time_{0.0};
};

}  // namespace pm01_fastlio_nav

PLUGINLIB_EXPORT_CLASS(pm01_fastlio_nav::WalkingTrajectoryGenerator, dwb_core::TrajectoryGenerator)
