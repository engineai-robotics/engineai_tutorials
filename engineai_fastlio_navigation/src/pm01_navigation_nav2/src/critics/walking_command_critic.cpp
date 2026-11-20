#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <dwb_core/exceptions.hpp>
#include <dwb_core/trajectory_critic.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "pm01_navigation_nav2/walking_command_limits.hpp"

namespace pm01_fastlio_nav {
class WalkingCommandCritic : public dwb_core::TrajectoryCritic {
 public:
  void onInit() override {
    auto node = node_.lock();
    if (!node) throw std::runtime_error("WalkingCommand critic has no node");
    const auto prefix = dwb_plugin_name_ + "." + name_ + ".";
    auto parameter = [&](const std::string & key, double fallback) {
      if (!node->has_parameter(prefix + key)) node->declare_parameter(prefix + key, fallback);
      return node->get_parameter(prefix + key).as_double();
    };
    min_linear_ = parameter("min_linear_speed", 0.25);
    min_angular_ = parameter("min_angular_speed", 0.62);
    epsilon_ = parameter("zero_epsilon", 0.001);
    if (!std::isfinite(min_linear_) || !std::isfinite(min_angular_) ||
        !std::isfinite(epsilon_) || min_linear_ <= 0 || min_angular_ <= 0 ||
        epsilon_ < 0 || epsilon_ >= std::min(min_linear_, min_angular_))
      throw std::invalid_argument("Invalid walking command limits");
  }

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override {
    const auto & v = trajectory.velocity;
    if (!reachableWalkingCommand(v.x, v.y, v.theta, min_linear_, min_angular_, epsilon_))
      throw dwb_core::IllegalTrajectoryException(name_, "Velocity component below walking response limit");
    // Feasibility filter only. Other critics still score collision, path and
    // goal costs using this exact sampled velocity; no command is amplified.
    return 0.0;
  }
 private:
  double min_linear_{0.25}, min_angular_{0.62}, epsilon_{0.001};
};
}  // namespace pm01_fastlio_nav

PLUGINLIB_EXPORT_CLASS(pm01_fastlio_nav::WalkingCommandCritic, dwb_core::TrajectoryCritic)
