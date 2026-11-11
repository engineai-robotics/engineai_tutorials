#include <cmath>
#include <stdexcept>
#include <dwb_core/dwb_local_planner.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "pm01_navigation_nav2/quiet_window.hpp"

namespace pm01_fastlio_nav {
// Keep DWB's planning and collision checks for every nonzero command. At the
// endpoint, only insert zero commands while the walking policy settles. This
// also releases RotateToGoal's XY latch after a coast outside the position
// window, rather than waiting for a progress-checker timeout/recovery.
class SettlingDWBController : public dwb_core::DWBLocalPlanner {
 public:
  void setPlan(const nav_msgs::msg::Path & path) override {
    if (!path.poses.empty()) {
      const auto & goal = path.poses.back();
      const bool changed = !goal_seen_ || goal.header.frame_id != last_goal_.header.frame_id ||
        std::hypot(goal.pose.position.x-last_goal_.pose.position.x,
                   goal.pose.position.y-last_goal_.pose.position.y) > 1e-4 ||
        std::abs(angle(tf2::getYaw(goal.pose.orientation)-tf2::getYaw(last_goal_.pose.orientation))) > 1e-4;
      if (changed) { braking_ = rotation_started_ = false; quiet_.reset(); }
      last_goal_ = goal; goal_seen_ = true;
    }
    dwb_core::DWBLocalPlanner::setPlan(path);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,
      const geometry_msgs::msg::Twist & velocity,
      nav2_core::GoalChecker * checker) override {
    if (!checker) throw std::runtime_error("Endpoint settling requires a goal checker");
    geometry_msgs::msg::Pose tolerance;
    geometry_msgs::msg::Twist stopped;
    if (!checker->getTolerances(tolerance, stopped))
      throw std::runtime_error("No endpoint tolerances");
    nav_2d_msgs::msg::Pose2DStamped current, goal;
    nav_2d_msgs::msg::Path2D plan;
    current.header = pose.header;
    current.pose.x = pose.pose.position.x; current.pose.y = pose.pose.position.y;
    current.pose.theta = tf2::getYaw(pose.pose.orientation);
    prepareGlobalPlan(current, plan, goal, false);
    const bool xy_ok = std::hypot(current.pose.x-goal.pose.x,
                                  current.pose.y-goal.pose.y) <= tolerance.position.x;
    const bool yaw_ok = std::abs(angle(current.pose.theta-goal.pose.theta)) <=
      std::abs(tf2::getYaw(tolerance.orientation));
    const bool slow = std::hypot(velocity.linear.x, velocity.linear.y) <= stopped.linear.x &&
      std::abs(velocity.angular.z) <= stopped.angular.z;

    // Finish a turn before evaluating XY again: interrupting a turn whenever
    // the pelvis shifts sideways makes the biped alternate position/yaw
    // corrections indefinitely. Stop and reassess both after heading arrival.
    if (!braking_ && ((xy_ok && !rotation_started_) ||
                     (rotation_started_ && yaw_ok))) {
      braking_ = true; quiet_.reset();
    }
    if (braking_) {
      const bool settled = quiet_.update(clock_->now().seconds(), slow);
      // Hold at a complete pose until the goal checker has also observed its
      // continuous quiet window. If coasting left the pose, re-enter DWB.
      if (!settled || (xy_ok && yaw_ok)) return zero(pose.header);
      braking_ = false; rotation_started_ = xy_ok;
      quiet_.reset();
      for (auto & critic : critics_) critic->reset();
    }
    return dwb_core::DWBLocalPlanner::computeVelocityCommands(pose, velocity, checker);
  }

  void cleanup() override {
    braking_ = rotation_started_ = goal_seen_ = false; quiet_.reset();
    dwb_core::DWBLocalPlanner::cleanup();
  }
 private:
  static double angle(double x) { return std::atan2(std::sin(x), std::cos(x)); }
  static geometry_msgs::msg::TwistStamped zero(const std_msgs::msg::Header & header) {
    geometry_msgs::msg::TwistStamped command; command.header = header; return command;
  }
  bool braking_{false}, rotation_started_{false}, goal_seen_{false};
  geometry_msgs::msg::PoseStamped last_goal_;
  QuietWindow quiet_{0.4};
};
}  // namespace pm01_fastlio_nav

PLUGINLIB_EXPORT_CLASS(pm01_fastlio_nav::SettlingDWBController, nav2_core::Controller)
