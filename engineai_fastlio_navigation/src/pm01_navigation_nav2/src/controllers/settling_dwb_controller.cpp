#include <algorithm>
#include <cmath>
#include <limits>
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
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    const auto node = parent.lock();
    if (!node) throw std::runtime_error("Controller lifecycle node has expired");
    const auto generator = name + ".trajectory_generator_name";
    if (!node->has_parameter(generator))
      node->declare_parameter(generator, std::string("pm01_fastlio_nav::WalkingTrajectoryGenerator"));
    const auto coast = name + ".stopping_projection_time";
    if (!node->has_parameter(coast)) node->declare_parameter(coast, 0.85);
    coast_time_ = node->get_parameter(coast).as_double();
    if (!std::isfinite(coast_time_) || coast_time_ < 0.)
      throw std::invalid_argument("stopping_projection_time must be finite and nonnegative");
    DWBLocalPlanner::configure(parent, name, tf, costmap);
  }

  void setPlan(const nav_msgs::msg::Path & path) override {
    if (!path.poses.empty()) {
      const auto & goal = path.poses.back();
      const bool changed = !goal_seen_ || goal.header.frame_id != last_goal_.header.frame_id ||
        std::hypot(goal.pose.position.x-last_goal_.pose.position.x,
                   goal.pose.position.y-last_goal_.pose.position.y) > 1e-4 ||
        std::abs(angle(tf2::getYaw(goal.pose.orientation)-tf2::getYaw(last_goal_.pose.orientation))) > 1e-4;
      if (changed) { approach_braked_ = braking_ = rotation_started_ = false; quiet_.reset(); stable_pose_.reset(); }
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
    bool coast_into_goal = false;
    if (!braking_ && !xy_ok && !rotation_started_ && !approach_braked_) {
      coast_into_goal = shouldBrake(current.pose, goal.pose, plan, velocity, tolerance.position.x);
      if (coast_into_goal) approach_braked_ = true;
    }
    if (!braking_ && (coast_into_goal || (xy_ok && !rotation_started_) ||
                     (rotation_started_ && yaw_ok))) {
      braking_ = true; quiet_.reset(); stable_pose_.reset();
    }
    if (braking_) {
      const auto * policy = dynamic_cast<const GoalSettlingPolicy *>(checker);
      const bool pose_settled = policy && !policy->requiresStoppedVelocity() &&
        stable_pose_.update(rclcpp::Time(pose.header.stamp).seconds(), current.pose.x, current.pose.y,
          current.pose.theta, stopped.linear.x, stopped.angular.z);
      const bool settled = quiet_.update(clock_->now().seconds(), slow) || pose_settled;
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
    approach_braked_ = braking_ = rotation_started_ = goal_seen_ = false; quiet_.reset(); stable_pose_.reset();
    dwb_core::DWBLocalPlanner::cleanup();
  }
 private:
  bool shouldBrake(const geometry_msgs::msg::Pose2D & pose, const geometry_msgs::msg::Pose2D & goal,
      const nav_2d_msgs::msg::Path2D & plan, const geometry_msgs::msg::Twist & twist,
      double tolerance) const {
    const double dx = goal.x - pose.x, dy = goal.y - pose.y;
    const double speed = std::hypot(twist.linear.x, twist.linear.y);
    const double lookahead = std::min(0.5, speed * coast_time_);
    if (lookahead <= 0. || std::hypot(dx, dy) > lookahead + tolerance * 0.5 ||
        plan.poses.size() < 2) return false;
    const auto & end = plan.poses.back();
    if (std::hypot(end.x - goal.x, end.y - goal.y) > 1e-4) return false;

    const double vx = std::cos(pose.theta)*twist.linear.x - std::sin(pose.theta)*twist.linear.y;
    const double vy = std::sin(pose.theta)*twist.linear.x + std::cos(pose.theta)*twist.linear.y;
    const double forward = (dx*vx + dy*vy) / speed;
    const double lateral = std::abs(dx*vy - dy*vx) / speed;
    if (forward <= 0. || lateral > tolerance * 0.5) return false;

    // Use distance along the remaining plan so a nearby goal across a bend
    // does not cause a premature stop. Prefer the earlier segment on ties.
    double nearest = std::numeric_limits<double>::infinity();
    double travelled = 0., nearest_at = 0.;
    for (size_t i = 1; i < plan.poses.size(); ++i) {
      const auto & a = plan.poses[i-1];
      const auto & b = plan.poses[i];
      const double ex = b.x-a.x, ey = b.y-a.y, length = std::hypot(ex, ey);
      if (length <= 1e-9) continue;
      const double along = std::clamp(((pose.x-a.x)*ex + (pose.y-a.y)*ey)/length, 0., length);
      const double gap = std::hypot(pose.x-a.x-along*ex/length, pose.y-a.y-along*ey/length);
      if (gap < nearest) { nearest = gap; nearest_at = travelled+along; }
      travelled += length;
    }
    return nearest + travelled - nearest_at <= lookahead + tolerance * 0.5;
  }

  static double angle(double x) { return std::atan2(std::sin(x), std::cos(x)); }
  static geometry_msgs::msg::TwistStamped zero(const std_msgs::msg::Header & header) {
    geometry_msgs::msg::TwistStamped command; command.header = header; return command;
  }
  double coast_time_{0.85};
  bool approach_braked_{false};
  bool braking_{false}, rotation_started_{false}, goal_seen_{false};
  geometry_msgs::msg::PoseStamped last_goal_;
  QuietWindow quiet_{0.4};
  StablePoseWindow stable_pose_;
};
}  // namespace pm01_fastlio_nav

PLUGINLIB_EXPORT_CLASS(pm01_fastlio_nav::SettlingDWBController, nav2_core::Controller)
