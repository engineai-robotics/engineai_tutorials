#include <cmath>
#include <stdexcept>
#include <nav2_controller/plugins/stopped_goal_checker.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "pm01_navigation_nav2/quiet_window.hpp"

namespace pm01_fastlio_nav {
class SettledGoalChecker : public nav2_controller::StoppedGoalChecker, public GoalSettlingPolicy {
 public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
      const std::string & name,
      const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    nav2_controller::StoppedGoalChecker::initialize(parent, name, costmap);
    const auto node = parent.lock();
    if (!node || node->get_parameter(name + ".stateful").as_bool())
      throw std::invalid_argument("SettledGoalChecker requires stateful: false");
    if (!node->has_parameter(name + ".require_stopped"))
      node->declare_parameter<bool>(name + ".require_stopped", true);
    require_stopped_ = node->get_parameter(name + ".require_stopped").as_bool();
    clock_ = node->get_clock();
  }
  void reset() override {
    nav2_controller::StoppedGoalChecker::reset();
    quiet_.reset();
    stable_pose_.reset();
  }
  bool requiresStoppedVelocity() const override { return require_stopped_; }
  bool isGoalReached(const geometry_msgs::msg::Pose & pose,
      const geometry_msgs::msg::Pose & goal,
      const geometry_msgs::msg::Twist & velocity) override {
    const bool finite = std::isfinite(velocity.linear.x) && std::isfinite(velocity.linear.y) &&
      std::isfinite(velocity.angular.z) && std::isfinite(pose.position.x) &&
      std::isfinite(pose.position.y) && std::isfinite(goal.position.x) && std::isfinite(goal.position.y);
    const double now = clock_->now().seconds();
    const bool stopped = finite &&
      nav2_controller::StoppedGoalChecker::isGoalReached(pose, goal, velocity);
    const bool quiet = quiet_.update(now, stopped);
    if (require_stopped_) return quiet;
    if (!finite || !nav2_controller::SimpleGoalChecker::isGoalReached(pose, goal, velocity)) {
      stable_pose_.reset();
      return false;
    }
    const bool stable = stable_pose_.update(now, pose.position.x, pose.position.y,
      tf2::getYaw(pose.orientation), trans_stopped_velocity_, rot_stopped_velocity_);
    return quiet || stable;
  }
 private:
  rclcpp::Clock::SharedPtr clock_;
  bool require_stopped_{true};
  QuietWindow quiet_{0.4};
  StablePoseWindow stable_pose_;
};

}  // namespace pm01_fastlio_nav
PLUGINLIB_EXPORT_CLASS(pm01_fastlio_nav::SettledGoalChecker, nav2_core::GoalChecker)
