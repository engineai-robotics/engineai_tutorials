#include <cmath>
#include <stdexcept>
#include <nav2_controller/plugins/stopped_goal_checker.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "pm01_navigation_nav2/quiet_window.hpp"

namespace pm01_fastlio_nav {
class SettledGoalChecker : public nav2_controller::StoppedGoalChecker {
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
    nav2_controller::StoppedGoalChecker::reset(); quiet_.reset();
  }
  bool isGoalReached(const geometry_msgs::msg::Pose & pose,
      const geometry_msgs::msg::Pose & goal,
      const geometry_msgs::msg::Twist & velocity) override {
    const bool finite = std::isfinite(velocity.linear.x) && std::isfinite(velocity.linear.y) &&
      std::isfinite(velocity.angular.z) && std::isfinite(pose.position.x) &&
      std::isfinite(pose.position.y) && std::isfinite(goal.position.x) && std::isfinite(goal.position.y);
    // A position/heading arrival window may finish before the walking policy's
    // measured body sway falls below the stopped-velocity threshold. The
    // controller still commands zero in the window; this switch only changes
    // completion, and keeps the stricter simulation default intact.
    const bool reached = finite && (require_stopped_ ?
      nav2_controller::StoppedGoalChecker::isGoalReached(pose, goal, velocity) :
      nav2_controller::SimpleGoalChecker::isGoalReached(pose, goal, velocity));
    return quiet_.update(clock_->now().seconds(), reached);
  }
 private:
  rclcpp::Clock::SharedPtr clock_;
  bool require_stopped_{true};
  QuietWindow quiet_{0.4};
};

}  // namespace pm01_fastlio_nav
PLUGINLIB_EXPORT_CLASS(pm01_fastlio_nav::SettledGoalChecker, nav2_core::GoalChecker)
