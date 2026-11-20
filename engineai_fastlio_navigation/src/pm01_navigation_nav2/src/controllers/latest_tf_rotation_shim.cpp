// SPDX-License-Identifier: Apache-2.0
// Adapt the Nav2 rotation shim's sampling time to the asynchronous LIO TF tree.
#include <cmath>
#include <algorithm>
#include <optional>
#include <stdexcept>
#include <action_msgs/msg/goal_status_array.hpp>
#include <nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "pm01_navigation_nav2/quiet_window.hpp"
#include "pm01_navigation_nav2/walking_command_limits.hpp"

namespace pm01_fastlio_nav {
class LatestTfRotationShimController
    : public nav2_rotation_shim_controller::RotationShimController {
 public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    RotationShimController::configure(parent, name, tf, costmap);
    if (rotate_to_goal_heading_)
      throw std::invalid_argument("Use the primary settling controller for goal heading");
    const auto node = parent.lock();
    min_rate_param_ = name + ".min_in_place_angular_vel";
    max_rate_param_ = name + ".rotate_to_heading_angular_vel";
    if (!node->has_parameter(min_rate_param_))
      node->declare_parameter(min_rate_param_, std::min(0.56, rotate_to_heading_angular_vel_));
    if (!validRates(node->get_parameter(min_rate_param_).as_double(), rotate_to_heading_angular_vel_))
      throw std::invalid_argument("In-place turn rates require 0 <= minimum <= maximum");
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    for (const auto * topic : {"navigate_to_pose/_action/status", "navigate_through_poses/_action/status"}) {
      goal_subscriptions_.push_back(parent.lock()->create_subscription<Statuses>(topic, qos,
        [this](Statuses::ConstSharedPtr status) { observeGoals(*status); }));
    }
  }

  void activate() override {
    const auto node = node_.lock();
    if (!validRates(node->get_parameter(min_rate_param_).as_double(),
                    node->get_parameter(max_rate_param_).as_double()))
      throw std::invalid_argument("In-place turn rates require 0 <= minimum <= maximum");
    RotationShimController::activate();
    // Register after the upstream updater, so invalid pairs are rejected before
    // it consumes a changed maximum. Read both proposed values for atomic sets.
    validate_rates_ = node->add_on_set_parameters_callback([this](const auto & changes) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      const auto node = node_.lock();
      auto minimum = node->get_parameter(min_rate_param_).as_double();
      auto maximum = node->get_parameter(max_rate_param_).as_double();
      for (const auto & p : changes) {
        if (p.get_name() != min_rate_param_ && p.get_name() != max_rate_param_) continue;
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result.successful = false;
          break;
        }
        if (p.get_name() == min_rate_param_) minimum = p.as_double();
        else maximum = p.as_double();
      }
      result.successful = result.successful && validRates(minimum, maximum);
      if (!result.successful) result.reason = "In-place turn rates require 0 <= minimum <= maximum";
      return result;
    });
  }

  void deactivate() override {
    validate_rates_.reset();
    RotationShimController::deactivate();
    std::lock_guard<std::mutex> lock(mutex_);
    handoff_.reset();
    after_pure_turn_ = false;
    path_updated_ = true;
    in_rotation_ = false;
  }

  void setPlan(const nav_msgs::msg::Path & path) override {
    std::lock_guard<std::mutex> lock(mutex_);
    bool align = path_updated_ || current_path_.poses.empty();
    if (!active_goal_ && !current_path_.poses.empty() && !path.poses.empty()) {
      const auto & a = current_path_.poses.back().pose;
      const auto & b = path.poses.back().pose;
      const bool new_goal = current_path_.header.frame_id != path.header.frame_id ||
        std::hypot(a.position.x-b.position.x, a.position.y-b.position.y) > 1e-4 ||
        std::abs(angles::shortest_angular_distance(tf2::getYaw(a.orientation), tf2::getYaw(b.orientation))) > 1e-4;
      if (new_goal) { handoff_.reset(); in_rotation_ = after_pure_turn_ = false; align = true; }
    }
    RotationShimController::setPlan(path);
    path_updated_ = align;
  }

  void cleanup() override {
    goal_subscriptions_.clear();
    active_goal_.reset();
    stationary_pose_.reset();
    handoff_.reset();
    current_path_.poses.clear();
    RotationShimController::cleanup();
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,
      const geometry_msgs::msg::Twist & velocity,
      nav2_core::GoalChecker * checker) override {
    // Read the committed value before taking the plan mutex used by parameter callbacks.
    const double minimum = node_.lock()->get_parameter(min_rate_param_).as_double();
    geometry_msgs::msg::Pose goal_tolerance;
    geometry_msgs::msg::Twist stopped;
    const auto * tolerance = checker && checker->getTolerances(goal_tolerance, stopped)
      ? &goal_tolerance : nullptr;
    const double stopped_rate = std::isfinite(stopped.angular.z) && stopped.angular.z > 0.
      ? stopped.angular.z : 0.1;
    const double stopped_xy = tolerance && stopped.linear.x > 0. ? stopped.linear.x : 0.05;
    const bool walking = std::hypot(velocity.linear.x, velocity.linear.y) > stopped_xy;
    {
      auto * costmap = costmap_ros_->getCostmap();
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap->getMutex());
      std::lock_guard<std::mutex> plan_lock(mutex_);
      const bool stationary = tolerance && stationary_pose_.update(
        rclcpp::Time(pose.header.stamp).seconds(), pose.pose.position.x, pose.pose.position.y,
        tf2::getYaw(pose.pose.orientation), stopped.linear.x, stopped.angular.z);
      if (path_updated_ || stationary || after_pure_turn_) {
        const auto heading = alignmentHeading(tolerance, angular_disengage_threshold_);
        const bool starting = path_updated_ || (after_pure_turn_ && !walking);
        const bool coast = (path_updated_ || after_pure_turn_) &&
          needsTurnBrake(heading.error, velocity.angular.z, heading.tolerance, stopped_rate);
        const double entry = heading.endpoint ? heading.tolerance : angular_dist_threshold_;
        const bool realign = (starting || (stationary && heading.side_or_behind)) &&
          std::abs(heading.error) > entry;
        if (coast) handoff_.brake();
        if (in_rotation_ || coast || realign) {
          path_updated_ = true;
          if (auto alignment = alignCommand(heading, pose, velocity.angular.z, minimum, stopped.angular.z))
            return *alignment;
        }
        path_updated_ = false;
      }
      in_rotation_ = false;
    }
    auto command = primary_controller_->computeVelocityCommands(pose, velocity, checker);
    const auto & twist = command.twist;
    const bool translating = std::hypot(twist.linear.x, twist.linear.y) > 1e-6;
    const bool pure_turn = !translating && std::abs(twist.angular.z) > 1e-6;
    // Check the next stride even when DWB itself initiated the turn. In
    // particular a primary-controller stop between turn and stride does not
    // mean the body has stopped rotating.
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(
      *costmap_ros_->getCostmap()->getMutex(), std::defer_lock);
    if (pure_turn) costmap_lock.lock();
    std::lock_guard<std::mutex> plan_lock(mutex_);
    if (translating) after_pure_turn_ = false;
    if (pure_turn) {
      after_pure_turn_ = true;
      const auto heading = alignmentHeading(tolerance, angular_disengage_threshold_);
      const double entry = heading.endpoint ? heading.tolerance : angular_dist_threshold_;
      if ((heading.endpoint || !walking) && std::abs(twist.angular.z) < minimum &&
          std::abs(heading.error) > entry) {
        path_updated_ = true;
        if (auto alignment = alignCommand(heading, pose, velocity.angular.z, minimum, stopped.angular.z))
          return *alignment;
      }
    }
    last_angular_vel_ = command.twist.angular.z;
    return command;
  }

 private:
  struct Heading { double error, tolerance; bool side_or_behind, endpoint; };
  StablePoseWindow stationary_pose_;
  TurnHandoff handoff_;
  bool after_pure_turn_{false};
  std::string min_rate_param_, max_rate_param_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr validate_rates_;
  static bool validRates(double minimum, double maximum) {
    return std::isfinite(minimum) && std::isfinite(maximum) && minimum >= 0.0 && minimum <= maximum;
  }
  using Statuses = action_msgs::msg::GoalStatusArray;
  using GoalId = decltype(action_msgs::msg::GoalInfo().goal_id.uuid);
  std::optional<GoalId> active_goal_;
  std::vector<rclcpp::Subscription<Statuses>::SharedPtr> goal_subscriptions_;

  void observeGoals(const Statuses & message) {
    using Status = action_msgs::msg::GoalStatus;
    const Status * newest = nullptr;
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & s : message.status_list) {
      if (s.status == Status::STATUS_EXECUTING || s.status == Status::STATUS_ACCEPTED) {
        if (!newest || rclcpp::Time(s.goal_info.stamp) >= rclcpp::Time(newest->goal_info.stamp))
          newest = &s;
      } else if (active_goal_ == s.goal_info.goal_id.uuid) {
        active_goal_.reset();
      }
    }
    if (newest && active_goal_ != newest->goal_info.goal_id.uuid) {
      active_goal_ = newest->goal_info.goal_id.uuid;
      handoff_.reset();
      after_pure_turn_ = false;
      stationary_pose_.reset();
      path_updated_ = true;
      in_rotation_ = false;
    }
  }

  Heading alignmentHeading(const geometry_msgs::msg::Pose * tolerance, double path_tolerance) {
    const auto & points = current_path_.poses;
    if (points.empty()) throw nav2_core::PlannerException("No path to align");
    const auto to_base = tf_->lookupTransform(costmap_ros_->getBaseFrameID(),
      current_path_.header.frame_id, tf2::TimePointZero);
    geometry_msgs::msg::Pose local;
    tf2::doTransform(points.back().pose, local, to_base);
    if (tolerance && std::hypot(local.position.x, local.position.y) <= tolerance->position.x) {
      return {tf2::getYaw(local.orientation), std::abs(tf2::getYaw(tolerance->orientation)), false, true};
    }
    // The robot may already have traversed the beginning of this same plan.
    tf2::Transform path_in_base;
    tf2::fromMsg(to_base.transform, path_in_base);
    const auto robot = tf2::toMsg(path_in_base.inverse().getOrigin());
    geometry_msgs::msg::Point target;
    alignmentWaypoint(points, forward_sampling_distance_, &robot, &target);
    const auto ahead = path_in_base * tf2::Vector3(target.x, target.y, target.z);
    return {std::atan2(ahead.y(), ahead.x()), path_tolerance,
            ahead.x() < std::abs(ahead.y()), false};
  }

  std::optional<geometry_msgs::msg::TwistStamped> alignCommand(
      Heading heading, const geometry_msgs::msg::PoseStamped & pose, double measured,
      double minimum, double stopped_rate) {
    // Goal checkers which do not supply a stopped tolerance use the PM01 default.
    if (!std::isfinite(stopped_rate) || stopped_rate <= 0.0) stopped_rate = 0.1;
    const auto action = handoff_.update(heading.error, measured, heading.tolerance, stopped_rate,
      rclcpp::Time(pose.header.stamp).seconds(), tf2::getYaw(pose.pose.orientation));
    if (action == TurnHandoff::Command::track) return std::nullopt;
    in_rotation_ = true;
    geometry_msgs::msg::TwistStamped command;
    command.header = pose.header;
    if (action == TurnHandoff::Command::turn) command = turnCommand(heading, pose, measured, minimum);
    last_angular_vel_ = command.twist.angular.z;
    return command;
  }

  geometry_msgs::msg::TwistStamped turnCommand(Heading heading,
      const geometry_msgs::msg::PoseStamped & pose, double measured, double minimum) {
    geometry_msgs::msg::TwistStamped command;
    command.header = pose.header;
    if (!std::isfinite(measured)) return command;
    const double target = alignmentSpeed(heading.error, measured, rotate_to_heading_angular_vel_,
      max_angular_accel_, minimum, heading.tolerance);
    const bool have_previous = last_angular_vel_ != std::numeric_limits<double>::max();
    double previous = closed_loop_ ? measured : (have_previous ? last_angular_vel_ : 0.0);
    // While starting from rest, allow the command ramp to cross the gait's
    // deadband even when measured yaw has not started to respond yet.
    if (closed_loop_ && have_previous && measured*target >= 0.0 && last_angular_vel_*target > 0.0 &&
        std::abs(measured) < minimum) {
      previous = std::copysign(std::max(std::abs(measured),
        std::min(std::abs(last_angular_vel_), minimum)), target);
    }
    const double step = max_angular_accel_ * control_duration_;
    command.twist.angular.z = std::clamp(std::clamp(target, previous-step, previous+step),
      -rotate_to_heading_angular_vel_, rotate_to_heading_angular_vel_);
    // Test the footprint of the actual ramped command over the lookahead.
    // This also checks both turn directions near the angular handoff threshold.
    const auto footprint = costmap_ros_->getRobotFootprint();
    const double yaw = tf2::getYaw(pose.pose.orientation);
    const int steps = std::max(1, static_cast<int>(std::ceil(simulate_ahead_time_/control_duration_)));
    for (int i = 1; i <= steps; ++i) {
      const double dt = std::min(i*control_duration_, simulate_ahead_time_);
      const auto cost = collision_checker_->footprintCostAtPose(pose.pose.position.x,
        pose.pose.position.y, yaw + command.twist.angular.z*dt, footprint);
      if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE)
        throw nav2_core::PlannerException("In-place turn blocked by the local costmap");
    }
    return command;
  }
};
}  // namespace pm01_fastlio_nav

PLUGINLIB_EXPORT_CLASS(pm01_fastlio_nav::LatestTfRotationShimController, nav2_core::Controller)
