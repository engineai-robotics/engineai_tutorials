// SPDX-License-Identifier: Apache-2.0
// Adapt the Nav2 rotation shim's sampling time to the asynchronous LIO TF tree.
#include <cmath>
#include <stdexcept>
#include <nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp>
#include <pluginlib/class_list_macros.hpp>

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
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,
      const geometry_msgs::msg::Twist & velocity,
      nav2_core::GoalChecker * checker) override {
    if (path_updated_) {
      auto * costmap = costmap_ros_->getCostmap();
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap->getMutex());
      std::lock_guard<std::mutex> plan_lock(mutex_);
      try {
        auto sample = getSampledPathPt();
        // A plan point is a static spatial target. Query the latest common
        // measured TF time, not /clock now ahead of the 10 Hz odometry TF.
        sample.header.stamp = rclcpp::Time(0, 0, clock_->get_clock_type());
        const auto relative = transformPoseToBaseFrame(sample);
        const double error = std::atan2(relative.position.y, relative.position.x);
        const double threshold = in_rotation_ ? angular_disengage_threshold_ : angular_dist_threshold_;
        if (std::abs(error) > threshold) {
          in_rotation_ = true;
          auto command = computeRotateToHeadingCommand(error, pose, velocity);
          // The upstream method checks the footprint for this rotation.
          last_angular_vel_ = command.twist.angular.z;
          return command;
        }
      } catch (const std::runtime_error & error) {
        RCLCPP_DEBUG(logger_, "Path alignment delegated to primary controller: %s", error.what());
      }
      path_updated_ = false;
    }
    in_rotation_ = false;
    auto command = primary_controller_->computeVelocityCommands(pose, velocity, checker);
    last_angular_vel_ = command.twist.angular.z;
    return command;
  }
};
}  // namespace pm01_fastlio_nav

PLUGINLIB_EXPORT_CLASS(pm01_fastlio_nav::LatestTfRotationShimController, nav2_core::Controller)
