// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_navfn_planner/navfn_planner.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace pm01_fastlio_nav {
using Pose = geometry_msgs::msg::PoseStamped;
using Point = geometry_msgs::msg::Point;
using Path = nav_msgs::msg::Path;

// A periodic plan update should not interrupt a safe route to the same goal.
class ContinuousNavfnPlanner : public nav2_navfn_planner::NavfnPlanner {
 public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    NavfnPlanner::configure(parent, name, tf, costmap);
    costmap_ros_ = std::move(costmap);
  }

  void deactivate() override {
    cached_ = Path{};
    NavfnPlanner::deactivate();
  }

  void cleanup() override {
    cached_ = Path{};
    costmap_ros_.reset();
    NavfnPlanner::cleanup();
  }

  Path createPlan(const Pose & start, const Pose & goal) override {
    const auto age = std::chrono::steady_clock::now() - planned_at_;
    const bool same_goal = goal.header.frame_id == goal_.header.frame_id &&
      goal.pose == goal_.pose && start.header.frame_id == global_frame_;
    Path remaining;
    if (!cached_.poses.empty() && same_goal && age <= std::chrono::seconds(3)) {
      remaining = traversableRemainder(start);
      if (!remaining.poses.empty() && age < std::chrono::seconds(2)) return remaining;
    }
    // A failed retry, including one after the BT clears its costmap, must not
    // revive a path that failed the cost check on this cycle.
    if (remaining.poses.empty()) cached_ = Path{};

    auto fresh = NavfnPlanner::createPlan(start, goal);
    if (!fresh.poses.empty()) {
      cached_ = fresh;
      goal_ = goal;
      planned_at_ = std::chrono::steady_clock::now();
      return fresh;
    }
    // Planning released the costmap lock. Check the current map again, even
    // if the same path passed just before makePlan.
    if (!remaining.poses.empty() &&
        std::chrono::steady_clock::now() - planned_at_ <= std::chrono::seconds(3)) {
      remaining = traversableRemainder(start);
      if (!remaining.poses.empty()) return remaining;
    }
    cached_ = Path{};
    return fresh;
  }

 private:
  Path traversableRemainder(const Pose & start) {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
    // With a noncircular footprint, defer to the configured planner.
    if (!costmap_ros_->getUseRadius() || !costmap_ros_->isCurrent()) return Path{};
    const auto & poses = cached_.poses;
    const auto & robot = start.pose.position;
    std::size_t closest = 0;
    double distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < poses.size(); ++i) {
      const auto & p = poses[i].pose.position;
      const auto d = std::hypot(p.x - robot.x, p.y - robot.y);
      if (d < distance) { closest = i; distance = d; }
    }
    if (distance > costmap_ros_->getLayeredCostmap()->getInscribedRadius()) return Path{};
    const Point * previous = &robot;
    for (std::size_t i = closest; i < poses.size(); ++i) {
      const auto & p = poses[i].pose.position;
      if (!clearSegment(*previous, p)) return Path{};
      previous = &p;
    }
    Path path;
    path.header = cached_.header;
    path.header.stamp = clock_->now();
    if (distance > costmap_->getResolution() * 0.5) path.poses.push_back(start);
    path.poses.insert(path.poses.end(), poses.begin() + closest, poses.end());
    // These poses describe the route on the map we just checked.
    for (auto & pose : path.poses) pose.header = path.header;
    return path;
  }

  bool clearSegment(const Point & a, const Point & b) const {
    unsigned int x, y, end_x, end_y;
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(b.x) || !std::isfinite(b.y) ||
        !costmap_->worldToMap(a.x, a.y, x, y) || !costmap_->worldToMap(b.x, b.y, end_x, end_y))
      return false;
    const auto clear = [this](unsigned int col, unsigned int row) {
      return costmap_->getCost(col, row) < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    };
    if (!clear(x, y)) return false;
    const double dx = b.x - a.x, dy = b.y - a.y;
    // Inflation covers known obstacles, but may leave unknown cells and the
    // outside of the grid uninflated. Check the whole swept circular footprint.
    const double radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
    if (!std::isfinite(radius) || radius <= 0.) return false;
    unsigned int left, bottom, right, top;
    if (!costmap_->worldToMap(std::min(a.x, b.x)-radius, std::min(a.y, b.y)-radius, left, bottom) ||
        !costmap_->worldToMap(std::max(a.x, b.x)+radius, std::max(a.y, b.y)+radius, right, top))
      return false;
    const double square = dx*dx + dy*dy;
    const double margin = radius + std::sqrt(0.5) * costmap_->getResolution();
    for (unsigned int row = bottom; row <= top; ++row) {
      for (unsigned int col = left; col <= right; ++col) {
        if (costmap_->getCost(col, row) < nav2_costmap_2d::LETHAL_OBSTACLE) continue;
        double cx, cy;
        costmap_->mapToWorld(col, row, cx, cy);
        const double along = square > 0. ?
          std::clamp(((cx-a.x)*dx + (cy-a.y)*dy)/square, 0., 1.) : 0.;
        if (std::hypot(cx-a.x-along*dx, cy-a.y-along*dy) <= margin) return false;
      }
    }
    const int steps = std::max(1, static_cast<int>(std::ceil(
      2 * std::hypot(dx, dy) / costmap_->getResolution())));
    for (int i = 1; i <= steps; ++i) {
      unsigned int nx = end_x, ny = end_y;
      const double fraction = static_cast<double>(i) / steps;
      if (i < steps && !costmap_->worldToMap(a.x + dx*fraction, a.y + dy*fraction, nx, ny)) return false;
      if ((nx != x || ny != y) && !clear(nx, ny)) return false;
      // A diagonal step can cross either adjacent cell before its endpoint.
      if (nx != x && ny != y && (!clear(nx, y) || !clear(x, ny))) return false;
      x = nx; y = ny;
    }
    return true;
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::chrono::steady_clock::time_point planned_at_;
  Path cached_;
  Pose goal_;
};
}  // namespace pm01_fastlio_nav
PLUGINLIB_EXPORT_CLASS(pm01_fastlio_nav::ContinuousNavfnPlanner, nav2_core::GlobalPlanner)
