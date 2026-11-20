#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace pm01_fastlio_nav {
// Sample a fixed arc length beyond the robot's projection on the polyline.
// Splitting a segment into more poses leaves the steering reference unchanged.
inline std::size_t alignmentWaypoint(const std::vector<geometry_msgs::msg::PoseStamped> & path,
    double lookahead, const geometry_msgs::msg::Vector3 * robot = nullptr,
    geometry_msgs::msg::Point * target = nullptr) {
  if (path.empty()) throw std::invalid_argument("Cannot align to an empty path");
  const auto interpolate = [](const auto & a, const auto & b, double t) {
    geometry_msgs::msg::Point p;
    p.x = a.x + (b.x-a.x)*t;
    p.y = a.y + (b.y-a.y)*t;
    p.z = a.z + (b.z-a.z)*t;
    return p;
  };
  auto start = path.front().pose.position;
  std::size_t next = 1;
  if (robot) {
    double nearest = std::numeric_limits<double>::infinity();
    for (std::size_t i = 1; i < path.size(); ++i) {
      const auto & a = path[i-1].pose.position;
      const auto & b = path[i].pose.position;
      const double dx = b.x-a.x, dy = b.y-a.y, sq = dx*dx+dy*dy;
      if (sq == 0.) continue;
      const double t = std::clamp(((robot->x-a.x)*dx + (robot->y-a.y)*dy)/sq, 0., 1.);
      const auto p = interpolate(a, b, t);
      const double ex = p.x-robot->x, ey = p.y-robot->y, distance = ex*ex+ey*ey;
      if (distance < nearest) {
        nearest = distance;
        start = p;
        next = i;
      }
    }
  }
  for (std::size_t i = next; i < path.size(); ++i) {
    const auto & end = path[i].pose.position;
    const double length = std::hypot(end.x-start.x, end.y-start.y);
    if (length > 0. && lookahead <= length) {
      if (target) *target = interpolate(start, end, std::max(0., lookahead)/length);
      return i;
    }
    lookahead -= length;
    start = end;
  }
  if (target) *target = path.back().pose.position;
  return path.size()-1;
}

constexpr double alignment_coast_seconds = 0.5;

inline bool needsTurnBrake(double error, double measured, double tolerance, double stopped_rate) {
  const double predicted = error - measured * alignment_coast_seconds;
  return std::abs(measured) > stopped_rate &&
    (std::abs(error) <= tolerance ||
     (std::abs(predicted) > tolerance &&
      (error * predicted <= 0.0 || std::abs(predicted) > std::abs(error))));
}

class TurnHandoff {
 public:
  enum class Command { turn, brake, track };

  void reset() { *this = TurnHandoff{}; }

  void brake() { braking_ = true; }

  Command update(double error, double measured, double tolerance,
                 double stopped_rate, double stamp, double yaw = 0.0) {
    constexpr double settle_seconds = 0.4;
    if (!std::isfinite(error) || !std::isfinite(measured) || !std::isfinite(stamp) || !std::isfinite(yaw)) {
      reset();
      braking_ = true;
      return Command::brake;
    }
    const bool aligned = std::abs(error) <= tolerance;
    const int direction = error < 0.0 ? -1 : 1;
    const bool crossing = direction_ != 0 && direction != direction_;
    const bool coast_to_target = direction * measured > stopped_rate &&
      direction * measured * alignment_coast_seconds >= std::abs(error) - tolerance;
    if (aligned || crossing || coast_to_target) braking_ = true;
    if (braking_) {
      if (aligned != was_aligned_ || stamp < last_stamp_ || stamp - last_stamp_ > settle_seconds)
        quiet_since_.reset();
      was_aligned_ = aligned;
      last_stamp_ = stamp;
      if (std::abs(measured) > stopped_rate) quiet_since_.reset();
      else if (!quiet_since_ ||
          std::abs(std::remainder(yaw - quiet_yaw_, 2.0 * std::acos(-1.0))) > stopped_rate * settle_seconds) {
        quiet_since_ = stamp;
        quiet_yaw_ = yaw;
      }
      if (!quiet_since_ || stamp - *quiet_since_ < settle_seconds) return Command::brake;
      reset();
      if (aligned) return Command::track;
    }
    direction_ = direction;
    return Command::turn;
  }

 private:
  int direction_{0};
  bool braking_{false}, was_aligned_{false};
  double last_stamp_{0.0}, quiet_yaw_{0.0};
  std::optional<double> quiet_since_;
};

inline double alignmentSpeed(double angle, double measured_rate, double max_rate, double acceleration,
                             double min_rate = 0.0, double tolerance = 0.0) {
  if (std::abs(angle) <= tolerance) return 0.0;
  const double direction = std::copysign(1.0, angle);
  const double coast = std::max(0.0, direction * measured_rate) * alignment_coast_seconds;
  const double remaining = std::max(0.0, std::abs(angle) - coast);
  return direction * std::min(max_rate, std::max(min_rate, std::sqrt(2.0 * acceleration * remaining)));
}

// Lateral corrections need a walking stride. A large lateral component alone
// must not make a stationary or sub-threshold longitudinal command executable.
inline bool reachableWalkingCommand(double vx, double vy, double wz,
                                    double min_linear, double min_angular,
                                    double epsilon = 0.001) {
  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) return false;
  const bool stationary_xy = std::hypot(vx, vy) <= epsilon;
  return (stationary_xy || std::abs(vx) >= min_linear) &&
         (std::abs(wz) <= epsilon || std::abs(wz) >= min_angular);
}
}  // namespace pm01_fastlio_nav
