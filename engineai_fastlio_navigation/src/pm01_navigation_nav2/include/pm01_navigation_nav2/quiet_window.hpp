#pragma once
#include <cmath>
#include <deque>

namespace pm01_fastlio_nav {
class GoalSettlingPolicy {
 public:
  virtual ~GoalSettlingPolicy() = default;
  virtual bool requiresStoppedVelocity() const = 0;
};

// Use the extent of all observed poses, including excursions between samples.
class StablePoseWindow {
 public:
  void reset() { samples_.clear(); }
  bool update(double t, double x, double y, double yaw, double linear, double angular) {
    constexpr double seconds = 0.4;
    if (!std::isfinite(t+x+y+yaw+linear+angular) || linear < 0 || angular < 0) {
      reset();
      return false;
    }
    if (!samples_.empty() && (t < samples_.back().t || t-samples_.back().t > seconds))
      reset();
    if (samples_.empty() || t != samples_.back().t) samples_.push_back({t,x,y,yaw});
    while (samples_.size() > 1 && samples_[1].t <= t-seconds) samples_.pop_front();
    for (const auto & p : samples_) {
      if (std::hypot(x-p.x,y-p.y) > seconds*linear ||
          std::abs(std::remainder(yaw-p.yaw, 2.0*std::acos(-1.0))) > seconds*angular) {
        reset();
        samples_.push_back({t,x,y,yaw});
        return false;
      }
    }
    return t-samples_.front().t >= seconds;
  }
 private:
  struct Sample { double t, x, y, yaw; };
  std::deque<Sample> samples_;
};

class QuietWindow {
 public:
  explicit QuietWindow(double duration = 0.4) : duration_(duration) {}
  void reset() { started_ = false; }
  bool update(double stamp, bool eligible) {
    if (!std::isfinite(stamp) || !eligible || (started_ && stamp < last_)) {
      reset(); return false;
    }
    if (!started_) { first_ = stamp; started_ = true; }
    last_ = stamp;
    return stamp - first_ >= duration_;
  }
 private:
  double duration_, first_{0.0}, last_{0.0};
  bool started_{false};
};
}  // namespace pm01_fastlio_nav
