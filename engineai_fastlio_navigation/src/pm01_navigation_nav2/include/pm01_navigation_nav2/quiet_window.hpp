#pragma once
#include <cmath>

namespace pm01_fastlio_nav {
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
