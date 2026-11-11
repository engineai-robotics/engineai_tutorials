#pragma once

#include <cmath>

namespace pm01_fastlio_nav {
// Zero is a valid stop/straight component. Each nonzero component must be
// executable on its own; a large yaw cannot excuse an unusable small XY speed.
inline bool reachableWalkingCommand(double vx, double vy, double wz,
                                    double min_linear, double min_angular,
                                    double epsilon = 0.001) {
  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) return false;
  const double speed = std::hypot(vx, vy);
  return (speed <= epsilon || speed >= min_linear) &&
         (std::abs(wz) <= epsilon || std::abs(wz) >= min_angular);
}
}  // namespace pm01_fastlio_nav
