#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace pm01_fastlio_nav {
// Nav2 already publishes m/s and rad/s. Optional clamps change neither units
// nor axes. Invalid input is rejected in both modes; the bridge then emits zero.
inline bool convert_body_velocity(
    const std::array<double, 3> & input, const std::array<double, 3> & limits,
    bool limit_velocity, std::array<double, 3> & output) {
  for (const auto value : input) {
    if (!std::isfinite(value)) return false;
  }
  if (limit_velocity) {
    for (const auto value : limits) {
      if (!std::isfinite(value) || value <= 0.0) return false;
    }
  }
  for (std::size_t axis = 0; axis < input.size(); ++axis) {
    output[axis] = limit_velocity ? std::clamp(input[axis], -limits[axis], limits[axis]) : input[axis];
  }
  return true;
}
}  // namespace pm01_fastlio_nav
