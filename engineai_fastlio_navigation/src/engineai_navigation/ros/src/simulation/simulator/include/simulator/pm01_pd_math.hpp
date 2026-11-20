// Copyright 2026 EngineAI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <algorithm>
#include <cmath>

namespace simulator
{

inline double compute_pd_effort(
  double target_position, double position, double velocity,
  double kp, double kd, double effort_limit)
{
  if (
    !std::isfinite(target_position) || !std::isfinite(position) ||
    !std::isfinite(velocity) || !std::isfinite(kp) ||
    !std::isfinite(kd) || !std::isfinite(effort_limit) ||
    effort_limit < 0.0)
  {
    return 0.0;
  }
  const double effort =
    kp * (target_position - position) - kd * velocity;
  return std::clamp(effort, -effort_limit, effort_limit);
}

}  // namespace simulator
