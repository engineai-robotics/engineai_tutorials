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

#include "simulator/pm01_pd_math.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

TEST(Pm01PdMath, MatchesSdkControlLaw)
{
  EXPECT_DOUBLE_EQ(
    simulator::compute_pd_effort(1.0, 0.8, 0.5, 100.0, 4.0, 50.0),
    18.0);
}

TEST(Pm01PdMath, ClipsAtMotorEffortLimit)
{
  EXPECT_DOUBLE_EQ(
    simulator::compute_pd_effort(2.0, 0.0, 0.0, 100.0, 4.0, 61.0),
    61.0);
  EXPECT_DOUBLE_EQ(
    simulator::compute_pd_effort(-2.0, 0.0, 0.0, 100.0, 4.0, 61.0),
    -61.0);
}

TEST(Pm01PdMath, RejectsNonFiniteState)
{
  EXPECT_DOUBLE_EQ(
    simulator::compute_pd_effort(
      0.0, std::numeric_limits<double>::quiet_NaN(),
      0.0, 100.0, 4.0, 61.0),
    0.0);
}

TEST(Pm01PdMath, MapsAll24JointTargetsIndependently)
{
  constexpr std::size_t joint_count = 24;
  std::vector<double> efforts(joint_count, 0.0);
  for (std::size_t index = 0; index < joint_count; ++index) {
    const double target = 0.01 * static_cast<double>(index + 1);
    efforts[index] = simulator::compute_pd_effort(
      target, 0.0, 0.0, 10.0, 1.0, 100.0);
  }
  for (std::size_t index = 0; index < joint_count; ++index) {
    EXPECT_NEAR(
      efforts[index], 0.1 * static_cast<double>(index + 1), 1e-12);
  }
}

TEST(Pm01PdMath, DefaultTargetHoldsCurrentPose)
{
  EXPECT_DOUBLE_EQ(
    simulator::compute_pd_effort(-0.25, -0.25, 0.0, 30.0, 0.3, 61.0),
    0.0);
}
