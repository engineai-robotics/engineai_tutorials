#include <limits>
#include <gtest/gtest.h>
#include "pm01_navigation_nav2/walking_command_limits.hpp"

using pm01_fastlio_nav::reachableWalkingCommand;

TEST(WalkingCommand, RecordedNearGoalStall) {
  EXPECT_FALSE(reachableWalkingCommand(0.07105263, 0., 0.62631579, .25, .62));
  EXPECT_FALSE(reachableWalkingCommand(0.3, 0., 0.23947368, .25, .62));
  EXPECT_FALSE(reachableWalkingCommand(0., 0.15, 0.23947368, .25, .62));
}
TEST(WalkingCommand, ExecutableStraightTurnArcAndStop) {
  EXPECT_TRUE(reachableWalkingCommand(.3, 0., 0., .25, .62));
  EXPECT_TRUE(reachableWalkingCommand(0., 0., .62631579, .25, .62));
  EXPECT_TRUE(reachableWalkingCommand(.3, 0., -.62631579, .25, .62));
  EXPECT_TRUE(reachableWalkingCommand(0., 0., 0., .25, .62));
}
TEST(WalkingCommand, BoundariesAndNonfinite) {
  EXPECT_TRUE(reachableWalkingCommand(.25, 0., -.62, .25, .62));
  EXPECT_FALSE(reachableWalkingCommand(.249, 0., -.62, .25, .62));
  EXPECT_FALSE(reachableWalkingCommand(.25, 0., .619, .25, .62));
  EXPECT_TRUE(reachableWalkingCommand(.0001, 0., .0001, .25, .62));
  EXPECT_FALSE(reachableWalkingCommand(std::numeric_limits<double>::quiet_NaN(), 0., .7, .25, .62));
  EXPECT_FALSE(reachableWalkingCommand(.3, 0., std::numeric_limits<double>::infinity(), .25, .62));
}
