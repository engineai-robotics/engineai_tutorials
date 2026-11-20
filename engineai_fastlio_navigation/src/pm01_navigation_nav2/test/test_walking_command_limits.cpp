#include <limits>
#include <gtest/gtest.h>
#include "pm01_navigation_nav2/walking_command_limits.hpp"

using pm01_fastlio_nav::reachableWalkingCommand;

TEST(AlignmentWaypoint, ShortPathBehindRobotKeepsItsEndpoint) {
  using pm01_fastlio_nav::alignmentWaypoint;
  std::vector<geometry_msgs::msg::PoseStamped> path(3);
  path[1].pose.position.x = -.27;
  path[2].pose.position.x = -.55;
  EXPECT_EQ(alignmentWaypoint(path, .60), 2u);
  path.resize(5);
  path[3].pose.position.x = -.62;
  path[4].pose.position.x = -.84;
  EXPECT_EQ(alignmentWaypoint(path, .60), 3u);
  path.resize(1);
  EXPECT_EQ(alignmentWaypoint(path, .60), 0u);
  path.clear();
  EXPECT_THROW(alignmentWaypoint(path, .60), std::invalid_argument);
}

TEST(AlignmentSpeed, AccountsForMeasuredCoastInTheDirectionOfTheGoal) {
  using pm01_fastlio_nav::alignmentSpeed;
  EXPECT_DOUBLE_EQ(alignmentSpeed(.28, 0., .65, 1.5), .65);
  EXPECT_DOUBLE_EQ(alignmentSpeed(.28, .60, .65, 1.5), 0.);
  EXPECT_DOUBLE_EQ(alignmentSpeed(-.28, -.60, .65, 1.5), 0.);
  EXPECT_DOUBLE_EQ(alignmentSpeed(-.28, .60, .65, 1.5), -.65);
}

TEST(AlignmentSpeed, KeepsAnExecutableTargetUntilInsideTheAngularTolerance) {
  using pm01_fastlio_nav::alignmentSpeed;
  for (double sign : {-1.0, 1.0}) {
    EXPECT_DOUBLE_EQ(alignmentSpeed(sign*.28, sign*.60, .65, 1.5, .56, .20), sign*.56);
    EXPECT_DOUBLE_EQ(alignmentSpeed(sign*.20, sign*.60, .65, 1.5, .56, .20), 0.0);
    EXPECT_DOUBLE_EQ(alignmentSpeed(0.0, sign*.60, .65, 1.5, .56, .20), 0.0);
    EXPECT_DOUBLE_EQ(alignmentSpeed(sign*2.7, 0.0, .65, 1.5, .56, .35), sign*.65);
  }
}

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
