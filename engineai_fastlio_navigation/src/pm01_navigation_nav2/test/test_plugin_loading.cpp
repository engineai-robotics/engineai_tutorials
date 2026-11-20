#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_core/goal_checker.hpp>
#include <dwb_core/trajectory_critic.hpp>

TEST(PluginLoading, ExistingPublicIdentifiersResolveFromNewPackage) {
  pluginlib::ClassLoader<nav2_core::Controller> controllers("nav2_core", "nav2_core::Controller");
  EXPECT_NE(controllers.createSharedInstance("pm01_fastlio_nav::LatestTfRotationShimController"), nullptr);
  EXPECT_NE(controllers.createSharedInstance("pm01_fastlio_nav::SettlingDWBController"), nullptr);
  pluginlib::ClassLoader<nav2_core::GoalChecker> goals("nav2_core", "nav2_core::GoalChecker");
  EXPECT_NE(goals.createSharedInstance("pm01_fastlio_nav::SettledGoalChecker"), nullptr);
  pluginlib::ClassLoader<dwb_core::TrajectoryCritic> critics("dwb_core", "dwb_core::TrajectoryCritic");
  EXPECT_NE(critics.createSharedInstance("pm01_fastlio_nav::WalkingCommandCritic"), nullptr);
}
