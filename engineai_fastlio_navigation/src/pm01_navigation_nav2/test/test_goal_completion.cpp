#include <cmath>
#include <memory>
#include <gtest/gtest.h>
#include <rcl/time.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <pluginlib/class_loader.hpp>
#include <nav2_core/goal_checker.hpp>
#include <nav2_core/controller.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace {
constexpr double pi = 3.14159265358979323846;
geometry_msgs::msg::Pose pose(double x, double y, double yaw_deg) {
  geometry_msgs::msg::Pose p;
  p.position.x = x; p.position.y = y;
  tf2::Quaternion q; q.setRPY(0, 0, yaw_deg*pi/180.0);
  p.orientation = tf2::toMsg(q);
  return p;
}

class GoalCompletion : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    loader = std::make_unique<pluginlib::ClassLoader<nav2_core::GoalChecker>>(
      "nav2_core", "nav2_core::GoalChecker");
  }
  void configure(bool legacy = false) {
    auto parameters = std::vector<rclcpp::Parameter>{
      {"goal.stateful", false}, {"goal.xy_goal_tolerance", .25},
      {"goal.yaw_goal_tolerance", 25*pi/180.0},
      {"goal.trans_stopped_velocity", .05}, {"goal.rot_stopped_velocity", .1}};
    if (!legacy) parameters.emplace_back("goal.require_stopped", false);
    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("goal_completion_test", options);
    checker = loader->createSharedInstance("pm01_fastlio_nav::SettledGoalChecker");
    checker->initialize(node, "goal", nullptr);
    ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), RCL_RET_OK);
  }
  bool reached(double time, const geometry_msgs::msg::Pose & p,
      const geometry_msgs::msg::Twist & velocity = geometry_msgs::msg::Twist()) {
    EXPECT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),
      static_cast<int64_t>(time*1e9)), RCL_RET_OK);
    return checker->isGoalReached(p, goal, velocity);
  }
  void TearDown() override {
    checker.reset(); node.reset(); loader.reset(); rclcpp::shutdown();
  }
  geometry_msgs::msg::Pose goal{pose(0, 0, 0)};
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::GoalChecker>> loader;
  std::shared_ptr<nav2_core::GoalChecker> checker;
};

TEST_F(GoalCompletion, PositionAndHeadingMustHoldTogetherForFourTenths) {
  configure();
  EXPECT_FALSE(reached(100, pose(.24, 0, 24)));
  EXPECT_FALSE(reached(100.2, pose(.24, 0, 24)));
  EXPECT_FALSE(reached(100.35, pose(.251, 0, 24)));
  EXPECT_FALSE(reached(101, pose(.24, 0, 24)));
  EXPECT_FALSE(reached(101.3, pose(.24, 0, 25.1)));
  EXPECT_FALSE(reached(102, pose(.24, 0, 24)));
  EXPECT_FALSE(reached(102.399, pose(.24, 0, 24)));
  EXPECT_TRUE(reached(102.401, pose(.24, 0, 24)));
}

TEST_F(GoalCompletion, RadiusIsEuclideanRatherThanPerAxis) {
  configure();
  EXPECT_FALSE(reached(100, pose(.2, .2, 0)));
  EXPECT_FALSE(reached(101, pose(.2, .2, 0)));
}

TEST_F(GoalCompletion, WrappedHeadingAndFiniteBodySwayDoNotBlockArrival) {
  configure(); goal = pose(0, 0, -179);
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = .6; velocity.angular.z = 1.;
  EXPECT_FALSE(reached(100, pose(.24, 0, 179), velocity));
  EXPECT_TRUE(reached(100.41, pose(.24, 0, 179), velocity));
}

TEST_F(GoalCompletion, ExistingDefaultStillRequiresStoppedVelocity) {
  configure(true);
  geometry_msgs::msg::Twist velocity; velocity.linear.x = .06;
  EXPECT_FALSE(reached(100, pose(0, 0, 0), velocity));
  EXPECT_FALSE(reached(101, pose(0, 0, 0), velocity));
  EXPECT_FALSE(reached(102, pose(0, 0, 0)));
  EXPECT_TRUE(reached(102.41, pose(0, 0, 0)));
}

TEST_F(GoalCompletion, ResetRequiresACompleteNewWindow) {
  configure();
  EXPECT_FALSE(reached(100, pose(.24, 0, 0)));
  EXPECT_TRUE(reached(100.41, pose(.24, 0, 0)));
  checker->reset();
  EXPECT_FALSE(reached(100.42, pose(.24, 0, 0)));
  EXPECT_TRUE(reached(100.83, pose(.24, 0, 0)));
}
}  // namespace

TEST_F(GoalCompletion, GoalCheckerDoesNotStealStandardDwbFactory) {
  configure();
  pluginlib::ClassLoader<nav2_core::Controller> controllers("nav2_core", "nav2_core::Controller");
  auto controller = controllers.createSharedInstance("dwb_core::DWBLocalPlanner");
  ASSERT_NE(controller, nullptr);
}
