#include <gtest/gtest.h>
#include <thread>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "pm01_navigation_nav2/navigation_ready_wait.hpp"

class RunningChild : public BT::StatefulActionNode
{
public:
  RunningChild() : BT::StatefulActionNode("planning_and_recovery", {}) {}
  int starts = 0, ticks = 0, stops = 0;
  BT::NodeStatus onStart() override {++starts; return BT::NodeStatus::RUNNING;}
  BT::NodeStatus onRunning() override {++ticks; return BT::NodeStatus::RUNNING;}
  void onHalted() override {++stops;}
};

class NavigationReadyWaitTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "/pm01/navigation_readiness:=/test_ready_wait/readiness",
      "-r", "/pm01/navigation_wait_status:=/test_ready_wait/wait"});
    node = std::make_shared<rclcpp::Node>("readiness_wait_test", options);
    publisher = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/pm01/navigation_readiness", rclcpp::QoS(1).transient_local());
    config.blackboard = BT::Blackboard::create();
    config.blackboard->set("node", node);
    config.input_ports = {{"timeout", "0.3"}, {"heartbeat_timeout", "0.1"}};
    wait = std::make_unique<pm01_navigation::NavigationReadyWait>("guard", config);
    wait->setChild(&child);
    for (int i = 0; i < 100 && publisher->get_subscription_count() == 0; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(publisher->get_subscription_count(), 0u);
  }
  BT::NodeStatus publish(bool ready, double stamp_offset = 0.)
  {
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = node->now() + rclcpp::Duration::from_seconds(stamp_offset);
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = ready ? 0 : 1;
    status.message = ready ? "tracking" : "scan_stale";
    diagnostic_msgs::msg::KeyValue value;
    value.key = "ready"; value.value = ready ? "true" : "false";
    status.values.push_back(value); msg.status.push_back(status);
    publisher->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    return wait->executeTick();
  }
  RunningChild child;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher;
  BT::NodeConfiguration config;
  std::unique_ptr<pm01_navigation::NavigationReadyWait> wait;
};

TEST_F(NavigationReadyWaitTest, HaltsWholeChildAndRestartsForSameGoal)
{
  EXPECT_EQ(publish(true), BT::NodeStatus::RUNNING);
  EXPECT_EQ(child.starts, 1);
  EXPECT_EQ(publish(false), BT::NodeStatus::RUNNING);
  EXPECT_EQ(child.stops, 1);
  EXPECT_EQ(publish(false), BT::NodeStatus::RUNNING);
  EXPECT_EQ(child.starts, 1);
  EXPECT_EQ(publish(true), BT::NodeStatus::RUNNING);
  EXPECT_EQ(child.starts, 2);
}

TEST_F(NavigationReadyWaitTest, TimeoutFailsOutsideRecoveryAndCannotResume)
{
  EXPECT_EQ(publish(false), BT::NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(320));
  EXPECT_EQ(publish(true), BT::NodeStatus::FAILURE);
  EXPECT_EQ(child.starts, 0);
  EXPECT_EQ(publish(true), BT::NodeStatus::FAILURE);
}

TEST_F(NavigationReadyWaitTest, HaltCancelsWaitingWithoutStartingChild)
{
  publish(true);
  publish(false);
  wait->halt();
  EXPECT_EQ(wait->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(child.status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(child.starts, 1);
  EXPECT_EQ(child.stops, 1);
}

TEST_F(NavigationReadyWaitTest, MissingHeartbeatStopsActiveChild)
{
  publish(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  EXPECT_EQ(wait->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(child.stops, 1);
}

TEST_F(NavigationReadyWaitTest, FreshStatusOneClockTickAheadCanRun)
{
  EXPECT_EQ(publish(true, .1), BT::NodeStatus::RUNNING);
  EXPECT_EQ(child.starts, 1);
}

TEST_F(NavigationReadyWaitTest, StaleTransientStatusCannotStartANewTree)
{
  publish(true, -.5);
  EXPECT_EQ(child.starts, 0);
  RunningChild late_child;
  pm01_navigation::NavigationReadyWait late("late_guard", config);
  late.setChild(&late_child);
  for (int i = 0; i < 100 && publisher->get_subscription_count() < 2; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(publisher->get_subscription_count(), 2u);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  EXPECT_EQ(late.executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(late_child.starts, 0);
}

TEST_F(NavigationReadyWaitTest, NavigatorHaltResetsTimeoutForANewGoal)
{
  std::string latest_state;
  rclcpp::executors::SingleThreadedExecutor observer;
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  observer.add_callback_group(group, node->get_node_base_interface());
  rclcpp::SubscriptionOptions options;
  options.callback_group = group;
  auto status = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/pm01/navigation_wait_status", rclcpp::QoS(1).transient_local(),
    [&latest_state](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
      latest_state = msg->status.front().message;
    }, options);
  publish(false);
  std::this_thread::sleep_for(std::chrono::milliseconds(320));
  EXPECT_EQ(publish(true), BT::NodeStatus::FAILURE);
  std::this_thread::sleep_for(std::chrono::milliseconds(30)); observer.spin_some();
  EXPECT_EQ(latest_state, "timed_out");
  wait->halt();  // Nav2 halts the tree when the previous action terminates.
  EXPECT_EQ(wait->status(), BT::NodeStatus::IDLE);
  EXPECT_EQ(publish(true), BT::NodeStatus::RUNNING);
  EXPECT_EQ(child.starts, 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(30)); observer.spin_some();
  EXPECT_EQ(latest_state, "running");
}

TEST_F(NavigationReadyWaitTest, LoadsExportedPluginAndTreeHaltAllowsAnotherGoal)
{
  BT::BehaviorTreeFactory factory;
  ASSERT_NO_THROW(factory.registerFromPlugin(PM01_READY_BT_LIBRARY));
  int calls = 0;
  factory.registerSimpleAction("Task", [&calls](BT::TreeNode &) {
      ++calls; return BT::NodeStatus::SUCCESS;
    });
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute='Main'><BehaviorTree ID='Main'>"
    "<NavigationReadyWait timeout='0.1'><Task/></NavigationReadyWait>"
    "</BehaviorTree></root>", config.blackboard);
  publish(false);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(calls, 0);
  tree.haltTree();
  publish(true);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(calls, 1);
}
