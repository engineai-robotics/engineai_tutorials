#include <gtest/gtest.h>
#include <thread>
#include <vector>
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "pm01_navigation_nav2/navigation_command_gate.hpp"

class CommandGateTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "/pm01/navigation_readiness:=/test_command_gate/readiness",
      "-r", "/pm01/ground_plane_status:=/test_command_gate/ground",
      "-r", "/pm01/scan_status:=/test_command_gate/scan",
      "-r", "/pm01/localization_status:=/test_command_gate/localization",
      "-r", "/pm01/map_update_status:=/test_command_gate/map",
      "-r", "navigation_cmd_vel_requested:=/test_command_gate/requested",
      "-r", "cmd_vel:=/test_command_gate/output"});
    options.parameter_overrides({rclcpp::Parameter("resume_delay", .1)});
    gate = pm01_navigation::make_navigation_command_gate(options);
    node = std::make_shared<rclcpp::Node>("command_gate_test", options);
    executor.add_node(gate); executor.add_node(node);
    for (const auto & topic : {"/pm01/ground_plane_status", "/pm01/scan_status",
        "/pm01/localization_status", "/pm01/map_update_status"})
    {
      statuses.push_back(node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(topic, 10));
    }
    // Both smoother and behavior outputs use the same guarded inlet.
    controller = node->create_publisher<geometry_msgs::msg::Twist>("navigation_cmd_vel_requested", 1);
    backup = node->create_publisher<geometry_msgs::msg::Twist>("navigation_cmd_vel_requested", 1);
    output = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {received.push_back(*msg);});
    spin(.25);
  }
  void TearDown() override
  {
    executor.remove_node(gate); executor.remove_node(node);
  }
  void spin(double seconds)
  {
    const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < until) {
      executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  void observations(bool matched = true, double localization_stamp_offset = 0.)
  {
    for (size_t i = 0; i < statuses.size(); ++i) {
      diagnostic_msgs::msg::DiagnosticArray msg;
      msg.header.stamp = node->now() + rclcpp::Duration::from_seconds(
        i == 2 ? localization_stamp_offset : 0.);
      diagnostic_msgs::msg::DiagnosticStatus s;
      s.level = (i == 2 && !matched) ? 1 : 0;
      s.message = i == 1 ? "projecting" : (matched ? "tracking" : "recovering");
      for (const auto & key : {"ground_valid", "local_observation_usable",
          "map_alignment_usable", "observation_usable", "matched", "source_stamp", "cloud_stamp"})
      {
        diagnostic_msgs::msg::KeyValue value;
        value.key = key;
        value.value = std::string(key).find("stamp") != std::string::npos ?
          std::to_string(node->now().seconds()) : (std::string(key) == "matched" && !matched ? "false" : "true");
        s.values.push_back(value);
      }
      msg.status.push_back(s); statuses[i]->publish(msg);
    }
    spin(.025);
  }
  bool moving() const
  {
    for (const auto & msg : received) {
      if (msg.linear.x != 0. || msg.linear.y != 0. || msg.angular.z != 0.) {return true;}
    }
    return false;
  }
  rclcpp::Node::SharedPtr gate, node;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::vector<rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr> statuses;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr controller, backup;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr output;
  std::vector<geometry_msgs::msg::Twist> received;
};

TEST_F(CommandGateTest, BlocksControllerAndRecoveryAndDoesNotReplayOldCommand)
{
  geometry_msgs::msg::Twist forward, reverse;
  forward.linear.x = .4; reverse.linear.x = -.1;
  controller->publish(forward); backup->publish(reverse); spin(.03);
  EXPECT_FALSE(moving());
  observations(); spin(.12); observations(); received.clear();
  controller->publish(forward); spin(.03);
  ASSERT_TRUE(moving());
  observations(false); received.clear();
  controller->publish(forward); backup->publish(reverse); spin(.03);
  EXPECT_FALSE(moving());
  observations(); spin(.12); observations(); received.clear(); spin(.03);
  EXPECT_FALSE(moving());  // Recovery alone must not replay a pre-pause command.
  backup->publish(reverse); spin(.03);
  EXPECT_TRUE(moving());
}

TEST_F(CommandGateTest, OneClockTickAheadStatusDoesNotBlockFreshObservations)
{
  observations(true, .1); spin(.12); observations(true, .1);
  received.clear();
  geometry_msgs::msg::Twist command;
  command.linear.x = .4;
  controller->publish(command); spin(.03);
  EXPECT_TRUE(moving());
}
