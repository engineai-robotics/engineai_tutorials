#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <interface_protocol/msg/body_vel_cmd.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "pm01_navigation_hardware/body_velocity_command.hpp"

class CmdVelBridge : public rclcpp::Node {
 public:
  CmdVelBridge() : Node("cmd_vel_bridge") {
    limit_velocity_ = declare_parameter("limit_velocity", true);
    release_on_goal_success_ = declare_parameter("release_on_goal_success", false);
    max_x_ = declare_parameter("max_vel_x", 0.7);
    max_y_ = declare_parameter("max_vel_y", 0.4);
    max_yaw_ = declare_parameter("max_yaw", 0.7);
    timeout_ = declare_parameter("cmd_timeout_sec", 0.2);
    const auto hz = declare_parameter("body_vel_hz", 50.0);
    frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const auto ready_service = declare_parameter<std::string>("ready_service", "");
    armed_ = ready_service.empty();  // Standalone developer invocation remains usable.
    consumer_ = declare_parameter<std::string>("expected_consumer", "locomotion_interface_node");
    for (auto v : {max_x_, max_y_, max_yaw_, timeout_, hz}) {
      if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("Limits, timeout and rate must be finite and positive");
    }
    pub_ = create_publisher<interface_protocol::msg::BodyVelCmd>("/motion/body_vel_cmd", 1);
    if (!ready_service.empty()) ready_ = create_service<std_srvs::srv::Trigger>(ready_service,
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response) {
        bool consumer_found = false;
        for (const auto & endpoint : get_subscriptions_info_by_topic(pub_->get_topic_name())) {
          if (endpoint.node_name() == consumer_ &&
              endpoint.topic_type() == "interface_protocol/msg/BodyVelCmd") consumer_found = true;
        }
        response->success = published_ && consumer_found && pub_->get_subscription_count() > 0;
        response->message = response->success ? "transport_ready; physical_controller_owner=unknown" :
          "waiting_for_motion_consumer:" + consumer_;
        // Discovery alone cannot prove physical control ownership. Do not
        // forward a cached command received while the transport was starting.
        if (response->success && !armed_) { valid_ = false; armed_ = true; }
      });
    const auto armed_at = now().nanoseconds();
    if (release_on_goal_success_) {
      status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
        "/navigate_to_pose/_action/status", rclcpp::QoS(1).transient_local().reliable(),
        [this, armed_at](action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) {
          for (const auto & status : msg->status_list) {
            const auto goal_stamp = rclcpp::Time(status.goal_info.stamp).nanoseconds();
            // Retained success from a previous arm must not release a new one.
            if (status.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED &&
                goal_stamp >= armed_at && !releasing_) {
              releasing_ = true;
              release_started_ = std::chrono::steady_clock::now();
              RCLCPP_INFO(get_logger(),
                "Navigation succeeded: sending zero for 1 second, then releasing velocity input");
            }
          }
        });
    }
    sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        if (releasing_ || !armed_) return;
        std::array<double, 3> velocity;
        valid_ = pm01_fastlio_nav::convert_body_velocity(
          {msg->linear.x, msg->linear.y, msg->angular.z},
          {max_x_, max_y_, max_yaw_}, limit_velocity_, velocity);
        if (valid_) {
          vx_ = velocity[0];
          vy_ = velocity[1];
          wz_ = velocity[2];
          last_ = std::chrono::steady_clock::now();
        }
      });
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / hz)), [this]() {
        const auto steady_now = std::chrono::steady_clock::now();
        const bool fresh = !releasing_ && valid_ &&
          std::chrono::duration<double>(steady_now - last_).count() < timeout_;
        interface_protocol::msg::BodyVelCmd out;
        out.header.stamp = now(); out.header.frame_id = frame_;
        out.linear_velocity = {fresh ? vx_ : 0.0, fresh ? vy_ : 0.0};
        out.yaw_velocity = fresh ? wz_ : 0.0;
        pub_->publish(out);
        published_ = true;
        if (releasing_ && std::chrono::duration<double>(steady_now - release_started_).count() >= 1.0) {
          RCLCPP_INFO(get_logger(), "Zero command drain complete; unregistering navigation velocity publisher");
          timer_->cancel();
          rclcpp::shutdown();
        }
      });
  }
 private:
  double max_x_, max_y_, max_yaw_, timeout_, vx_{0}, vy_{0}, wz_{0};
  bool valid_{false}, limit_velocity_{true}, release_on_goal_success_{false}, releasing_{false};
  bool published_{false}, armed_{false};
  std::string consumer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ready_;
  std::string frame_;
  std::chrono::steady_clock::time_point last_, release_started_;
  rclcpp::Publisher<interface_protocol::msg::BodyVelCmd>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<CmdVelBridge>()); rclcpp::shutdown();
}
