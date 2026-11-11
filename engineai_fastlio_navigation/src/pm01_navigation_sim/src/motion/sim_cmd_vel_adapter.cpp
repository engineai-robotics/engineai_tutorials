#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <string>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_srvs/srv/trigger.hpp>

// The upstream walking policy consumes normalized joystick values and then
// multiplies by command_scale_pos/neg. Keep Nav2's public /cmd_vel in SI units.
class SimCmdVelAdapter : public rclcpp::Node {
 public:
  SimCmdVelAdapter() : Node("sim_cmd_vel_adapter") {
    scales_ = declare_parameter<std::vector<double>>("command_scales", {0.7, 0.4, 0.7});
    timeout_ = declare_parameter("cmd_timeout_sec", 0.2);
    require_lio_health_ = declare_parameter("require_lio_health", false);
    health_timeout_ = declare_parameter("lio_timeout_sec", 0.5);
    if (!std::isfinite(health_timeout_) || health_timeout_ <= 0)
      throw std::invalid_argument("LIO health timeout must be positive");
    if (scales_.size() != 3 || !std::isfinite(timeout_) || timeout_ <= 0)
      throw std::invalid_argument("Expected three scales and a positive timeout");
    for (double value : scales_) {
      if (!std::isfinite(value) || value <= 0)
        throw std::invalid_argument("Command scales must be finite and positive");
    }
    pub_ = create_publisher<geometry_msgs::msg::Twist>("/pm01/walking_cmd_vel", 1);
    const auto ready_service = declare_parameter<std::string>("ready_service", "");
    armed_ = ready_service.empty();
    if (!ready_service.empty()) ready_ = create_service<std_srvs::srv::Trigger>(ready_service,
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response) {
        response->success = published_ && pub_->get_subscription_count() > 0;
        response->message = response->success ? "transport_ready" : "waiting_for_walking_policy";
        if (response->success && !armed_) { valid_ = false; armed_ = true; }
      });
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/pm01/navigation_motion_status", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto & v = msg->twist.twist;
        if (!std::isfinite(v.linear.x) || !std::isfinite(v.linear.y) ||
            !std::isfinite(v.angular.z)) return;
        const auto stamp = std::chrono::steady_clock::now();
        odom_speed_ = std::hypot(v.linear.x, v.linear.y);
        odom_yaw_rate_ = v.angular.z;
        const bool quiet = odom_speed_ < 0.05 && std::abs(odom_yaw_rate_) < 0.10;
        if (quiet && !odom_quiet_) quiet_since_ = stamp;
        odom_quiet_ = quiet; last_odom_ = stamp; odom_received_ = true;
      });
    if (require_lio_health_) {
      health_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/lio/status", 10,
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
          motion_supported_ = false;
          const double age = (now()-rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type())).seconds();
          if (age < -0.1 || age > health_timeout_) return;
          for (const auto & status : msg->status) {
            if (status.name != "fast_lio/geometry") continue;
            for (const auto & value : status.values)
              if (value.key == "motion_supported") motion_supported_ = value.value == "true";
          }
          last_health_ = std::chrono::steady_clock::now();
        });
    }
    sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        if (!armed_) return;
        received_ = true;
        last_ = std::chrono::steady_clock::now();
        valid_ = std::isfinite(msg->linear.x) && std::isfinite(msg->linear.y) &&
          std::isfinite(msg->angular.z);
        if (!valid_) return;
        command_.linear.x = std::clamp(msg->linear.x / scales_[0], -1.0, 1.0);
        command_.linear.y = std::clamp(msg->linear.y / scales_[1], -1.0, 1.0);
        command_.angular.z = std::clamp(msg->angular.z / scales_[2], -1.0, 1.0);
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {
      const bool fresh = valid_ &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_).count() < timeout_;
      const bool healthy = !require_lio_health_ || (motion_supported_ &&
        std::chrono::duration<double>(std::chrono::steady_clock::now()-last_health_).count() < health_timeout_);
      pub_->publish(fresh && healthy ? command_ : geometry_msgs::msg::Twist());
      published_ = true;
      if (++status_tick_ % 10 != 0) return;
      const bool requested = valid_ && (std::abs(command_.linear.x) > 1e-6 ||
        std::abs(command_.linear.y) > 1e-6 || std::abs(command_.angular.z) > 1e-6);
      const bool health_fresh = std::chrono::duration<double>(
        std::chrono::steady_clock::now()-last_health_).count() < health_timeout_;
      diagnostic_msgs::msg::DiagnosticArray array;
      array.header.stamp = now();
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "pm01/simulation_motion_gate";
      status.hardware_id = "pm01_gazebo";
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = !received_ ? "waiting_for_command" : !valid_ ? "invalid_command" :
        !fresh ? "command_timeout" : !requested ? "zero_command" :
        !healthy ? (!health_fresh ? "lio_timeout" : "lio_unsupported") : "active";
      if (status.message == "invalid_command" || (fresh && requested && !healthy))
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      auto value = [&](const std::string & key, bool flag) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key; item.value = flag ? "true" : "false";
        status.values.push_back(item);
      };
      value("command_fresh", fresh);
      value("motion_requested", requested);
      value("lio_required", require_lio_health_);
      value("lio_fresh", health_fresh);
      value("lio_motion_supported", motion_supported_);
      value("output_motion_allowed", fresh && healthy && requested);
      const auto stamp = std::chrono::steady_clock::now();
      const bool odom_fresh = odom_received_ &&
        std::chrono::duration<double>(stamp-last_odom_).count() < 0.5;
      value("odometry_fresh", odom_fresh);
      auto detail = [&](const std::string & key, const std::string & content) {
        diagnostic_msgs::msg::KeyValue item; item.key = key; item.value = content;
        status.values.push_back(item);
      };
      detail("estimated_motion_state", !odom_fresh ? "unknown" : !odom_quiet_ ? "moving" :
        std::chrono::duration<double>(stamp-quiet_since_).count() < 0.4 ? "settling" : "stopped");
      detail("odometry_speed_mps", std::to_string(odom_speed_));
      detail("odometry_yaw_rate_radps", std::to_string(odom_yaw_rate_));
      array.status.push_back(status); status_pub_->publish(array);
    });
  }
 private:
  std::vector<double> scales_;
  double timeout_;
  bool require_lio_health_{false}, motion_supported_{false};
  double health_timeout_{0.5};
  std::chrono::steady_clock::time_point last_health_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_sub_;
  bool valid_{false}, received_{false};
  bool published_{false}, armed_{false};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ready_;
  unsigned status_tick_{0};
  geometry_msgs::msg::Twist command_;
  std::chrono::steady_clock::time_point last_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  bool odom_received_{false}, odom_quiet_{false};
  double odom_speed_{0.0}, odom_yaw_rate_{0.0};
  std::chrono::steady_clock::time_point last_odom_, quiet_since_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimCmdVelAdapter>());
  rclcpp::shutdown();
}
