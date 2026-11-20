#include <chrono>
#include <memory>
#include <sstream>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pm01_navigation_nav2/navigation_readiness.hpp"
#include "pm01_navigation_nav2/navigation_command_gate.hpp"

namespace pm01_navigation
{
class NavigationCommandGate : public rclcpp::Node
{
public:
  explicit NavigationCommandGate(const rclcpp::NodeOptions & options)
  : Node("navigation_command_gate", options)
  {
    readiness_.heartbeat_timeout = declare_parameter("heartbeat_timeout", 2.5);
    readiness_.observation_timeout = declare_parameter("observation_timeout", 1.0);
    readiness_.map_observation_timeout = declare_parameter("map_observation_timeout", 2.5);
    readiness_.resume_delay = declare_parameter("resume_delay", 1.);
    for (auto value : {readiness_.heartbeat_timeout, readiness_.observation_timeout,
        readiness_.map_observation_timeout, readiness_.resume_delay})
    {
      if (!std::isfinite(value) || value <= 0.) {throw std::invalid_argument("Invalid readiness timeout");}
    }
    if (declare_parameter("publish_commands", true)) {command_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);}
    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/pm01/navigation_readiness", rclcpp::QoS(1).transient_local());
    for (const auto & entry : std::vector<std::pair<std::string, std::string>>{
        {"ground", "/pm01/ground_plane_status"}, {"scan", "/pm01/scan_status"},
        {"localization", "/pm01/localization_status"}, {"map", "/pm01/map_update_status"}})
    {
      subscriptions_.push_back(create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        entry.second, rclcpp::QoS(10).best_effort(),
        [this, key = entry.first](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
          ObservationStatus status;
          status.received = wall_now();
          status.stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
          // These topics each own one status; an empty/malformed update closes the gate.
          if (msg->status.size() == 1) {
            const auto & source = msg->status.front();
            status.level = source.level;
            status.reason = source.message;
            for (const auto & value : source.values) {status.values[value.key] = value.value;}
          }
          readiness_.observations[key] = status;
          update();
        }));
    }
    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "navigation_cmd_vel_requested", rclcpp::QoS(1),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        update();
        if (result_.ready && command_pub_) {command_pub_->publish(*msg);}
        // No command cache: requests received while paused are discarded.
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {update();});
  }

private:
  static double wall_now()
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void update()
  {
    const auto was_ready = result_.ready;
    result_ = readiness_.update(wall_now(), now().nanoseconds());
    if (!result_.ready && command_pub_) {command_pub_->publish(geometry_msgs::msg::Twist());}
    if (was_ready && !result_.ready) {
      std::ostringstream ages;
      ages.setf(std::ios::fixed); ages.precision(3);
      const auto ros = now().seconds(), wall = wall_now();
      for (const auto & entry : readiness_.observations) {
        const auto source = entry.first == "localization" ? entry.second.stamp_ns * 1e-9 :
          entry.second.number(entry.first == "scan" ? "cloud_stamp" : "source_stamp");
        ages << " " << entry.first << "(source_age=" << ros - source <<
          ", heartbeat_age=" << wall - entry.second.received << ")";
      }
      RCLCPP_WARN(get_logger(), "Navigation paused: %s;%s", result_.reason.c_str(), ages.str().c_str());
    } else if (!was_ready && result_.ready) {
      RCLCPP_INFO(get_logger(), "Navigation observations ready");
    }
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "pm01/navigation_readiness";
    status.level = result_.ready ? 0 : 1;
    status.message = result_.reason;
    auto add = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue field;
        field.key = key; field.value = value; status.values.push_back(field);
      };
    add("ready", result_.ready ? "true" : "false");
    add("state", result_.state);
    add("reason", result_.reason);
    add("healthy_duration_sec", std::to_string(result_.healthy_duration));
    for (const auto & entry : readiness_.observations) {
      const auto source = entry.first == "localization" ? entry.second.stamp_ns * 1e-9 :
        entry.second.number(entry.first == "scan" ? "cloud_stamp" : "source_stamp");
      add("source_" + entry.first + "_stamp", std::to_string(source));
      add("source_" + entry.first + "_age_sec", std::to_string(now().seconds() - source));
    }
    // Map and command gate run on the same host. Measure recovery with its
    // monotonic clock, independently of simulated time or clock adjustments.
    const auto map = readiness_.observations.find("map");
    if (map != readiness_.observations.end()) {
      const auto & fields = map->second.values;
      const auto epoch = fields.find("loop_epoch"), revision = fields.find("loop_revision");
      const double published = map->second.number("loop_map_published_monotonic_sec");
      if (epoch != fields.end() && revision != fields.end() && std::isfinite(published)) {
        if (epoch->second != loop_epoch_ || revision->second != loop_revision_) {
          loop_epoch_ = epoch->second;
          loop_revision_ = revision->second;
          loop_resume_sec_.reset();
        }
        if (result_.ready && !loop_resume_sec_ && wall_now() >= published) {
          loop_resume_sec_ = wall_now() - published;
        }
        add("loop_epoch", loop_epoch_);
        add("loop_revision", loop_revision_);
        if (loop_resume_sec_) {add("loop_resume_sec", std::to_string(*loop_resume_sec_));}
      }
    }
    msg.status.push_back(std::move(status));
    status_pub_->publish(msg);
  }

  std::string loop_epoch_, loop_revision_;
  std::optional<double> loop_resume_sec_;
  NavigationReadiness readiness_;
  ReadinessResult result_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  std::vector<rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
};
std::shared_ptr<rclcpp::Node> make_navigation_command_gate(const rclcpp::NodeOptions & options)
{
  return std::make_shared<NavigationCommandGate>(options);
}
}  // namespace pm01_navigation
