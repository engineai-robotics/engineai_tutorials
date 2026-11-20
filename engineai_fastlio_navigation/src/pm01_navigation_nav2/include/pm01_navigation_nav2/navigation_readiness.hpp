#pragma once

#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <string>

namespace pm01_navigation
{
// Status messages describe a recent decision, not a new sensor measurement.
// Independent /clock callbacks can place that header one tick ahead of the
// receiver. Preserve strict source-time checks for the actual observations.
inline bool statusStampFresh(int64_t now_ns, int64_t stamp_ns, double timeout)
{
  return now_ns >= 0 && stamp_ns >= 0 && now_ns - stamp_ns >= -200000000LL &&
         now_ns - stamp_ns <= std::llround(timeout * 1e9);
}

struct ObservationStatus
{
  double received = -1.;
  int64_t stamp_ns = -1;
  int level = 2;
  std::string reason;
  std::map<std::string, std::string> values;

  bool flag(const std::string & key) const
  {
    const auto it = values.find(key);
    return it != values.end() && it->second == "true";
  }
  double number(const std::string & key) const
  {
    const auto it = values.find(key);
    if (it == values.end()) {return NAN;}
    try {
      size_t consumed = 0;
      const auto value = std::stod(it->second, &consumed);
      return consumed == it->second.size() ? value : NAN;
    } catch (...) {return NAN;}
  }
};

struct ReadinessResult
{
  bool ready = false;
  std::string state = "blocked";
  std::string reason = "waiting_for_observations";
  double healthy_duration = 0.;
};

// The ROS command outlet and the BT share this one computed decision. Source
// ages are ROS time; transport liveness and settling use monotonic time.
class NavigationReadiness
{
public:
  double heartbeat_timeout = 2.5;
  double observation_timeout = 1.0;
  double map_observation_timeout = 2.5;
  double resume_delay = 1.;
  std::map<std::string, ObservationStatus> observations;

  static bool fresh(double now, double stamp, double timeout)
  {
    return std::isfinite(stamp) && stamp >= 0. &&
           now - stamp >= -.05 && now - stamp <= timeout;
  }

  ReadinessResult update(double wall, int64_t ros_ns)
  {
    const auto reason = unavailable(wall, ros_ns);
    if (!reason.empty()) {
      healthy_since_.reset();
      return {false, "blocked", reason, 0.};
    }
    if (!healthy_since_ || wall < *healthy_since_) {healthy_since_ = wall;}
    const auto duration = wall - *healthy_since_;
    if (duration < resume_delay) {
      return {false, "settling", "waiting_for_stable_observations", duration};
    }
    return {true, "ready", "tracking", duration};
  }

private:
  std::optional<double> healthy_since_;

  std::string unavailable(double wall, int64_t ros_ns) const
  {
    const double ros = ros_ns * 1e-9;
    for (const std::string key : {"ground", "scan", "localization", "map"}) {
      const auto it = observations.find(key);
      if (it == observations.end() || !fresh(wall, it->second.received, heartbeat_timeout)) {
        return key + "_status_stale";
      }
      const auto & s = it->second;
      if (key == "ground") {
        if (!s.flag("ground_valid") || !s.flag("local_observation_usable")) {
          return "ground_unavailable:" + s.reason;
        }
        if (!fresh(ros, s.number("source_stamp"), observation_timeout)) {
          return "ground_observation_stale";
        }
      } else if (key == "scan") {
        if (s.level != 0) {return "scan_unavailable:" + s.reason;}
        if (!fresh(ros, s.number("cloud_stamp"), observation_timeout)) {
          return "scan_observation_stale";
        }
      } else if (key == "localization") {
        const auto mode=s.values.find("mode");
        const bool held=s.level==1 && s.flag("held_alignment_usable") &&
          mode!=s.values.end() && mode->second=="\"slam\"";
        if (!(s.level==0 && s.flag("matched")) && !held) {
          return "localization_unavailable:" + s.reason;
        }
        if (!statusStampFresh(ros_ns, s.stamp_ns, heartbeat_timeout)) {return "localization_status_stale";}
      } else {
        if (!s.flag("local_observation_usable") || !s.flag("map_alignment_usable") ||
          !s.flag("observation_usable"))
        {
          return "map_unavailable:" + s.reason;
        }
        if (!fresh(ros, s.number("source_stamp"), map_observation_timeout)) {
          return "map_observation_stale";
        }
      }
    }
    return {};
  }
};
}  // namespace pm01_navigation
