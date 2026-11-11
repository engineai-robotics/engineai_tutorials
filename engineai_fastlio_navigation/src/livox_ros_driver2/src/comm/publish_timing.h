#ifndef PM01_LIVOX_PUBLISH_TIMING_H_
#define PM01_LIVOX_PUBLISH_TIMING_H_

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <utility>

namespace livox_ros {

// One instance per publishing poll thread/channel. Disabled unless explicitly
// enabled for this process; never changes timestamps, QoS, or the data queue.
class PublishTiming {
 public:
  explicit PublishTiming(const char* channel, double budget_sec = 120.)
      : channel_(channel), enabled_(Enabled()), window_(Clock::now()),
        started_(window_), budget_sec_(budget_sec) {}

  template <class Send>
  void Publish(uint64_t source_ns, Send&& send) {
    if (!enabled_) {
      std::forward<Send>(send)();
      return;
    }
    const auto begin = Clock::now();
    if (std::chrono::duration<double>(begin - started_).count() >= budget_sec_) {
      enabled_ = false;
      std::forward<Send>(send)();
      return;
    }
    const double age = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count() -
        static_cast<double>(source_ns) * 1e-9;
    // Preserve exactly-once invocation and the original exception behavior.
    std::forward<Send>(send)();
    const auto end = Clock::now();
    const double elapsed = std::chrono::duration<double>(end - begin).count();
    const double gap = has_previous_ ?
        std::chrono::duration<double>(begin - previous_).count() : 0.;
    previous_ = begin;
    has_previous_ = true;
    ++count_;
    age_sum_ += age;
    publish_sum_ += elapsed;
    age_max_ = count_ == 1 ? age : std::max(age_max_, age);
    publish_max_ = std::max(publish_max_, elapsed);
    gap_max_ = std::max(gap_max_, gap);
    const double interval = std::chrono::duration<double>(end - window_).count();
    if (interval < 1.) return;
    const double wall_end = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    std::fprintf(stderr,
        "PM01_DRIVER_TIMING {\"channel\":\"%s\",\"count\":%llu,"
        "\"wall_end_sec\":%.6f,\"interval_sec\":%.6f,\"source_age_mean_sec\":%.6f,"
        "\"source_age_max_sec\":%.6f,\"publish_mean_sec\":%.6f,"
        "\"publish_max_sec\":%.6f,\"inter_call_gap_max_sec\":%.6f}\n",
        channel_, static_cast<unsigned long long>(count_), wall_end, interval,
        age_sum_ / count_, age_max_, publish_sum_ / count_, publish_max_, gap_max_);
    window_ = end;
    count_ = 0;
    age_sum_ = publish_sum_ = age_max_ = publish_max_ = gap_max_ = 0.;
  }

 private:
  static bool Enabled() {
    const char* flag = std::getenv("PM01_LIVOX_PUBLISH_TIMING");
    return flag && std::strcmp(flag, "1") == 0;
  }
  using Clock = std::chrono::steady_clock;
  const char* channel_;
  bool enabled_;
  Clock::time_point window_, started_, previous_{};
  double budget_sec_;
  bool has_previous_ = false;
  uint64_t count_ = 0;
  double age_sum_ = 0., publish_sum_ = 0., age_max_ = 0.,
         publish_max_ = 0., gap_max_ = 0.;
};

}  // namespace livox_ros
#endif
