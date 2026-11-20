#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/time.hpp>

namespace fast_lio_time {
constexpr int64_t nanoseconds_per_second = 1000000000;
constexpr int64_t max_stamp_ns =
    static_cast<int64_t>(std::numeric_limits<int32_t>::max()) * nanoseconds_per_second +
    nanoseconds_per_second - 1;

// Round computed durations, not absolute input headers. An absolute double at
// Unix-epoch scale cannot represent every nanosecond of the source message.
inline int64_t rounded_nanoseconds(double seconds) {
    if (!std::isfinite(seconds))
        throw std::invalid_argument("Time must be finite");
    const long double rounded = std::round(
        static_cast<long double>(seconds) * nanoseconds_per_second);
    const long double limit = std::ldexp(1.0L, 63);
    if (rounded < -limit || rounded >= limit)
        throw std::out_of_range("Nanosecond duration exceeds int64 range");
    return static_cast<int64_t>(rounded);
}

inline rclcpp::Time from_nanoseconds(int64_t stamp) {
    if (stamp < 0 || stamp > max_stamp_ns)
        throw std::out_of_range("Timestamp is outside the ROS message time range");
    return rclcpp::Time(stamp);
}

inline rclcpp::Time from_seconds(double seconds) {
    return from_nanoseconds(rounded_nanoseconds(seconds));
}

inline rclcpp::Time add_seconds(
    const builtin_interfaces::msg::Time & source, double offset_seconds) {
    const int64_t stamp = rclcpp::Time(source).nanoseconds();
    const int64_t delta = rounded_nanoseconds(offset_seconds);
    if ((delta > 0 && stamp > std::numeric_limits<int64_t>::max() - delta) ||
        (delta < 0 && stamp < std::numeric_limits<int64_t>::min() - delta))
        throw std::out_of_range("Timestamp offset exceeds int64 range");
    return from_nanoseconds(stamp + delta);
}
}  // namespace fast_lio_time
