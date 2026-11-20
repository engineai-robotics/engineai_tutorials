#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <tf2/buffer_core.h>
#include <tf2/time.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "time_utils.hpp"

namespace {
void require(bool value, const std::string & message) {
    if (!value) throw std::runtime_error(message);
}

template<typename Exception, typename Callback>
void rejects(Callback callback, const std::string & message) {
    bool rejected = false;
    try { callback(); } catch (const Exception &) { rejected = true; }
    require(rejected, message);
}

// Exercise a real TF buffer at the original failure boundary, not just arithmetic.
void test_live_samples_and_tf_boundary() {
    const int64_t samples[] = {774212000000LL, 775026000000LL, 613204000000LL,
                              775598000000LL, 776496000000LL};
    for (const auto source_ns : samples) {
        const builtin_interfaces::msg::Time source = rclcpp::Time(source_ns);
        const double seconds = rclcpp::Time(source).seconds();
        const auto old_sec = static_cast<int32_t>(std::floor(seconds));
        const auto old_nsec = static_cast<uint32_t>((seconds - std::floor(seconds))*1e9);
        const auto old_ns = rclcpp::Time(old_sec, old_nsec).nanoseconds();
        require(old_ns < source_ns, "Live sample no longer reproduces old truncation");
        require(fast_lio_time::from_seconds(seconds).nanoseconds() == source_ns,
                "Floating conversion still truncates a live sample");

        geometry_msgs::msg::TransformStamped tf;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "lidar";
        tf.transform.rotation.w = 1.;
        tf.header.stamp = rclcpp::Time(old_ns);
        tf2::BufferCore old_buffer;
        require(old_buffer.setTransform(tf, "test"), "Cannot insert old transform");
        const tf2::TimePoint when{tf2::Duration(source_ns)};
        require(!old_buffer.canTransform("odom", "lidar", when),
                "Old TF unexpectedly accepts the future point-cloud timestamp");

        tf.header.stamp = fast_lio_time::add_seconds(source, 0.);
        tf2::BufferCore fixed_buffer;
        require(fixed_buffer.setTransform(tf, "test"), "Cannot insert fixed transform");
        require(fixed_buffer.canTransform("odom", "lidar", when),
                "Exact point-cloud timestamp still fails TF lookup");
    }
}

void test_epoch_headers_preserve_every_nanosecond() {
    for (int64_t offset = 0; offset < 10000; ++offset) {
        const int64_t source_ns = 1793180372000000000LL + offset;
        const builtin_interfaces::msg::Time source = rclcpp::Time(source_ns);
        require(fast_lio_time::add_seconds(source, 0.).nanoseconds() == source_ns,
                "Snapshot/no-offset IMU header lost precision at Unix-epoch scale");
        require(fast_lio_time::add_seconds(source, .1).nanoseconds() == source_ns + 100000000,
                "Rolling scan end time lost source precision");
        require(fast_lio_time::add_seconds(source, -.002).nanoseconds() == source_ns - 2000000,
                "Negative IMU correction lost source precision");
    }
}

void test_second_carry_and_signed_offsets() {
    require(fast_lio_time::from_seconds(1.9999999996).nanoseconds() == 2000000000,
            "Nanosecond rounding does not carry into the next second");
    const builtin_interfaces::msg::Time source = rclcpp::Time(1999999999LL);
    const builtin_interfaces::msg::Time carried = fast_lio_time::add_seconds(source, 2e-9);
    require(carried.sec == 2 && carried.nanosec == 1, "Offset second carry is invalid");
    require(fast_lio_time::rounded_nanoseconds(-.002) == -2000000,
            "Negative duration has wrong sign or units");
    require(fast_lio_time::from_seconds(0.).nanoseconds() == 0, "Zero time is invalid");
}

void test_invalid_times_and_overflow() {
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity()})
        rejects<std::invalid_argument>([&]() { fast_lio_time::from_seconds(invalid); },
                                       "Non-finite timestamp was accepted");
    rejects<std::out_of_range>([]() { fast_lio_time::from_seconds(-1.); },
                              "Negative absolute timestamp was accepted");
    rejects<std::out_of_range>([]() { fast_lio_time::rounded_nanoseconds(1e20); },
                              "Integer overflow was accepted");
    const builtin_interfaces::msg::Time zero = rclcpp::Time(static_cast<int64_t>(0));
    rejects<std::out_of_range>([&]() { fast_lio_time::add_seconds(zero, -1e-9); },
                              "Timestamp underflow was accepted");
    const builtin_interfaces::msg::Time maximum = rclcpp::Time(fast_lio_time::max_stamp_ns);
    rejects<std::out_of_range>([&]() { fast_lio_time::add_seconds(maximum, 1e-9); },
                              "ROS message seconds overflow was accepted");
}
}  // namespace

int main() {
    try {
        test_live_samples_and_tf_boundary();
        test_epoch_headers_preserve_every_nanosecond();
        test_second_carry_and_signed_offsets();
        test_invalid_times_and_overflow();
        std::cout << "Live TF boundary, 10000 epoch headers, offsets and time bounds passed\n";
    } catch (const std::exception & error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
