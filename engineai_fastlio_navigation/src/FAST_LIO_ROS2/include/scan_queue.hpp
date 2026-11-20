#pragma once
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <optional>

namespace fast_lio_queue {
// These helpers only discard unprocessed lidar frames. IMU retention belongs
// to the synchronizer and is independent of the selected lidar frame.
template<class Queue>
void trim(Queue & frames, std::size_t limit, uint64_t & skipped) {
    while (frames.size() > limit) {frames.pop_front(); ++skipped;}
}

template<class Queue, class EndTime>
void select(Queue & frames, double now, double max_wait, double imu_end,
            EndTime end_time, uint64_t & skipped) {
    if (frames.empty() || now-frames.front().received_steady <= max_wait) return;
    std::size_t selected = 0;
    for (std::size_t i = 1; i < frames.size(); ++i)
        if (end_time(frames[i]) <= imu_end) selected = i;
    while (selected--) {frames.pop_front(); ++skipped;}
}
enum class Coverage { complete, waiting, missing };
template<class Queue, class Stamp>
Coverage coverage(const Queue & imu, double boundary, double begin, double end,
                  double max_gap, Stamp stamp) {
    if (imu.empty() || stamp(imu.back()) < end) return Coverage::waiting;
    if (boundary < 0. && stamp(imu.front()) > begin) return Coverage::waiting;
    double previous = boundary < 0. ? stamp(imu.front()) : boundary;
    for (const auto & sample : imu) {
        const double current = stamp(sample);
        if (!std::isfinite(current) || current < previous || current-previous > max_gap)
            return Coverage::missing;
        if (current >= end) return Coverage::complete;
        previous = current;
    }
    return Coverage::waiting;
}

// A new scan can be deskewed after a gap once its own interval is complete.
// Retain the sample immediately before scan start; older gaps no longer belong
// to this interval. The caller must rebase prediction and validate against the
// retained lidar map before publishing a recovered pose.
template<class Queue, class Stamp>
std::optional<std::size_t> covered_window(const Queue & imu, double begin,
                                        double end, double max_gap, Stamp stamp) {
    if (imu.empty() || stamp(imu.back()) < end) return {};
    std::optional<std::size_t> first;
    for (std::size_t i=0; i<imu.size() && stamp(imu[i])<=begin; ++i) first=i;
    if (!first) return {};
    double previous=stamp(imu[*first]);
    if (!std::isfinite(previous) || begin-previous>max_gap) return {};
    if (previous>=end) return first;
    for (std::size_t i=*first+1; i<imu.size(); ++i) {
        const double current=stamp(imu[i]);
        if (!std::isfinite(current) || current<=previous || current-previous>max_gap) return {};
        if (current>=end) return first;
        previous=current;
    }
    return {};
}

}
