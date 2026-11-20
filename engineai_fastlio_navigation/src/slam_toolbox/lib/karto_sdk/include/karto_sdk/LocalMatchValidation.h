// PM01 local tracking validation. LGPL-3.0-or-later, as the containing Karto SDK.
// This state is deliberately not serialized into upstream pose-graph archives.
#ifndef KARTO_SDK_LOCAL_MATCH_VALIDATION_H_
#define KARTO_SDK_LOCAL_MATCH_VALIDATION_H_
#include <cmath>
#include <string>

namespace karto {
struct LocalMatchPose { double x = 0, y = 0, yaw = 0; };
inline double localAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline double localDistance(const LocalMatchPose &a, const LocalMatchPose &b) {
  return std::hypot(a.x-b.x, a.y-b.y);
}
struct LocalMatchValidation {
  bool enabled = false;
  double max_translation = .20, recovery_max_translation = .40, max_yaw = .035;
  double min_response = .15, min_support = .08, min_margin = .015;
  double ambiguity_yaw = .0175, cumulative_yaw = .035, recovery_cumulative_yaw = .0873, anchor_distance = 1.;
  int recovery_frames = 3;
  double recovery_radius = 4.;
  int recovery_max_references = 30;
  unsigned total_beams = 0, finite_beams = 0, valid_beams = 0;
  unsigned reference_count = 0;
  bool history_search_attempted = false;
  std::string reference_mode = "running", validation_mode = "normal";
  LocalMatchPose prediction, odometry, candidate, anchor_odom, recovery_pose, recovery_odom;
  double response = 0, support = 0, margin = 0, anchor_yaw = 0, cumulative_residual = 0;
  double last_time = -1, last_attempt_time = -1, current_time = 0;
  bool anchored = false, degraded = false, recovered = false, loop_closed = false;
  int recovery_count = 0, rejected_near_chains = 0;
  std::string reason = "initializing";

  static LocalMatchPose correction(const LocalMatchPose &p, const LocalMatchPose &o) {
    const double a = localAngle(p.yaw-o.yaw), c = std::cos(a), s = std::sin(a);
    return {p.x-c*o.x+s*o.y, p.y-s*o.x-c*o.y, a};
  }
  void begin(const LocalMatchPose &p, const LocalMatchPose &o, double time,
             unsigned total, unsigned finite, unsigned valid) {
    prediction = p; odometry = o; current_time = time;
    // Freeze the threshold mode for this measurement. A first ordinary
    // failure cannot immediately retry itself with a wider translation bound.
    validation_mode = degraded ? "recovery" : "normal";
    total_beams = total; finite_beams = finite; valid_beams = valid;
    support = total ? double(valid)/total : 0.;
    recovered = loop_closed = history_search_attempted = false;
    rejected_near_chains = 0; reference_count = 0; reference_mode = "running";
    response = margin = cumulative_residual = 0.; candidate = p;
    reason = "not_processed";
  }
  // Keep upstream correlation scores, covariances and loop thresholds intact.
  // Missing measurements are not mismatches. Real returns outside the grid
  // remain in the denominator: lack of overlap must still lower quality.
  double normalized(double raw) const {
    return finite_beams ? raw * double(total_beams)/finite_beams : 0.;
  }
  // Graph/near-chain callers always retain the ordinary tracking bound.
  std::string check(const LocalMatchPose &p, double score, double separation) const {
    return checkWithLimits(p, score, separation, max_translation, cumulative_yaw);
  }
  // Only the primary candidate may use the recovery bound. Its acceptance
  // still requires three distinct, consistent observations before graph writes.
  std::string checkPrimary(const LocalMatchPose &p, double score, double separation) const {
    return checkWithLimits(p, score, separation,
      validation_mode == "recovery" ? recovery_max_translation : max_translation,
      validation_mode == "recovery" ? recovery_cumulative_yaw : cumulative_yaw);
  }
  bool recoveryConsistent(const LocalMatchPose &p) const {
    if (!recovery_count) return true;
    // Transport the first candidate with the measured LIO increment, and
    // compare at the robot rather than the distant map/odom world origin.
    const double a = localAngle(recovery_pose.yaw-recovery_odom.yaw);
    const double dx = odometry.x-recovery_odom.x, dy = odometry.y-recovery_odom.y;
    const LocalMatchPose expected{
      recovery_pose.x+std::cos(a)*dx-std::sin(a)*dy,
      recovery_pose.y+std::sin(a)*dx+std::cos(a)*dy,
      localAngle(recovery_pose.yaw+odometry.yaw-recovery_odom.yaw)};
    return localDistance(p, expected) <= max_translation*.5 &&
      std::abs(localAngle(p.yaw-expected.yaw)) <= max_yaw*.5;
  }
  double cumulativeYawResidual(const LocalMatchPose &p) const {
    return anchored ? localAngle(correction(p, odometry).yaw-anchor_yaw) : 0.;
  }
  double translationResidual() const { return localDistance(candidate, prediction); }
  double yawResidual() const { return localAngle(candidate.yaw-prediction.yaw); }
private:
  std::string checkWithLimits(const LocalMatchPose &p, double score, double separation,
                             double translation_limit, double cumulative_limit) const {
    if (!enabled) return "";
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw) ||
        !std::isfinite(score) || !std::isfinite(separation)) return "nonfinite_candidate";
    if (current_time <= last_time) return "nonmonotonic_measurement";
    if (!finite_beams || valid_beams < 12 || support < min_support) return "insufficient_support";
    if (normalized(score) < min_response) return "weak_response";
    const double turn = std::abs(localAngle(p.yaw-prediction.yaw));
    if (localDistance(p, prediction) > translation_limit) return "translation_residual";
    if (turn > max_yaw) return "yaw_residual";
    if (turn > ambiguity_yaw && normalized(separation) < min_margin) return "ambiguous_peak";
    // During a turn, do not reset the reference on every accepted small correction.
    // A completed recovery, metre of travel or accepted global loop starts a new segment.
    if (anchored && localDistance(odometry, anchor_odom) < anchor_distance &&
        std::abs(cumulativeYawResidual(p)) > cumulative_limit)
      return "cumulative_yaw_residual";
    return "";
  }
public:
  bool acceptCandidate(const LocalMatchPose &p, double score, double separation) {
    candidate = p; response = score; margin = separation;
    cumulative_residual = cumulativeYawResidual(p);
    // Repeated delivery or alternate references for one measurement cannot
    // count as additional recovery observations. Graph checks within this
    // measurement still use check(), which is deliberately read-only.
    if (enabled && current_time <= last_attempt_time) {
      reason = "nonmonotonic_measurement";
      return false;
    }
    reason = checkPrimary(p, score, separation);
    last_attempt_time = current_time;
    if (!reason.empty()) { degraded = true; recovery_count = 0; return false; }
    if (enabled && degraded) {
      // Anchor the whole sequence, so pairwise-small changes cannot creep
      // past the agreement bound over several observations.
      if (!recoveryConsistent(p)) recovery_count = 0;
      if (!recovery_count) { recovery_pose = p; recovery_odom = odometry; }
      if (++recovery_count < recovery_frames) { reason = "recovering"; return false; }
      recovered = true;
    }
    reason = recovered ? "recovered" : "accepted";
    return true;
  }
  void commit(const LocalMatchPose &p, bool loop) {
    if (!anchored || loop || recovered || localDistance(odometry, anchor_odom) >= anchor_distance) {
      anchor_odom = odometry; anchor_yaw = correction(p, odometry).yaw; anchored = true;
    }
    degraded = false; recovery_count = 0; last_time = last_attempt_time = current_time; loop_closed = loop;
    if (loop) reason = "loop_closure";
  }
};
} // namespace karto
#endif
