// PM01 local matching regression tests. LGPL-3.0-or-later.
#include <karto_sdk/LocalMatchValidation.h>
#include <stdexcept>
#include <iostream>

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}
int main() {
  try {
    karto::LocalMatchValidation gate;
    gate.enabled = true;
    for (unsigned finite : {720u, 360u, 180u, 90u, 60u}) {
      gate.begin({}, {}, 1., 720, finite, finite);
      const double raw = .99*finite/720.;
      require(gate.check({}, raw, .02*finite/720.).empty(), "Sparse equivalent geometry rejected");
      require(std::abs(gate.normalized(raw)-.99) < 1e-12, "Quality changes with NaN padding");
    }
    gate.begin({}, {}, 1., 720, 50, 50);
    require(gate.check({}, 50./720., .02).compare("insufficient_support") == 0, "Insufficient support accepted");
    gate.begin({}, {}, 1., 720, 90, 90);
    require(gate.check({}, .001, .02) == "weak_response", "Low overlap accepted");
    require(gate.check({.295, 0, 1.064}, .12, .02) == "translation_residual", "Invalid recovery pose accepted");
    require(gate.check({0, 0, .02}, .12, .001) == "ambiguous_peak", "Sparse ambiguous peak accepted");
    require(gate.check({0, 0, .04}, .12, .02) == "yaw_residual", "Yaw guard widened");
    gate.commit({}, false);
    gate.begin({}, {}, 2., 720, 90, 90);
    require(!gate.acceptCandidate({}, .001, .02), "Bad observation accepted");
    for (int i = 0; i < 3; ++i) {
      gate.begin({}, {}, 3.+i, 720, 90, 90);
      const bool accepted = gate.acceptCandidate({}, .12, .02);
      require(accepted == (i == 2), "Recovery did not require three observations");
      if (i < 2) {
        require(gate.reason == "recovering", "Missing recovery state");
        const auto count = gate.recovery_count;
        require(!gate.acceptCandidate({}, .12, .02), "Duplicate timestamp counted as recovery");
        require(gate.recovery_count == count, "Duplicate damaged recovery sequence");
        require(gate.last_time == 1., "Uncommitted observation changed accepted time");
      }
    }
    require(gate.recovered, "Final recovery not reported");
    require(gate.response == .12, "Raw diagnostic score changed");
    // Near-chain validation occurs after acceptCandidate but before commit.
    require(gate.check({}, .12, .02).empty(), "Same-frame graph validation incorrectly rejected");
    gate.commit({}, false);
    require(!gate.degraded && gate.last_time == 5., "Recovery was not committed");
    gate.begin({}, {}, 6., 720, 90, 90);
    require(gate.check({0, 0, .04}, .12, .02) == "yaw_residual", "Recovery loosened protection");
    karto::LocalMatchValidation recovery;
    recovery.enabled = true;
    recovery.begin({}, {}, 1., 720, 90, 90);
    require(!recovery.acceptCandidate({.30, 0, 0}, .12, .02), "Ordinary bound was widened");
    require(recovery.validation_mode == "normal" && recovery.recovery_count == 0,
            "First ordinary failure counted with the wider recovery bound");
    for (int i = 0; i < 3; ++i) {
      const karto::LocalMatchPose odom{10.+i*.1, -7.-i*.1, i*.1};
      const karto::LocalMatchPose candidate{10.3+i*.1, -7.-i*.1, i*.1};
      recovery.begin(odom, odom, 2.+i, 720, 90, 90);
      require(recovery.check(candidate, .12, .02) == "translation_residual",
              "Near-chain inherited wider recovery bound");
      require(recovery.acceptCandidate(candidate, .12, .02) == (i == 2),
              "LIO-consistent recovery did not require three distinct frames");
      require(recovery.validation_mode == "recovery", "Recovery metadata mode missing");
    }
    require(recovery.check({10.39, -7.2, .2}, .12, .02).empty(),
            "Ordinary near-chain check fixture is not within its .20 m bound");
    require(!recovery.recoveryConsistent({10.39, -7.2, .2}),
            "Final near-chain fusion may replace the recovery consensus");
    require(recovery.recoveryConsistent({10.41, -7.2, .2}),
            "Consensus-compatible fusion was unnecessarily excluded");
    recovery.commit(recovery.candidate, false);
    recovery.begin({}, {}, 5., 720, 90, 90);
    require(!recovery.acceptCandidate({.30, 0, 0}, .12, .02), "Ordinary bound stayed widened after recovery");
    recovery.begin({}, {}, 6., 720, 90, 90);
    require(!recovery.acceptCandidate({.41, 0, 0}, .12, .02), "Recovery bound was ignored");
    recovery.begin({}, {}, 7., 720, 90, 90);
    require(!recovery.acceptCandidate({.30, 0, .04}, .12, .02) && recovery.reason == "yaw_residual",
            "Recovery widened yaw protection");
    // Pairwise-consistent increments (.08 m) must not accumulate past .10 m
    // from the first recovery hypothesis over the three observations.
    for (int i = 0; i < 3; ++i) {
      recovery.begin({}, {}, 8.+i, 720, 90, 90);
      require(!recovery.acceptCandidate({.21+i*.08, 0, 0}, .12, .02),
              "Inconsistent recovery sequence crept through");
    }
    require(recovery.recovery_count == 1, "Inconsistent sequence did not restart");
    recovery.begin({}, {}, 11., 720, 90, 90);
    require(!recovery.acceptCandidate({.37, 0, 0}, .12, .02), "Only two stable observations accepted");
    recovery.begin({}, {}, 12., 720, 90, 90);
    require(recovery.acceptCandidate({.37, 0, 0}, .12, .02), "Stable restarted sequence did not recover");
    // A valid incremental correction can exceed the old segment's cumulative
    // limit while stationary. Recovery must not remain locked to that segment.
    karto::LocalMatchValidation cumulative;
    cumulative.enabled = true;
    cumulative.begin({0, 0, .02}, {}, 1., 720, 90, 90);
    cumulative.commit({0, 0, .02}, false);
    const karto::LocalMatchPose held{0, 0, -.01}, reliable{0, 0, -.04};
    cumulative.begin(held, {}, 2., 720, 90, 90);
    require(!cumulative.acceptCandidate(reliable, .12, .02) &&
            cumulative.reason == "cumulative_yaw_residual", "Ordinary cumulative protection was widened");
    for (int i = 0; i < 3; ++i) {
      cumulative.begin(held, {}, 3.+i, 720, 90, 90);
      require(cumulative.check(reliable, .12, .02) == "cumulative_yaw_residual",
              "Near-chain inherited recovery cumulative allowance");
      require(cumulative.acceptCandidate(reliable, .12, .02) == (i == 2),
              "Cumulative recovery did not require three fresh observations");
      require(std::abs(cumulative.anchor_yaw-.02) < 1e-12 && cumulative.last_time == 1.,
              "Recovery reset the anchor before graph commit");
      if (i < 2) {
        const int count = cumulative.recovery_count;
        require(!cumulative.acceptCandidate(reliable, .12, .02) && cumulative.recovery_count == count,
                "Duplicate cumulative recovery frame counted");
      }
    }
    cumulative.commit(reliable, false);
    require(std::abs(cumulative.anchor_yaw+.04) < 1e-12 &&
            std::abs(cumulative.cumulative_residual+.06) < 1e-12,
            "Recovery commit did not reset anchor or preserve diagnostic residual");
    cumulative.begin(reliable, {}, 6., 720, 90, 90);
    require(cumulative.validation_mode == "normal" && cumulative.acceptCandidate(reliable, .12, .02),
            "Next ordinary frame remained locked to the old cumulative anchor");
    cumulative.commit(reliable, false);
    cumulative.begin({0, 0, -.11}, {}, 7., 720, 90, 90);
    require(!cumulative.acceptCandidate({0, 0, -.14}, .12, .02), "Ordinary cumulative limit missing");
    cumulative.begin({0, 0, -.11}, {}, 8., 720, 90, 90);
    require(!cumulative.acceptCandidate({0, 0, -.14}, .12, .02) &&
            cumulative.reason == "cumulative_yaw_residual", "Recovery cumulative limit missing");
    require(std::abs(cumulative.anchor_yaw+.04) < 1e-12, "Rejected recovery reset the anchor");
    for (double degrees : {9.2, 10.8}) {
      cumulative.begin({}, {}, 9.+degrees, 720, 90, 90);
      require(!cumulative.acceptCandidate({0, 0, degrees*std::acos(-1.)/180.}, .12, .02) &&
              cumulative.reason == "yaw_residual", "Large original turning failure was accepted");
    }
    // Consistency is measured at the robot, not at the map/odom world origin.
    karto::LocalMatchValidation distant;
    distant.enabled = distant.degraded = true;
    for (int i = 0; i < 3; ++i) {
      const karto::LocalMatchPose odom{100., 100., 0.};
      distant.begin(odom, odom, i+1., 720, 90, 90);
      require(distant.acceptCandidate({100.3, 100., i*.005}, .12, .02) == (i == 2),
              "Small yaw noise far from origin broke a consistent local recovery");
    }
    std::cout << "local match validation passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
