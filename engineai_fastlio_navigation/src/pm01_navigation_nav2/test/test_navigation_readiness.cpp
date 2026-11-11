#include <gtest/gtest.h>
#include "pm01_navigation_nav2/navigation_readiness.hpp"

using pm01_navigation::NavigationReadiness;
using pm01_navigation::ObservationStatus;

static void fresh(NavigationReadiness & gate, double wall, double ros)
{
  for (const std::string key : {"ground", "scan", "localization", "map"}) {
    ObservationStatus s;
    s.received = wall; s.stamp = ros; s.level = 0;
    s.reason = key == "scan" ? "projecting" : "tracking";
    s.values = {{"source_stamp", std::to_string(ros)}, {"cloud_stamp", std::to_string(ros)},
      {"ground_valid", "true"}, {"local_observation_usable", "true"},
      {"map_alignment_usable", "true"}, {"observation_usable", "true"}, {"matched", "true"}};
    gate.observations[key] = s;
  }
}

TEST(NavigationReadiness, RequiresContinuousHealthAndClosesImmediately)
{
  NavigationReadiness gate;
  EXPECT_FALSE(gate.update(10., 100.).ready);
  fresh(gate, 10., 100.);
  EXPECT_EQ(gate.update(10., 100.).state, "settling");
  fresh(gate, 11., 101.);
  EXPECT_TRUE(gate.update(11., 101.).ready);
  gate.observations["ground"].values["ground_valid"] = "false";
  EXPECT_FALSE(gate.update(11., 101.).ready);
  fresh(gate, 11.1, 101.1);
  EXPECT_FALSE(gate.update(11.1, 101.1).ready);
  fresh(gate, 12.1, 102.1);
  EXPECT_TRUE(gate.update(12.1, 102.1).ready);
}

TEST(NavigationReadiness, WeakLioRankDoesNotReplaceMeasuredObservationQualification)
{
  NavigationReadiness gate;
  fresh(gate, 10., 100.);
  gate.observations["ground"].values["motion_supported"] = "false";
  gate.observations["ground"].values["pose_rank"] = "5";
  gate.resume_delay = .1;
  gate.update(10., 100.);
  EXPECT_TRUE(gate.update(10.2, 100.2).ready);
  gate.observations["map"].values["local_observation_usable"] = "false";
  EXPECT_FALSE(gate.update(10.2, 100.2).ready);
}

TEST(NavigationReadiness, FreshStatusCannotHideStaleMeasurement)
{
  NavigationReadiness gate;
  fresh(gate, 10., 100.);
  gate.observations["scan"].values["cloud_stamp"] = "98";
  EXPECT_EQ(gate.update(10., 100.).reason, "scan_observation_stale");
  fresh(gate, 10., 100.);
  gate.observations["ground"].values["source_stamp"] = "null";
  EXPECT_EQ(gate.update(10., 100.).reason, "ground_observation_stale");
  fresh(gate, 10., 100.);
  gate.observations["ground"].values["source_stamp"] = "100broken";
  EXPECT_EQ(gate.update(10., 100.).reason, "ground_observation_stale");
}

TEST(NavigationReadiness, OneHertzMapDiagnosticsAndStationaryRevisionAreValid)
{
  NavigationReadiness gate;
  fresh(gate, 10., 100.);
  gate.update(10., 100.);
  auto map = gate.observations["map"];
  map.values["map_revision"] = "320";
  fresh(gate, 11.2, 101.2);
  gate.observations["map"] = map;
  EXPECT_TRUE(gate.update(11.2, 101.2).ready);
  fresh(gate, 13., 103.);
  gate.observations["map"] = map;
  EXPECT_FALSE(gate.update(13., 103.).ready);
}

TEST(NavigationReadiness, HeldAlignmentDoesNotAllowNavigation)
{
  NavigationReadiness gate;
  fresh(gate, 10., 100.);
  gate.observations["localization"].level = 1;
  gate.observations["localization"].values["matched"] = "false";
  gate.observations["localization"].values["held_alignment_usable"] = "true";
  gate.observations["localization"].reason = "recovering";
  EXPECT_EQ(gate.update(10., 100.).reason, "localization_unavailable:recovering");
}

TEST(NavigationReadiness, WallHeartbeatExpiresWhenRosClockStops)
{
  NavigationReadiness gate;
  fresh(gate, 10., 100.);
  gate.update(10., 100.);
  EXPECT_FALSE(gate.update(13., 100.).ready);
}

TEST(NavigationReadiness, StatusClockPhaseDoesNotRelaxMeasurementTime)
{
  NavigationReadiness gate;
  gate.resume_delay = .1;
  fresh(gate, 10., 100.);
  gate.observations["localization"].stamp = 100.1;
  EXPECT_EQ(gate.update(10., 100.).state, "settling");
  EXPECT_TRUE(gate.update(10.2, 100.).ready);
  gate.observations["scan"].values["cloud_stamp"] = "100.1";
  EXPECT_EQ(gate.update(10.2, 100.).reason, "scan_observation_stale");
  gate.observations["scan"].values["cloud_stamp"] = "100";
  gate.observations["localization"].stamp = 99.;
  EXPECT_EQ(gate.update(10.2, 100.).reason, "localization_status_stale");
  gate.observations["localization"].stamp = 100.3;
  EXPECT_EQ(gate.update(10.2, 100.).reason, "localization_status_stale");
}
