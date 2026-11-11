// Exercises the actual Karto matcher and graph, not a copied scoring model.
// LGPL-3.0-or-later, as the containing Karto SDK.
#include <karto_sdk/Mapper.h>
#include <slam_toolbox/graph_anchor_snapshot.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}
karto::RangeReadingsVector room(unsigned stride=1) {
  karto::RangeReadingsVector readings(720, std::numeric_limits<double>::quiet_NaN());
  for (unsigned i = 0; i < 720; i += stride) {
    const double a = -karto::KT_PI+i*2*karto::KT_PI/720;
    readings[i] = std::min(3./std::max(std::abs(std::cos(a)), 1e-9),
                           4./std::max(std::abs(std::sin(a)), 1e-9));
  }
  return readings;
}
struct Fixture {
  karto::Name name;
  karto::LaserRangeFinder *laser;
  karto::Mapper mapper;
  std::vector<std::unique_ptr<karto::LocalizedRangeScan>> scans;
  explicit Fixture(const char *sensor) : name(sensor) {
    laser = karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, name);
    laser->SetMinimumRange(.6); laser->SetMaximumRange(12.); laser->SetRangeThreshold(12.);
    laser->SetMinimumAngle(-karto::KT_PI);
    laser->SetMaximumAngle(karto::KT_PI-2*karto::KT_PI/720);
    laser->SetAngularResolution(2*karto::KT_PI/720);
    karto::SensorManager::GetInstance()->RegisterSensor(laser, true);
    mapper.setParamUseScanBarycenter(false);
    mapper.setParamMinimumTravelDistance(0); mapper.setParamMinimumTravelHeading(0);
    mapper.setParamDoLoopClosing(false); mapper.setParamScanBufferSize(1);
    mapper.setParamCorrelationSearchSpaceDimension(.5);
    mapper.setParamCorrelationSearchSpaceResolution(.01);
    mapper.setParamCorrelationSearchSpaceSmearDeviation(.1);
    mapper.local_validation.enabled = true;
  }
  ~Fixture() {
    mapper.Reset();
    karto::SensorManager::GetInstance()->UnregisterSensor(laser);
    delete laser;
  }
  karto::LocalizedRangeScan *scan(const karto::RangeReadingsVector &ranges, double t, double x=0., double y=0.) {
    scans.emplace_back(new karto::LocalizedRangeScan(name, ranges));
    auto *s = scans.back().get();
    s->SetOdometricPose(karto::Pose2(x, y, 0)); s->SetCorrectedPose(karto::Pose2(x, y, 0));
    s->SetTime(t); return s;
  }
};
int main() {
  try {
    for (unsigned stride : {1u, 2u, 4u, 8u, 12u}) {
      Fixture f("sparse");
      require(f.mapper.Process(f.scan(room(stride), 1)), "Cannot seed matcher");
      require(f.mapper.Process(f.scan(room(stride), 2)), "Equivalent sparse scan rejected");
      const auto &g = f.mapper.local_validation;
      std::cout << "stride=" << stride << " raw=" << g.response
                << " normalized=" << g.normalized(g.response) << std::endl;
      require(g.normalized(g.response) > .95, "Finite quality degraded with missing measurements");
      require(g.response < 1.01/stride, "Upstream raw score was changed");
    }
    {
      Fixture recovery("translation-recovery");
      require(recovery.mapper.Process(recovery.scan(room(), 1)), "Cannot seed translation recovery");
      require(!recovery.mapper.Process(recovery.scan(room(), 2, .24, .15)), "Ordinary .20 m bound was widened");
      require(recovery.mapper.local_validation.reason == "translation_residual", "Expected translation failure");
      require(recovery.mapper.local_validation.validation_mode == "normal", "First failure used recovery mode");
      for (int i = 0; i < 3; ++i) {
        const bool accepted = recovery.mapper.Process(recovery.scan(room(), 3+i, .24, .15));
        require(accepted == (i == 2), "Extended recovery did not wait for three observations");
        require(recovery.mapper.GetAllProcessedScans().size() == (i == 2 ? 2u : 1u),
                "Extended recovery polluted scan history before acceptance");
        require(recovery.mapper.local_validation.validation_mode == "recovery", "Missing extended recovery mode");
        if (i < 2) require(recovery.mapper.GetGraph()->GetEdges().empty(), "Extended recovery polluted graph edges");
      }
      require(recovery.mapper.local_validation.recovered, "Extended recovery did not report recovered");
    }
    {
      Fixture sticky("cumulative-recovery");
      require(sticky.mapper.Process(sticky.scan(room(), 1)), "Cannot seed cumulative recovery");
      auto &gate = sticky.mapper.local_validation;
      // Reproduce an old segment reference whose correction differs from a
      // correct current match by 3.4 degrees; current scan itself is unchanged.
      gate.anchor_yaw = .06;
      require(!sticky.mapper.Process(sticky.scan(room(), 2)) &&
              gate.reason == "cumulative_yaw_residual", "Missing ordinary cumulative rejection");
      for (int i = 0; i < 3; ++i) {
        require(sticky.mapper.Process(sticky.scan(room(), 3+i)) == (i == 2),
                "Cumulative recovery bypassed three-frame agreement");
        require(sticky.mapper.GetAllProcessedScans().size() == (i == 2 ? 2u : 1u),
                "Cumulative recovery wrote an unaccepted scan");
        if (i < 2) require(gate.anchor_yaw == .06 && sticky.mapper.GetGraph()->GetEdges().empty(),
                           "Cumulative recovery mutated the anchor or graph too early");
      }
      require(gate.recovered && std::abs(gate.anchor_yaw) < .01,
              "Accepted recovery did not replace the stale segment anchor");
      require(sticky.mapper.Process(sticky.scan(room(), 6)) && gate.validation_mode == "normal",
              "Normal matching froze again immediately after cumulative recovery");
    }
    {
      // A pose table must preserve each keyframe's independent correction;
      // one latest rigid map/odom transform cannot represent graph deformation.
      Fixture anchors("anchor-snapshot");
      anchors.mapper.local_validation.enabled = false;
      auto *first = anchors.scan(room(), 1);
      auto *second = anchors.scan(room(), 2, 1.);
      require(anchors.mapper.Process(first) && anchors.mapper.Process(second), "Cannot seed anchor snapshot");
      first->SetCorrectedPose(karto::Pose2(1., 2., .1));
      second->SetCorrectedPose(karto::Pose2(4., 5., -.2));
      const auto before = slam_toolbox::graphAnchorSnapshot("epoch", 2, 2, anchors.mapper.GetAllProcessedScans());
      require(before.find("\"id\":"+std::to_string(first->GetUniqueId())+",\"pose\":[1,2,") != std::string::npos &&
              before.find("\"id\":"+std::to_string(second->GetUniqueId())+",\"pose\":[4,5,") != std::string::npos,
              "Snapshot lost keyframe identity or independent corrected poses");
      second->SetCorrectedPose(karto::Pose2(-3., 7., .4));
      const auto after = slam_toolbox::graphAnchorSnapshot("epoch", 3, 3, anchors.mapper.GetAllProcessedScans());
      require(after.find("\"optimization_revision\":3") != std::string::npos &&
              after.find("\"pose\":[1,2,") != std::string::npos && after.find("\"pose\":[-3,7,") != std::string::npos,
              "Optimization snapshot flattened independent anchor corrections");
      require(before.find("\"pose\":[4,5,") != std::string::npos && before.find("ranges") == std::string::npos,
              "Snapshot was not immutable pose-only metadata");
    }
    Fixture f("history");
    f.mapper.local_validation.enabled = false;
    for (int i = 0; i < 35; ++i)
      require(f.mapper.Process(f.scan(room(), i+1)), "Cannot seed accepted history");
    auto *last = f.scan(karto::RangeReadingsVector(720, .8), 36, 4.5);
    require(f.mapper.Process(last), "Cannot seed distant last reference");
    // Establish an exact known held alignment for this reference-selection fixture.
    last->SetCorrectedPose(last->GetOdometricPose());
    f.mapper.local_validation.enabled = true;
    f.mapper.local_validation.anchored = false;
    const auto edges = f.mapper.GetGraph()->GetEdges().size();
    for (int i = 0; i < 3; ++i) {
      auto *candidate = f.scan(room(), 37+i);
      const bool accepted = f.mapper.Process(candidate);
      const auto &g = f.mapper.local_validation;
      require(g.history_search_attempted && g.reference_mode == "history", "History recovery not exercised");
      require(g.reference_count == 30, "History search is not bounded to 30 references");
      require(accepted == (i == 2), "History recovery must require three fresh observations");
      require(f.mapper.GetAllProcessedScans().size() == (i == 2 ? 37u : 36u), "Rejected candidate polluted scan history");
      if (i < 2) {
        require(f.mapper.GetGraph()->GetEdges().size() == edges, "Rejected candidate polluted graph edges");
        require(last->GetOdometricPose() == karto::Pose2(4.5, 0, 0), "Recovery modified odometry");
        require(last->GetCorrectedPose() == karto::Pose2(4.5, 0, 0), "Recovery modified accepted pose");
      }
    }
    auto *unsupported = f.scan(room(), 41, 20.);
    require(!f.mapper.Process(unsupported), "No-overlap observation accepted");
    require(f.mapper.GetAllProcessedScans().size() == 37, "No-overlap observation polluted history");
    std::cout << "mapper recovery passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
