// PM01 graph anchor metadata. LGPL-3.0-or-later, as the Karto SDK.
#ifndef SLAM_TOOLBOX_GRAPH_ANCHOR_SNAPSHOT_HPP_
#define SLAM_TOOLBOX_GRAPH_ANCHOR_SNAPSHOT_HPP_
#include <karto_sdk/Mapper.h>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

namespace slam_toolbox {
// Call under the same mapper lock as the graph update and map snapshot.
// Only poses are exported; accepted measurements remain inside the backend.
inline std::string graphAnchorSnapshot(const std::string &epoch,
  uint64_t revision, uint64_t optimization_revision,
  const karto::LocalizedRangeScanVector &scans)
{
  std::ostringstream out;
  out << std::setprecision(17)
      << "{\"schema\":1,\"backend\":\"pm01-slam-2.6.10\",\"epoch\":\"" << epoch
      << "\",\"revision\":" << revision << ",\"optimization_revision\":"
      << optimization_revision << ",\"anchors\":[";
  bool first = true;
  auto number = [&](double value) {
    if (std::isfinite(value)) out << value; else out << "null";
  };
  for (const auto *scan : scans) {
    if (!first) out << ',';
    first = false;
    const auto pose = scan->GetCorrectedPose();
    out << "{\"id\":" << scan->GetUniqueId() << ",\"pose\":[";
    number(pose.GetX()); out << ','; number(pose.GetY()); out << ',';
    number(pose.GetHeading()); out << "]}";
  }
  out << "]}";
  return out.str();
}
}  // namespace slam_toolbox
#endif
