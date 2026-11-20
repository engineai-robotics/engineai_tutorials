// Free observations need full footprint coverage at the same height and anchor.
// Cache forward and inverse planar geometry only for cells visited by this
// anchor.
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>

#include "../pm01_navigation_core/mapping/ray_samples.cuh"

namespace py = pybind11;
namespace {
using Point = std::array<double, 2>;
using Pose = std::array<double, 3>;
struct Grid {
  int h, w;
  double res, x, y, c, s;
  std::array<Point, 5> offsets;
  explicit Grid(std::array<double, 6> g)
      : h(g[0]),
        w(g[1]),
        res(g[2]),
        x(g[3]),
        y(g[4]),
        c(std::cos(g[5])),
        s(std::sin(g[5])) {
    if (h <= 0 || w <= 0 || !std::isfinite(res) || res <= 0 ||
        int64_t(h) * w > std::numeric_limits<int>::max())
      throw std::invalid_argument("invalid grid");
    const double k = .49999;
    offsets = {{{0, 0}, {-k, -k}, {-k, k}, {k, -k}, {k, k}}};
    for (auto& o : offsets)
      o = {(o[0] * c - o[1] * s) * res, (o[0] * s + o[1] * c) * res};
  }
  int size() const { return h * w; }
  Point centre(int cell) const {
    double dx = (cell % w + .5) * res, dy = (cell / w + .5) * res;
    return {x + c * dx - s * dy, y + s * dx + c * dy};
  }
  int cell(Point p) const {
    double dx = p[0] - x, dy = p[1] - y;
    double u = (c * dx + s * dy) / res, v = (-s * dx + c * dy) / res;
    if (!(u >= 0 && u < w && v >= 0 && v < h)) return -1;
    return int(v) * w + int(u);
  }
};
struct Transform {
  Pose from, to;
  double c, s;
  Transform(Pose a, Pose b)
      : from(a), to(b), c(std::cos(b[2] - a[2])), s(std::sin(b[2] - a[2])) {}
  Point apply(Point p, Point offset) const {
    double x = (p[0] + offset[0]) - from[0], y = (p[1] + offset[1]) - from[1];
    return {c * x - s * y + to[0], s * x + c * y + to[1]};
  }
};
// Reuse two planar masks across anchors. Only touched columns are reset.
struct Columns {
  std::vector<uint64_t> source, target;
  std::vector<int> sources, targets;
  Columns(size_t src, size_t dst) : source(src), target(dst) {}
};
// Only borrow contiguous, correctly typed storage. Never silently write to a
// copy.
template <typename T>
py::array_t<T, py::array::c_style> array(py::handle value, ssize_t size = -1,
                                         bool writable = false) {
  using A = py::array_t<T, py::array::c_style>;
  if (!A::check_(value))
    throw std::invalid_argument("invalid evidence array type or layout");
  auto a = py::reinterpret_borrow<A>(value);
  if ((size >= 0 && a.size() != size) || (writable && !a.writeable()))
    throw std::invalid_argument("invalid evidence array size or access");
  return a;
}
struct HostMasks {
  bool* low; bool* mid;
  void mark(int which, int id) const { (which ? mid : low)[id] = true; }
};
void trace_rays(py::array rays, py::array counts, py::array geometry, int w, int h,
                int bins, py::array low, py::array mid) {
  if (w <= 0 || h <= 0 || bins <= 0 || int64_t(w)*h > INT32_MAX/bins)
    throw std::invalid_argument("invalid ray grid");
  auto num = array<int32_t>(counts);
  auto delta = array<double>(rays, num.size()*3);
  auto g = array<double>(geometry, 14);
  auto lo = array<bool>(low, w*h, true), hi = array<bool>(mid, w*h*bins, true);
  HostMasks masks{lo.mutable_data(), hi.mutable_data()};
  const auto *steps = num.data(); const double *points = delta.data(), *params = g.data();
  py::gil_scoped_release release;
  for (ssize_t i = 0; i < num.size(); ++i)
    trace_ray(points+3*i, steps[i], params, w,h,bins,masks);
}
// Stable counting sort by the owning graph node, without a full-volume ID copy.
py::dict group_voxels(py::array_t<int32_t, py::array::c_style> anchors,
                      py::array observations, py::array hits,
                      py::array occupied, py::array evidence) {
  auto free = array<uint32_t>(observations);
  auto hit = array<uint32_t>(hits, free.size());
  auto occ = array<bool>(occupied, free.size());
  auto score = array<int16_t>(evidence, free.size());
  std::map<int32_t, size_t> index;
  std::vector<size_t> slot(anchors.size());
  for (ssize_t i = 1; i < anchors.size(); ++i)
    slot[i] = index.emplace(anchors.data()[i], index.size()).first->second;
  std::vector<ssize_t> lengths(index.size()), next(index.size());
  const auto* free_ids = free.data(); const auto* hit_ids = hit.data();
  const auto* taken = occ.data(); const auto* scores = score.data();
  const auto count = free.size();
  auto id_at = [&](ssize_t i) { return taken[i] || scores[i] > 0 ? hit_ids[i] : free_ids[i]; };
  {
    py::gil_scoped_release release;
    for (ssize_t i = 0; i < count; ++i) {
      const auto id = id_at(i);
      if (id >= slot.size()) throw std::invalid_argument("invalid voxel observation");
      if (id) ++lengths[slot[id]];
    }
  }
  py::dict groups;
  std::vector<int64_t*> dst(index.size());
  for (auto [label, k] : index) {
    if (!lengths[k]) continue;
    py::array_t<int64_t> indices(lengths[k]);
    dst[k] = indices.mutable_data();
    groups[py::int_(label)] = indices;
  }
  {
    py::gil_scoped_release release;
    for (ssize_t i = 0; i < count; ++i) {
      const auto id = id_at(i);
      if (id) { const auto k = slot[id]; dst[k][next[k]++] = i; }
    }
  }
  return groups;
}
void transfer(Columns &cols, int anchor, Pose old, Pose updated,
              std::array<std::array<double, 6>, 3> grids,
              py::array_t<int64_t, py::array::c_style> negative,
              py::tuple source, py::tuple target) {
  Grid src(grids[0]), dst(grids[1]), prior(grids[2]);
  auto obs = array<uint32_t>(source[0]);
  auto labels = array<int32_t>(source[1]);
  auto times = array<double>(source[2], labels.size());
  auto score = array<int16_t>(source[3], obs.size());
  auto occ = array<bool>(source[4], obs.size()),
       mixed = array<bool>(source[5], obs.size()),
       invalid = array<bool>(source[6], obs.size());
  auto cutoff = array<double>(source[7], prior.size());
  if (obs.size() % src.size())
    throw std::invalid_argument("incomplete height layer");
  auto out =
      array<uint32_t>(target[0], obs.size() / src.size() * dst.size(), true);
  auto out_score = array<int16_t>(target[1], out.size(), true);
  auto hit = array<bool>(target[2], out.size());
  auto out_mixed = array<bool>(target[3], out.size(), true);
  const auto* ids = obs.data();
  auto* written = out.mutable_data();
  auto* scores = out_score.mutable_data();
  auto* mix = out_mixed.mutable_data();
  const Transform forward(old, updated), inverse(updated, old);
  const auto* negative_data = negative.data();
  const auto* cutoff_data = cutoff.data();
  const auto* hit_data = hit.data();
  const auto* labels_data = labels.data();
  const auto* occ_data = occ.data();
  const auto* score_data = score.data();
  const auto* times_data = times.data();
  const auto* mixed_data = mixed.data();
  const auto* invalid_data = invalid.data();
  const auto obs_size = obs.size();
  const auto negative_size = negative.size();
  const auto labels_size = labels.size();
  const auto times_size = times.size();
  py::gil_scoped_release release;
  if (cols.source.size() != size_t(src.size()) || cols.target.size() != size_t(dst.size()))
    throw std::invalid_argument("column mask geometry mismatch");
  // One bit per height, in blocks for grids with more than 64 levels.
  ssize_t n = 0;
  while (n < negative_size) {
    const int64_t start = negative_data[n];
    if (start < 0 || start >= obs_size) throw std::invalid_argument("invalid voxel");
    const ssize_t first_level = (start / src.size() / 64) * 64;
    const ssize_t end = (first_level + 64) * src.size();
    for (; n < negative_size && negative_data[n] < end; ++n) {
      const auto v = negative_data[n];
      if (v < first_level * src.size() || v >= obs_size) throw std::invalid_argument("invalid voxel order");
      const int col = v % src.size();
      if (!cols.source[col]) cols.sources.push_back(col);
      cols.source[col] |= uint64_t(1) << (v / src.size() - first_level);
    }
    for (int col : cols.sources) {
      const Point p = src.centre(col);
      for (const auto &offset : src.offsets) {
        const int t = dst.cell(forward.apply(p, offset));
        if (t < 0) continue;
        if (!cols.target[t]) cols.targets.push_back(t);
        cols.target[t] |= cols.source[col];
      }
    }
    for (int t : cols.targets) {
      uint64_t levels = cols.target[t];
      cols.target[t] = 0;
      const Point p = dst.centre(t);
      std::array<int, 5> cells;
      std::array<double, 5> min_stamp;
      for (int i = 0; i < 5; ++i) {
        const Point q = inverse.apply(p, dst.offsets[i]);
        cells[i] = src.cell(q);
        levels &= cells[i] < 0 ? 0 : cols.source[cells[i]];
        if (!levels) break;
        const int k = prior.cell(q);
        min_stamp[i] = k < 0 ? -std::numeric_limits<double>::infinity() : cutoff_data[k];
      }
      // All inverse samples must contain free evidence on this same level.
      while (levels) {
        const ssize_t level = first_level + __builtin_ctzll(levels);
        levels &= levels - 1;
        const ssize_t d = level * dst.size() + t, base = level * src.size();
        if (hit_data[d] || scores[d] > 0) continue;
        uint32_t oldest = 0;
        ssize_t weakest = 0;
        int16_t max_score = std::numeric_limits<int16_t>::min();
        bool complete = true;
        for (int i = 0; i < 5; ++i) {
          const auto j = base + cells[i];
          const auto id = ids[j];
          if (!id || id >= labels_size || labels_data[id] != anchor || occ_data[j] ||
              score_data[j] > 0 || !(times_data[id] > min_stamp[i])) { complete = false; break; }
          if (!oldest || times_data[id] < times_data[oldest]) { oldest = id; weakest = j; }
          max_score = std::max(max_score, score_data[j]);
        }
        if (written[d] >= times_size) throw std::invalid_argument("invalid destination observation");
        if (complete && times_data[oldest] > times_data[written[d]]) {
          written[d] = oldest; scores[d] = max_score;
          mix[d] = mixed_data[weakest] && !invalid_data[weakest];
        }
      }
    }
    for (int col : cols.sources) cols.source[col] = 0;
    cols.sources.clear(); cols.targets.clear();
  }
}
}  // namespace
PYBIND11_MODULE(_pm01_evidence, m) {
  py::class_<Columns>(m, "Columns").def(py::init<size_t, size_t>());
  m.def("transfer_free", &transfer);
  m.def("trace_rays", &trace_rays);
  m.def("group_voxels", &group_voxels);
}
