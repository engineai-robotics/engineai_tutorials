#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

// Pose order is world translation, then right-local rotation. Normalize the
// rotational columns by their RMS lever arm before comparing information.
struct LioGeometry {
  int rank = 0;
  double weakest_information = 0.0;
  double strongest_information = 0.0;
  double rotation_scale = 1.0;
};

inline LioGeometry lio_geometry(const Eigen::MatrixXd & h, double threshold) {
  if (!std::isfinite(threshold) || threshold <= 0 || threshold >= 1)
    throw std::invalid_argument("Geometry threshold must lie between zero and one");
  LioGeometry result;
  if (h.rows() < 6 || h.cols() < 6 || !h.allFinite()) return result;
  const double translation = h.leftCols(3).squaredNorm();
  const double rotation = h.middleCols(3, 3).squaredNorm();
  result.rotation_scale = std::sqrt(std::max(1.0, rotation / std::max(translation, 1e-12)));
  Eigen::MatrixXd normalized = h.leftCols(6);
  normalized.rightCols(3) /= result.rotation_scale;
  Eigen::Matrix<double, 6, 6> information = normalized.transpose()*normalized/h.rows();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(information);
  if (solver.info() != Eigen::Success) return result;
  result.weakest_information = solver.eigenvalues()[0];
  result.strongest_information = solver.eigenvalues()[5];
  const double cutoff = threshold * std::max(result.strongest_information, 1e-12);
  for (int i = 0; i < 6; ++i) {
    if (solver.eigenvalues()[i] < cutoff) continue;
    ++result.rank;
  }
  return result;
}

// Reject implausible scan-matching corrections relative to the propagated
// state, restoring covariance as well as the mean. This is an optional local
// correction bound, not global relocalization or a replacement for loop closure.
template <typename Filter, typename Update>
bool guarded_lidar_update(Filter & filter, Update update, double max_translation,
                          double max_rotation, double & translation, double & rotation) {
  auto predicted = filter.get_x();
  auto covariance = filter.get_P();
  update();
  const auto & posterior = filter.get_x();
  translation = (posterior.pos-predicted.pos).norm();
  const Eigen::Matrix3d delta = predicted.rot.toRotationMatrix().transpose()*
      posterior.rot.toRotationMatrix();
  rotation = delta.allFinite() ? Eigen::AngleAxisd(delta).angle() : NAN;
  const bool accepted = std::isfinite(translation) && std::isfinite(rotation) &&
      filter.get_P().allFinite() && posterior.vel.allFinite() &&
      (max_translation <= 0 || translation <= max_translation) &&
      (max_rotation <= 0 || rotation <= max_rotation);
  if (!accepted) {
    filter.change_x(predicted);
    filter.change_P(covariance);
  }
  return accepted;
}
