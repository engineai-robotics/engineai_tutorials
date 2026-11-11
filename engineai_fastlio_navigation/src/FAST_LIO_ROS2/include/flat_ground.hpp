#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

// Optional flat-floor prior. Inputs are measured IMU-frame endpoints; the
// horizontal support plane is an explicit environment assumption, not a pose
// measurement. It cannot constrain horizontal position or world yaw.
struct FlatGroundObservation {
  std::vector<Eigen::Vector3d> points_imu;
  int inliers = 0;
  double rms = 0;
  double predicted_tilt = 0;
};

inline FlatGroundObservation select_flat_ground(
    const std::vector<Eigen::Vector3d> & points_imu, const Eigen::Matrix3d & rotation,
    const Eigen::Vector3d & position, double ground_z) {
  FlatGroundObservation result;
  std::vector<Eigen::Vector3d> world;
  std::vector<size_t> ids;
  for (size_t i = 0; i < points_imu.size(); ++i) {
    const auto & point = points_imu[i];
    if (!point.allFinite() || point.norm() < .7 || point.norm() > 8.) continue;
    const Eigen::Vector3d p = rotation*point+position;
    if (std::abs(p.z()-ground_z) > .35) continue;
    ids.push_back(i); world.push_back(p);
  }
  if (world.size() < 30) return result;
  std::vector<size_t> best;
  uint32_t seed = 168;
  auto sample = [&]() { seed = seed*1664525u+1013904223u; return seed%world.size(); };
  const double minimum_normal_z = std::cos(12.*M_PI/180.);
  for (int trial = 0; trial < 100; ++trial) {
    const size_t a=sample(), b=sample(), c=sample();
    Eigen::Vector3d normal = (world[b]-world[a]).cross(world[c]-world[a]);
    if (normal.norm() < 1e-6) continue;
    normal.normalize(); if (normal.z() < 0) normal = -normal;
    if (normal.z() < minimum_normal_z) continue;
    const double offset = -normal.dot(world[a]);
    const double at_origin = -(offset+normal.x()*position.x()+normal.y()*position.y())/normal.z();
    if (std::abs(at_origin-ground_z) > .12) continue;
    std::vector<size_t> inliers;
    for (size_t i = 0; i < world.size(); ++i)
      if (std::abs(normal.dot(world[i])+offset) < .035) inliers.push_back(i);
    if (inliers.size() > best.size()) best = std::move(inliers);
  }
  if (best.size() < 30 || best.size() < .55*world.size()) return result;
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (auto i : best) mean += world[i];
  mean /= best.size();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (auto i : best) {
    const Eigen::Vector3d d = world[i]-mean; covariance += d*d.transpose();
  }
  covariance /= best.size();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success || solver.eigenvalues()[1] < .08 ||
      solver.eigenvalues()[2] < .5) return result;
  Eigen::Vector3d normal = solver.eigenvectors().col(0);
  if (normal.z() < 0) normal = -normal;
  const double at_origin = mean.z()-(normal.x()*(position.x()-mean.x())+
      normal.y()*(position.y()-mean.y()))/normal.z();
  if (normal.z() < minimum_normal_z || std::abs(at_origin-ground_z) > .12) return result;
  result.rms = std::sqrt(std::max(0.,solver.eigenvalues()[0]));
  if (result.rms > .025) return FlatGroundObservation();
  result.inliers = best.size();
  result.predicted_tilt = std::acos(std::clamp(normal.z(),-1.,1.));
  // At most 16 widely spaced constraints limit correlated point evidence.
  std::vector<double> distance(best.size(),1e30);
  size_t selected = 0;
  for (size_t count = 0; count < std::min<size_t>(16,best.size()); ++count) {
    result.points_imu.push_back(points_imu[ids[best[selected]]]);
    for (size_t j = 0; j < best.size(); ++j)
      distance[j] = std::min(distance[j],(world[best[j]]-world[best[selected]]).squaredNorm());
    selected = std::max_element(distance.begin(),distance.end())-distance.begin();
  }
  return result;
}

inline void flat_ground_model(const std::vector<Eigen::Vector3d> & points_imu,
    const Eigen::Matrix3d & rotation, const Eigen::Vector3d & position,
    double ground_z, Eigen::MatrixXd & jacobian, Eigen::VectorXd & residual) {
  jacobian = Eigen::MatrixXd::Zero(points_imu.size(),12);
  residual.resize(points_imu.size());
  const Eigen::Vector3d normal_imu = rotation.transpose()*Eigen::Vector3d::UnitZ();
  for (size_t i = 0; i < points_imu.size(); ++i) {
    jacobian(i,2) = 1.;
    jacobian.block<1,3>(i,3) = points_imu[i].cross(normal_imu).transpose();
    residual[i] = ground_z-(rotation*points_imu[i]+position).z();
  }
}
