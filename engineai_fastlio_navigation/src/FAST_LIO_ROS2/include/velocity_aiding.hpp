#pragma once
#include "use-ikfom.hpp"

using LioFilter = esekfom::esekf<state_ikfom, 12, input_ikfom>;

// An independent velocity of the IMU origin, expressed in IMU axes. Commands
// and the existing LIO odometry are not valid measurements for this update.
inline bool fuse_body_velocity(LioFilter & filter, const Eigen::Vector3d & measurement,
                               const Eigen::Matrix3d & covariance, double & nis,
                               double max_nis = 25.0) {
  if (!measurement.allFinite() || !covariance.allFinite() ||
      !covariance.isApprox(covariance.transpose(), 1e-8) ||
      covariance.llt().info() != Eigen::Success) return false;
  const auto state = filter.get_x();
  const vect3 predicted = state.rot.conjugate()*state.vel;
  Eigen::Matrix<double, 3, 23> h = Eigen::Matrix<double, 3, 23>::Zero();
  h.block<3, 3>(0, 3) = MTK::hat(predicted);
  h.block<3, 3>(0, 12) = state.rot.toRotationMatrix().transpose();
  const auto innovation = measurement-predicted;
  const Eigen::Matrix3d s = h*filter.get_P()*h.transpose()+covariance;
  nis = innovation.dot(s.ldlt().solve(innovation));
  if (!std::isfinite(nis) || nis > max_nis)
    return false;
  // Treat attitude, gyro bias, extrinsics and gravity as nuisance states.
  // Encoder-derived velocity must not manufacture an absolute heading
  // constraint. Only position, velocity and accelerometer bias are active.
  Eigen::Matrix<double, 23, 3> gain = filter.get_P()*h.transpose()*s.inverse();
  gain.block<9, 3>(3, 0).setZero();
  gain.block<3, 3>(15, 0).setZero();
  gain.block<2, 3>(21, 0).setZero();
  Eigen::Matrix<double, 23, 1> correction = gain*innovation;
  auto updated_state = state;
  updated_state.boxplus(correction);
  const auto p = filter.get_P();
  const LioFilter::cov a = LioFilter::cov::Identity()-gain*h;
  LioFilter::cov posterior = a*p*a.transpose()+gain*covariance*gain.transpose();
  posterior = (.5*(posterior+posterior.transpose())).eval();
  // All corrected components are Euclidean, so no SO3/S2 reset is needed.
  filter.change_x(updated_state);
  filter.change_P(posterior);
  return true;
}

// Scalar flat-ground contact height. Other pose components are deliberately
// not corrected, including their covariance cross-couplings.
inline bool fuse_ground_height(LioFilter & filter, double height, double variance) {
  if (!std::isfinite(height) || !std::isfinite(variance) || variance < 1e-6 || variance > 1)
    return false;
  auto state = filter.get_x();
  const auto p = filter.get_P();
  const double innovation = height-state.pos.z();
  const double s = p(2, 2)+variance;
  if (innovation*innovation/s > 25.) return false;
  Eigen::Matrix<double, 23, 1> gain = Eigen::Matrix<double, 23, 1>::Zero();
  gain[2] = p(2, 2)/s;
  gain[14] = p(14, 2)/s;
  Eigen::Matrix<double, 23, 1> correction = gain*innovation;
  state.boxplus(correction);
  LioFilter::cov a = LioFilter::cov::Identity();
  a.col(2) -= gain;
  LioFilter::cov posterior = a*p*a.transpose()+variance*gain*gain.transpose();
  posterior = (.5*(posterior+posterior.transpose())).eval();
  filter.change_x(state);
  filter.change_P(posterior);
  return true;
}
