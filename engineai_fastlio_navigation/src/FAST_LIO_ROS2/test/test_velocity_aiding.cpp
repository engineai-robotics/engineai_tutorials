#include "velocity_aiding.hpp"
#include "lio_geometry.hpp"
#include <iostream>

int main() {
  auto require = [](bool ok, const char * message) {
    if (!ok) throw std::runtime_error(message);
  };
  LioFilter filter;
  double limit[23];
  std::fill(limit, limit+23, .001);
  filter.init_dyn_share(get_f, df_dx, df_dw,
      [](state_ikfom &, esekfom::dyn_share_datastruct<double> &) {}, 3, limit);
  auto state = filter.get_x();
  state.rot = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  filter.change_x(state);
  double nis = 0;
  require(fuse_body_velocity(filter, Eigen::Vector3d(0, -1, 0), .04*Eigen::Matrix3d::Identity(), nis),
          "Valid body velocity must update the filter");
  require(filter.get_x().vel.x() > .9 && std::abs(filter.get_x().vel.y()) < .01,
          "Body velocity must use the current attitude to constrain world velocity");
  const auto p = filter.get_P();
  require(p.allFinite() && p.isApprox(p.transpose(), 1e-8), "Updated covariance must be finite and symmetric");
  Eigen::SelfAdjointEigenSolver<LioFilter::cov> solver(p);
  require(solver.eigenvalues()[0] > 0, "Updated covariance must stay positive definite");
  auto before = filter.get_x();
  require(!fuse_body_velocity(filter, Eigen::Vector3d(100, 0, 0), .01*Eigen::Matrix3d::Identity(), nis),
          "Gross innovation must be rejected");
  require((filter.get_x().vel-before.vel).norm() < 1e-12, "Rejected observations must not change state");
  const auto orientation = filter.get_x().rot.toRotationMatrix();
  require(fuse_body_velocity(filter, Eigen::Vector3d(.1, -.8, 0), .04*Eigen::Matrix3d::Identity(), nis),
          "A second velocity observation must be accepted");
  require((filter.get_x().rot.toRotationMatrix()-orientation).norm() < 1e-12,
          "Schmidt velocity aiding must not invent a heading correction");
  require(!fuse_body_velocity(filter, Eigen::Vector3d::Zero(), -Eigen::Matrix3d::Identity(), nis),
          "Invalid covariance must be rejected");
  before = filter.get_x();
  require(fuse_ground_height(filter, .1, .001), "Finite flat-ground height must be accepted");
  require(filter.get_x().pos.z() > .09, "Height must constrain vertical position");
  require(std::abs(filter.get_x().pos.x()-before.pos.x()) < 1e-12 &&
          std::abs(filter.get_x().pos.y()-before.pos.y()) < 1e-12,
          "Contact height must not constrain horizontal position");
  require(!fuse_ground_height(filter, 100, .001), "Inconsistent height must be rejected");
  const auto saved = filter.get_x();
  const auto saved_covariance = filter.get_P();
  double translation = 0, rotation = 0;
  require(!guarded_lidar_update(filter, [&]() {
      auto bad = filter.get_x(); bad.pos.x() += 1.; bad.vel.y() += 2.;
      auto bad_covariance = (0.1*filter.get_P()).eval();
      filter.change_x(bad); filter.change_P(bad_covariance);
    }, .25, .15, translation, rotation), "A large scan correction must be rejected");
  require((filter.get_x().pos-saved.pos).norm() < 1e-12 &&
          (filter.get_x().vel-saved.vel).norm() < 1e-12 &&
          filter.get_P().isApprox(saved_covariance, 1e-12),
          "Rejection must restore both the state and covariance");
  require(guarded_lidar_update(filter, [&]() {
      auto good = filter.get_x(); good.pos.x() += .05; filter.change_x(good);
    }, .25, .15, translation, rotation), "A small scan correction must be accepted");
  const auto accepted = filter.get_x();
  require(!guarded_lidar_update(filter, [&]() {
      auto bad = filter.get_x();
      bad.rot = accepted.rot.toRotationMatrix()*Eigen::AngleAxisd(.2,Eigen::Vector3d::UnitZ()).toRotationMatrix();
      filter.change_x(bad);
    }, .25, .15, translation, rotation), "Excess scan rotation must be rejected");
  require((filter.get_x().rot.toRotationMatrix()-accepted.rot.toRotationMatrix()).norm() < 1e-12,
          "Rejected attitude must be restored");
  require(!guarded_lidar_update(filter, [&]() {
      auto bad = filter.get_x(); bad.pos.x() = NAN; filter.change_x(bad);
    }, .25, .15, translation, rotation), "Non-finite scan updates must be rejected");
  std::cout << "Body velocity fusion checks passed\n";
}
