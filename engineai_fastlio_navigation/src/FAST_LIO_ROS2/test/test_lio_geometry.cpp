#include "lio_geometry.hpp"
#include <iostream>

int main() {
  auto require = [](bool ok, const char * message) {
    if (!ok) throw std::runtime_error(message);
  };
  Eigen::MatrixXd plane = Eigen::MatrixXd::Zero(100, 12);
  for (int i = 0; i < 100; ++i) {
    plane(i, 2) = 1;
    plane(i, 3) = (i%10)-4.5;
    plane(i, 4) = (i/10)-4.5;
    plane(i, 0) = 1e-5*((i*7)%11);
  }
  auto geometry = lio_geometry(plane, .001);
  require(geometry.rank == 3, "Single plane must have three supported pose directions");
  require(geometry.weakest_information < .001*geometry.strongest_information,
          "Weak planar translation must be distinguishable from supported directions");
  Eigen::MatrixXd full = Eigen::MatrixXd::Zero(60, 12);
  for (int i = 0; i < 60; ++i) full(i, i%6) = 1;
  geometry = lio_geometry(full, .001);
  require(geometry.rank == 6, "Independent pose constraints must remain full rank");
  require(geometry.weakest_information > 0, "Independent constraints need positive information");
  require(lio_geometry(Eigen::MatrixXd::Zero(0, 12), .001).rank == 0, "Empty data cannot be healthy");
  bool rejected = false;
  try { lio_geometry(full, 0); } catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "Invalid thresholds must fail");
  std::cout << "LIO geometry checks passed\n";
}
