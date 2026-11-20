#include "flat_ground.hpp"
#include "velocity_aiding.hpp"
#include <iostream>

static std::vector<Eigen::Vector3d> measured_floor;

int main() {
  auto require=[](bool ok,const char * message) { if (!ok) throw std::runtime_error(message); };
  std::vector<Eigen::Vector3d> floor;
  for (int x=-5;x<=5;++x) for (int y=-5;y<=5;++y) floor.emplace_back(x*.4,y*.4,-1.);
  const Eigen::Vector3d position(14.,-12.,1.);
  const Eigen::Matrix3d rotation=(Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(.025,Eigen::Vector3d::UnitY())*
      Eigen::AngleAxisd(-.02,Eigen::Vector3d::UnitX())).toRotationMatrix();
  auto selected=select_flat_ground(floor,rotation,position,0.);
  require(selected.inliers==121 && selected.points_imu.size()==16,"Tilted, translated floor must be selected");
  auto platform=floor;
  for (auto & p : platform) p.z()+=.25;
  require(select_flat_ground(platform,Eigen::Matrix3d::Identity(),position,0.).points_imu.empty(),
          "A raised platform must not anchor the floor");
  std::vector<Eigen::Vector3d> line;
  for (int i=0;i<60;++i) line.emplace_back(i*.05,0,-1.);
  require(select_flat_ground(line,Eigen::Matrix3d::Identity(),position,0.).points_imu.empty(),
          "Collinear returns cannot constrain both tilt axes");
  Eigen::MatrixXd h; Eigen::VectorXd residual;
  flat_ground_model(selected.points_imu,rotation,position,0.,h,residual);
  require(h.leftCols(2).norm()==0,"Flat ground must not constrain XY");
  require((h.middleCols(3,3)*(rotation.transpose()*Eigen::Vector3d::UnitZ())).norm()<1e-12,
          "World yaw must remain unobservable");
  for (int axis=0;axis<3;++axis) {
    const double epsilon=1e-6;
    Eigen::MatrixXd hp;Eigen::VectorXd rp;
    flat_ground_model(selected.points_imu,rotation*Eigen::AngleAxisd(epsilon,
        Eigen::Vector3d::Unit(axis)).toRotationMatrix(),position,0.,hp,rp);
    require(((rp-residual)/epsilon+h.col(3+axis)).norm()<1e-4,"Ground rotation Jacobian must match finite differences");
  }
  LioFilter filter;double limits[23];std::fill(limits,limits+23,.001);
  measured_floor=selected.points_imu;
  filter.init_dyn_share(get_f,df_dx,df_dw,[](state_ikfom & state,esekfom::dyn_share_datastruct<double> & data) {
    flat_ground_model(measured_floor,state.rot.toRotationMatrix(),state.pos,0.,data.h_x,data.h);
  },4,limits);
  auto state=filter.get_x();state.pos=position;state.rot=rotation;filter.change_x(state);
  double solve=0;filter.update_iterated_dyn_share_modified(.001,solve);
  const auto after=filter.get_x();
  require((after.rot*Eigen::Vector3d::UnitZ()-Eigen::Vector3d::UnitZ()).norm()<.001,
          "Ground update must correct tilt");
  require(std::hypot(after.pos.x()-position.x(),after.pos.y()-position.y())<1e-10,
          "Ground update must preserve uncorrelated XY");
  Eigen::SelfAdjointEigenSolver<LioFilter::cov> eigen(filter.get_P());
  require(eigen.eigenvalues()[0]>0,"Ground update covariance must stay positive");
  std::cout<<"Flat-ground selection, observability and filter checks passed\n";
}
