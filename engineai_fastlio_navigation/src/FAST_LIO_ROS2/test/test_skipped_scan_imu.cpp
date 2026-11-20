#include "../src/IMU_Processing.hpp"
#include <stdexcept>

void require(bool value, const char * text) {if (!value) throw std::runtime_error(text);}
using Filter = esekfom::esekf<state_ikfom,12,input_ikfom>;
MeasureGroup scan(double begin, double end, double imu_begin, double yaw_rate) {
    MeasureGroup measure;
    measure.lidar_beg_time=begin; measure.lidar_end_time=end;
    measure.lidar.reset(new PointCloudXYZI());
    for (int i=0; i<5; ++i) {
        PointType point{}; point.x=2.; point.curvature=i*25.;
        measure.lidar->push_back(point);
    }
    for (int i=1; i<=std::lround((end-imu_begin)/.005); ++i) {
        auto imu=std::make_shared<sensor_msgs::msg::Imu>();
        imu->header.stamp=rclcpp::Time(int64_t(std::llround((imu_begin+i*.005)*1e9)));
        imu->linear_acceleration.z=9.81; imu->angular_velocity.z=yaw_rate;
        measure.imu.push_back(imu);
    }
    return measure;
}
void initialize(ImuProcess & process, Filter & filter) {
    double limits[23]; std::fill(limits,limits+23,.001);
    filter.init_dyn_share(get_f,df_dx,df_dw,
        [](state_ikfom &, esekfom::dyn_share_datastruct<double> &) {},3,limits);
    process.set_extrinsic(V3D::Zero(),M3D::Identity());
    process.set_gyr_cov(V3D::Constant(.1)); process.set_acc_cov(V3D::Constant(.1));
    auto points=std::make_shared<PointCloudXYZI>();
    process.Process(scan(1.,1.1,1.,0.),filter,points);
}
int main() {
    ImuProcess process, reference; Filter filter, reference_filter;
    initialize(process,filter); initialize(reference,reference_filter);
    auto points=std::make_shared<PointCloudXYZI>();
    auto expected=std::make_shared<PointCloudXYZI>();
    // Retain every IMU sample while skipping four unprocessed lidar scans.
    process.Process(scan(1.5,1.6,1.1,1.),filter,points);
    for(int i=0;i<5;++i)
        reference.Process(scan(1.1+i*.1,1.2+i*.1,1.1+i*.1,1.),reference_filter,expected);
    require(filter.get_x().rot.toRotationMatrix().isApprox(reference_filter.get_x().rot.toRotationMatrix(),1e-9),
        "Skipped lidar changed IMU rotation integration");
    require((filter.get_x().pos-reference_filter.get_x().pos).norm()<1e-8,
        "Skipped lidar changed IMU position integration");
    require(points->size()==expected->size(),"Incomplete deskew output");
    for(size_t i=0;i<points->size();++i) {
        if (((*points)[i].getVector3fMap()-(*expected)[i].getVector3fMap()).norm()>=1e-5)
            std::cerr << "point " << i << ": " << (*points)[i].getVector3fMap().transpose()
                      << " vs " << (*expected)[i].getVector3fMap().transpose() << '\n';
        require(((*points)[i].getVector3fMap()-(*expected)[i].getVector3fMap()).norm()<1e-5,
            "Intervening IMU poses changed deskew of the retained scan");
    }
    // Do not integrate a remembered angular rate/velocity over an unobserved
    // ten-second interval. Preserve pose, biases, gravity and mounting instead.
    ImuProcess interrupted;Filter recovered;
    initialize(interrupted,recovered);
    auto prior=recovered.get_x();
    prior.pos=V3D(3.,-2.,.4);prior.vel=V3D(.6,0.,0.);
    recovered.change_x(prior);
    auto resumed=scan(11.,11.1,10.995,0.);
    interrupted.Resume(11.,resumed.imu.front(),recovered);
    require(recovered.get_x().pos.isApprox(prior.pos),"Recovery reset the odometry origin");
    require(recovered.get_x().bg.isApprox(prior.bg),"Recovery discarded gyro calibration");
    require(recovered.get_x().vel.isZero(),"Unobserved velocity was extrapolated across a gap");
    interrupted.Process(resumed,recovered,points);
    ImuProcess baseline;Filter baseline_filter;
    initialize(baseline,baseline_filter);
    prior.vel.setZero();baseline_filter.change_x(prior);
    baseline.Process(scan(1.1,1.2,1.1,0.),baseline_filter,expected);
    require((recovered.get_x().pos-baseline_filter.get_x().pos).norm()<1e-8,
        "Missing interval changed integration of the current complete scan");
    require(points->size()==5,"Recovered scan was not deskewed");
}
