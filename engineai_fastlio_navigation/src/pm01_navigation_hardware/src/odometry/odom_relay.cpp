#include <cmath>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Core>

class OdomRelay : public rclcpp::Node {
 public:
  OdomRelay() : Node("odom_relay") {
    frame_ = declare_parameter<std::string>("base_frame", "base_link");
    base_body_ = extrinsic("lidar") * extrinsic("imu");
    pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    sub_ = create_subscription<nav_msgs::msg::Odometry>("/Odometry", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { relay(*msg); });
  }
 private:
  tf2::Transform extrinsic(const std::string & prefix) {
    const auto xyz = declare_parameter<std::vector<double>>(prefix + "_xyz", std::vector<double>{0,0,0});
    const auto rpy = declare_parameter<std::vector<double>>(prefix + "_rpy", std::vector<double>{0,0,0});
    if (xyz.size()!=3 || rpy.size()!=3) throw std::invalid_argument("Extrinsics require three components");
    for (auto v : xyz) if (!std::isfinite(v)) throw std::invalid_argument("Invalid translation");
    for (auto v : rpy) if (!std::isfinite(v)) throw std::invalid_argument("Invalid rotation");
    tf2::Quaternion q; q.setRPY(rpy[0],rpy[1],rpy[2]);
    return tf2::Transform(q, tf2::Vector3(xyz[0],xyz[1],xyz[2]));
  }
  void relay(const nav_msgs::msg::Odometry & msg) {
    if (msg.header.frame_id != "camera_init" || msg.child_frame_id != "body") return;
    const auto & p = msg.pose.pose;
    tf2::Quaternion q(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w);
    if (!std::isfinite(q.length2()) || q.length2()<1e-12 ||
        !std::isfinite(p.position.x) || !std::isfinite(p.position.y) || !std::isfinite(p.position.z)) return;
    q.normalize();
    const tf2::Transform input(q, tf2::Vector3(p.position.x,p.position.y,p.position.z));
    const auto pose = input * base_body_.inverse();
    const rclcpp::Time stamp(msg.header.stamp);
    if (have_prev_ && stamp <= prev_stamp_) return;
    nav_msgs::msg::Odometry out;
    out.header = msg.header; out.header.frame_id = "odom"; out.child_frame_id = frame_;
    out.pose.pose.position.x = pose.getOrigin().x();
    out.pose.pose.position.y = pose.getOrigin().y();
    out.pose.pose.position.z = pose.getOrigin().z();
    out.pose.pose.orientation = tf2::toMsg(pose.getRotation());
    // Shift world-frame pose covariance from IMU origin to base origin.
    Eigen::Matrix<double,6,6> covariance, jacobian = Eigen::Matrix<double,6,6>::Identity();
    for (int i=0;i<6;++i) for(int j=0;j<6;++j) covariance(i,j)=msg.pose.covariance[i*6+j];
    const auto offset = pose.getOrigin() - input.getOrigin();
    Eigen::Matrix3d cross;
    cross << 0,-offset.z(),offset.y(), offset.z(),0,-offset.x(), -offset.y(),offset.x(),0;
    jacobian.block<3,3>(0,3) = -cross;
    covariance = (jacobian * covariance * jacobian.transpose()).eval();
    for (int i=0;i<6;++i) for(int j=0;j<6;++j) out.pose.covariance[i*6+j]=covariance(i,j);
    if (have_prev_) {
      const double dt = (stamp-prev_stamp_).seconds();
      if (dt <= 0.5) {
        const auto v = pose.getBasis().transpose() * ((pose.getOrigin()-prev_.getOrigin()) / dt);
        auto dq = prev_.getRotation().inverse() * pose.getRotation();
        if (dq.w()<0) dq = tf2::Quaternion(-dq.x(),-dq.y(),-dq.z(),-dq.w());
        const auto w = dq.getAxis() * (dq.getAngle()/dt);
        out.twist.twist.linear.x=v.x(); out.twist.twist.linear.y=v.y(); out.twist.twist.linear.z=v.z();
        out.twist.twist.angular.x=w.x(); out.twist.twist.angular.y=w.y(); out.twist.twist.angular.z=w.z();
      }
    }
    // Derivative uncertainty is not supplied by FAST-LIO.
    for (int i=0;i<6;++i) out.twist.covariance[i*6+i]=1.0;
    prev_=pose; prev_stamp_=stamp; have_prev_=true;
    geometry_msgs::msg::TransformStamped transform;
    transform.header=out.header; transform.child_frame_id=frame_; transform.transform=tf2::toMsg(pose);
    tf_->sendTransform(transform); pub_->publish(out);
  }
  std::string frame_;
  tf2::Transform base_body_, prev_;
  rclcpp::Time prev_stamp_{0,0,RCL_ROS_TIME};
  bool have_prev_{false};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<OdomRelay>()); rclcpp::shutdown();
}
