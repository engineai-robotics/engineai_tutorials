#include <cmath>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// Fast-LIO publishes camera_init -> body, and body is the lidar IMU.
// Gazebo's robot_state_publisher roots the articulated body at base_footprint,
// so RViz cannot draw the links in camera_init until this bridge attaches that
// root. The waist joint sits between the root and the IMU, so the offset is
// read from TF instead of a fixed extrinsic.
class SimBodyTf : public rclcpp::Node {
 public:
  SimBodyTf() : Node("sim_body_tf") {
    input_frame_ = declare_parameter<std::string>("input_frame", "camera_init");
    output_frame_ = declare_parameter<std::string>("output_frame", "camera_init");
    body_frame_ = declare_parameter<std::string>("body_frame", "body");
    root_frame_ = declare_parameter<std::string>("root_frame", "base_footprint");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");
    publish_odom_ = declare_parameter<bool>("publish_odom", false);
    const auto topic = declare_parameter<std::string>("odom_topic", "/Odometry");
    const auto output_topic = declare_parameter<std::string>("output_odom_topic", "/odom");
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    if (publish_odom_) {
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic, 10);
    }
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      topic, rclcpp::QoS(rclcpp::KeepLast(20)),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { relay(*msg); });
  }

 private:
  void relay(const nav_msgs::msg::Odometry & msg) {
    if (msg.header.frame_id != input_frame_ || msg.child_frame_id != body_frame_) {
      return;
    }
    const auto & pose = msg.pose.pose;
    tf2::Quaternion rotation(
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    if (!std::isfinite(rotation.length2()) || rotation.length2() < 1e-12 ||
      !std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z))
    {
      return;
    }
    rotation.normalize();
    const tf2::Transform world_body(
      rotation, tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));

    geometry_msgs::msg::TransformStamped root_imu;
    try {
      root_imu = buffer_->lookupTransform(
        root_frame_, imu_frame_, msg.header.stamp, rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for same-time %s -> %s: %s",
        root_frame_.c_str(), imu_frame_.c_str(), error.what());
      return;  // Never attach a current joint pose to an older LIO observation.
    }
    tf2::Transform root_to_imu;
    tf2::fromMsg(root_imu.transform, root_to_imu);
    const tf2::Transform world_root = world_body * root_to_imu.inverse();

    geometry_msgs::msg::TransformStamped out;
    out.header.stamp = msg.header.stamp;
    out.header.frame_id = output_frame_;
    out.child_frame_id = root_frame_;
    out.transform = tf2::toMsg(world_root);
    broadcaster_->sendTransform(out);
    if (publish_odom_) {
      publish_odometry(msg.header.stamp, world_root);
    }
  }

  void publish_odometry(const builtin_interfaces::msg::Time & stamp, const tf2::Transform & pose) {
    const rclcpp::Time time(stamp);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = output_frame_;
    odom.child_frame_id = root_frame_;
    odom.pose.pose.position.x = pose.getOrigin().x();
    odom.pose.pose.position.y = pose.getOrigin().y();
    odom.pose.pose.position.z = pose.getOrigin().z();
    odom.pose.pose.orientation = tf2::toMsg(pose.getRotation());
    if (have_prev_ && time > prev_stamp_) {
      const double dt = (time - prev_stamp_).seconds();
      if (dt > 0.0 && dt <= 0.5) {
        const auto linear = pose.getBasis().transpose() * ((pose.getOrigin() - prev_.getOrigin()) / dt);
        auto delta = prev_.getRotation().inverse() * pose.getRotation();
        if (delta.w() < 0.0) {
          delta = tf2::Quaternion(-delta.x(), -delta.y(), -delta.z(), -delta.w());
        }
        const auto angular = delta.getAxis() * (delta.getAngle() / dt);
        odom.twist.twist.linear.x = linear.x();
        odom.twist.twist.linear.y = linear.y();
        odom.twist.twist.linear.z = linear.z();
        odom.twist.twist.angular.x = angular.x();
        odom.twist.twist.angular.y = angular.y();
        odom.twist.twist.angular.z = angular.z();
      }
    }
    for (int i = 0; i < 6; ++i) {
      odom.pose.covariance[i * 6 + i] = 0.01;
      odom.twist.covariance[i * 6 + i] = 0.01;
    }
    prev_ = pose;
    prev_stamp_ = time;
    have_prev_ = true;
    odom_pub_->publish(odom);
  }

  std::string input_frame_;
  std::string output_frame_;
  std::string body_frame_;
  std::string root_frame_;
  std::string imu_frame_;
  bool publish_odom_{false};
  bool have_prev_{false};
  tf2::Transform prev_;
  rclcpp::Time prev_stamp_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimBodyTf>());
  rclcpp::shutdown();
}
