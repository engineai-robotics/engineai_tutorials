// FAST-LIO publishes deskewed points in its IMU "body" frame at scan end.
// Express them at the physical lidar origin, preserving the measurement stamp.
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Transform.h>

class LioCloudAdapter : public rclcpp::Node {
 public:
  LioCloudAdapter() : Node("lio_cloud_adapter") {
    const auto input = declare_parameter<std::string>("input_topic", "/cloud_registered_body");
    const auto output = declare_parameter<std::string>("output_topic", "/pm01/deskewed_points");
    input_frame_ = declare_parameter<std::string>("input_frame", "body");
    output_frame_ = declare_parameter<std::string>("lidar_frame", "livox_frame");
    const auto xyz = declare_parameter<std::vector<double>>("imu_xyz", {0., 0., 0.});
    const auto rpy = declare_parameter<std::vector<double>>("imu_rpy", {0., 0., 0.});
    if (input.empty() || output.empty() || input == output || input_frame_.empty() ||
        output_frame_.empty() || xyz.size() != 3 || rpy.size() != 3) {
      throw std::invalid_argument("Invalid deskewed cloud topics, frames or IMU extrinsics");
    }
    for (const auto & values : {xyz, rpy}) {
      for (double v : values) {
        if (!std::isfinite(v)) throw std::invalid_argument("Nonfinite IMU extrinsics");
      }
    }
    tf2::Quaternion rotation;
    rotation.setRPY(rpy[0], rpy[1], rpy[2]);
    // imu_xyz/rpy is T_lidar_imu, also used by the physical TF and odom relay.
    lidar_from_imu_ = tf2::Transform(rotation, tf2::Vector3(xyz[0], xyz[1], xyz[2]));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output, rclcpp::SensorDataQoS());
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { receive(*msg); });
  }

 private:
  void receive(const sensor_msgs::msg::PointCloud2 & input) {
    if (input.width == 0 || input.height == 0) return;
    if (input.header.frame_id != input_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Expected FAST-LIO cloud frame '%s', received '%s'", input_frame_.c_str(),
        input.header.frame_id.c_str());
      return;
    }
    // PCL's FAST-LIO output is little-endian FLOAT32 XYZ. Validate layout before
    // using iterators; never relabel raw lidar or compensate with the latest TF.
    const auto packed_row = static_cast<uint64_t>(input.width) * input.point_step;
    const auto total = static_cast<uint64_t>(input.row_step) * input.height;
    if (input.is_bigendian || input.point_step < 12 || input.row_step != packed_row ||
        total != input.data.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                          "Unsupported FAST-LIO point cloud layout");
      return;
    }
    for (const auto * name : {"x", "y", "z"}) {
      bool valid = false;
      for (const auto & field : input.fields) {
        if (field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
            field.count == 1 && field.offset <= input.point_step - sizeof(float)) {
          valid = true;
        }
      }
      if (!valid) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Expected FLOAT32 XYZ in FAST-LIO cloud");
        return;
      }
    }
    auto output = input;
    output.header.frame_id = output_frame_;
    sensor_msgs::PointCloud2Iterator<float> x(output, "x"), y(output, "y"), z(output, "z");
    const auto count = static_cast<uint64_t>(output.width) * output.height;
    for (uint64_t i = 0; i < count; ++i, ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) continue;
      const auto point = lidar_from_imu_ * tf2::Vector3(*x, *y, *z);
      *x = static_cast<float>(point.x());
      *y = static_cast<float>(point.y());
      *z = static_cast<float>(point.z());
    }
    // No restamping: both cloud and odometry refer to lidar_end_stamp.
    pub_->publish(output);
  }

  std::string input_frame_, output_frame_;
  tf2::Transform lidar_from_imu_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioCloudAdapter>());
  rclcpp::shutdown();
}
