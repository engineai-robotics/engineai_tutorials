#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class CloudConverter : public rclcpp::Node {
 public:
  CloudConverter() : Node("custommsg_to_cloud") {
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/livox/points", rclcpp::SensorDataQoS());
    sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/livox/lidar", rclcpp::SensorDataQoS(), [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {
        if (msg->header.frame_id != "livox_frame" || msg->point_num != msg->points.size()) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejecting unexpected frame or inconsistent point count");
          return;
        }
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = msg->header;
        cloud.is_dense = true;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        using F = sensor_msgs::msg::PointField;
        modifier.setPointCloud2Fields(5, "x", 1, F::FLOAT32, "y", 1, F::FLOAT32,
          "z", 1, F::FLOAT32, "intensity", 1, F::FLOAT32, "offset_time", 1, F::UINT32);
        modifier.resize(msg->points.size());
        sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"), y(cloud,"y"), z(cloud,"z"), intensity(cloud,"intensity");
        sensor_msgs::PointCloud2Iterator<uint32_t> time(cloud,"offset_time");
        for (const auto & p : msg->points) {
          *x = p.x; *y = p.y; *z = p.z; *intensity = p.reflectivity; *time = p.offset_time;
          cloud.is_dense = cloud.is_dense && std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
          ++x; ++y; ++z; ++intensity; ++time;
        }
        pub_->publish(cloud);
      });
  }
 private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<CloudConverter>()); rclcpp::shutdown();
}
