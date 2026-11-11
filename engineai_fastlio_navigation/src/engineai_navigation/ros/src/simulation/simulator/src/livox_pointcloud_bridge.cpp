// Copyright 2026 EngineAI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "simulator/livox_pointcloud_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace simulator
{
namespace
{

const sensor_msgs::msg::PointField * find_field(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

template<typename ValueT>
ValueT read_bytes(const uint8_t * data, bool big_endian)
{
  ValueT value{};
  if (!big_endian) {
    std::memcpy(&value, data, sizeof(ValueT));
  } else {
    uint8_t reversed[sizeof(ValueT)];
    for (std::size_t index = 0; index < sizeof(ValueT); ++index) {
      reversed[index] = data[sizeof(ValueT) - 1U - index];
    }
    std::memcpy(&value, reversed, sizeof(ValueT));
  }
  return value;
}

bool read_numeric(
  const uint8_t * point, const sensor_msgs::msg::PointField & field,
  bool big_endian, double & value)
{
  const uint8_t * data = point + field.offset;
  using sensor_msgs::msg::PointField;
  switch (field.datatype) {
    case PointField::INT8:
      value = static_cast<double>(read_bytes<int8_t>(data, false));
      return true;
    case PointField::UINT8:
      value = static_cast<double>(read_bytes<uint8_t>(data, false));
      return true;
    case PointField::INT16:
      value = static_cast<double>(read_bytes<int16_t>(data, big_endian));
      return true;
    case PointField::UINT16:
      value = static_cast<double>(read_bytes<uint16_t>(data, big_endian));
      return true;
    case PointField::INT32:
      value = static_cast<double>(read_bytes<int32_t>(data, big_endian));
      return true;
    case PointField::UINT32:
      value = static_cast<double>(read_bytes<uint32_t>(data, big_endian));
      return true;
    case PointField::FLOAT32:
      value = static_cast<double>(read_bytes<float>(data, big_endian));
      return true;
    case PointField::FLOAT64:
      value = read_bytes<double>(data, big_endian);
      return true;
    default:
      return false;
  }
}

bool field_fits(
  const sensor_msgs::msg::PointField & field, uint32_t point_step)
{
  using sensor_msgs::msg::PointField;
  std::size_t size = 0;
  switch (field.datatype) {
    case PointField::INT8:
    case PointField::UINT8:
      size = 1;
      break;
    case PointField::INT16:
    case PointField::UINT16:
      size = 2;
      break;
    case PointField::INT32:
    case PointField::UINT32:
    case PointField::FLOAT32:
      size = 4;
      break;
    case PointField::FLOAT64:
      size = 8;
      break;
    default:
      return false;
  }
  return field.count > 0 && field.offset + size <= point_step;
}

}  // namespace

bool pointcloud_to_livox(
  const sensor_msgs::msg::PointCloud2 & cloud, double scan_period,
  livox_ros_driver2::msg::CustomMsg & output, std::string & error)
{
  output = livox_ros_driver2::msg::CustomMsg();
  output.header = cloud.header;
  output.timebase =
    static_cast<uint64_t>(cloud.header.stamp.sec) * 1000000000ULL +
    cloud.header.stamp.nanosec;
  output.lidar_id = 0;

  if (!std::isfinite(scan_period) || scan_period <= 0.0) {
    error = "scan_period must be finite and positive";
    return false;
  }
  const auto * x_field = find_field(cloud, "x");
  const auto * y_field = find_field(cloud, "y");
  const auto * z_field = find_field(cloud, "z");
  const auto * intensity_field = find_field(cloud, "intensity");
  if (!x_field || !y_field || !z_field) {
    error = "PointCloud2 must contain x, y and z fields";
    return false;
  }
  if (
    !field_fits(*x_field, cloud.point_step) ||
    !field_fits(*y_field, cloud.point_step) ||
    !field_fits(*z_field, cloud.point_step) ||
    (intensity_field && !field_fits(*intensity_field, cloud.point_step)))
  {
    error = "PointCloud2 field exceeds point_step or has invalid type";
    return false;
  }
  const uint64_t total_points =
    static_cast<uint64_t>(cloud.width) * cloud.height;
  if (
    cloud.point_step == 0 ||
    cloud.row_step < cloud.width * cloud.point_step ||
    cloud.data.size() < static_cast<std::size_t>(cloud.row_step) * cloud.height)
  {
    error = "PointCloud2 data dimensions are inconsistent";
    return false;
  }

  output.points.reserve(static_cast<std::size_t>(total_points));
  const double scan_nanoseconds = std::min(
    scan_period * 1.0e9,
    static_cast<double>(std::numeric_limits<uint32_t>::max()));
  uint64_t raw_index = 0;
  for (uint32_t row = 0; row < cloud.height; ++row) {
    for (uint32_t column = 0; column < cloud.width; ++column, ++raw_index) {
      const auto offset =
        static_cast<std::size_t>(row) * cloud.row_step +
        static_cast<std::size_t>(column) * cloud.point_step;
      const uint8_t * point = cloud.data.data() + offset;
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      if (
        !read_numeric(point, *x_field, cloud.is_bigendian, x) ||
        !read_numeric(point, *y_field, cloud.is_bigendian, y) ||
        !read_numeric(point, *z_field, cloud.is_bigendian, z) ||
        !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      {
        continue;
      }
      livox_ros_driver2::msg::CustomPoint converted;
      converted.x = static_cast<float>(x);
      converted.y = static_cast<float>(y);
      converted.z = static_cast<float>(z);
      double intensity = 0.0;
      if (intensity_field) {
        read_numeric(
          point, *intensity_field, cloud.is_bigendian, intensity);
      }
      if (!std::isfinite(intensity)) {
        intensity = 0.0;
      }
      converted.reflectivity = static_cast<uint8_t>(
        std::clamp(std::lround(intensity), 0L, 255L));
      converted.tag = 0;
      converted.line = static_cast<uint8_t>(row % 256U);
      const double ratio = total_points > 1 ?
        static_cast<double>(raw_index) /
        static_cast<double>(total_points - 1U) : 0.0;
      converted.offset_time =
        static_cast<uint32_t>(std::llround(scan_nanoseconds * ratio));
      output.points.push_back(converted);
    }
  }
  output.point_num = static_cast<uint32_t>(output.points.size());
  error.clear();
  return true;
}

class LivoxPointcloudBridge : public rclcpp::Node
{
public:
  LivoxPointcloudBridge()
  : Node("livox_pointcloud_bridge")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/livox/lidar/pointcloud");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/livox/lidar");
    scan_period_ = declare_parameter<double>("scan_period", 0.1);
    publisher_ =
      create_publisher<livox_ros_driver2::msg::CustomMsg>(
      output_topic, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
        livox_ros_driver2::msg::CustomMsg output;
        std::string error;
        if (!pointcloud_to_livox(*cloud, scan_period_, output, error)) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000, "%s", error.c_str());
          return;
        }
        publisher_->publish(std::move(output));
      });
  }

private:
  double scan_period_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace simulator

#ifndef SIMULATOR_LIVOX_BRIDGE_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simulator::LivoxPointcloudBridge>());
  rclcpp::shutdown();
  return 0;
}
#endif
