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

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include <sensor_msgs/msg/point_field.hpp>

namespace
{

sensor_msgs::msg::PointCloud2 make_cloud(uint32_t point_step)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "livox_frame";
  cloud.header.stamp.sec = 12;
  cloud.header.stamp.nanosec = 34;
  cloud.height = 1;
  cloud.width = 2;
  cloud.is_bigendian = false;
  cloud.point_step = point_step;
  cloud.row_step = point_step * cloud.width;
  cloud.data.resize(cloud.row_step);
  cloud.fields = {
    sensor_msgs::msg::PointField().set__name("x").set__offset(0).set__datatype(
      sensor_msgs::msg::PointField::FLOAT32).set__count(1),
    sensor_msgs::msg::PointField().set__name("y").set__offset(4).set__datatype(
      sensor_msgs::msg::PointField::FLOAT32).set__count(1),
    sensor_msgs::msg::PointField().set__name("z").set__offset(8).set__datatype(
      sensor_msgs::msg::PointField::FLOAT32).set__count(1),
    sensor_msgs::msg::PointField().set__name(
      "intensity").set__offset(16).set__datatype(
      sensor_msgs::msg::PointField::FLOAT32).set__count(1),
  };
  const float values[2][4] = {
    {1.0F, 2.0F, 3.0F, 42.0F},
    {4.0F, 5.0F, 6.0F, 300.0F},
  };
  for (std::size_t index = 0; index < 2; ++index) {
    auto * point = cloud.data.data() + index * point_step;
    std::memcpy(point, &values[index][0], sizeof(float));
    std::memcpy(point + 4, &values[index][1], sizeof(float));
    std::memcpy(point + 8, &values[index][2], sizeof(float));
    std::memcpy(point + 16, &values[index][3], sizeof(float));
  }
  return cloud;
}

}  // namespace

TEST(LivoxPointcloudBridge, ConvertsPaddedPointLayouts)
{
  for (const uint32_t point_step : {24U, 32U}) {
    const auto cloud = make_cloud(point_step);
    livox_ros_driver2::msg::CustomMsg output;
    std::string error;
    ASSERT_TRUE(
      simulator::pointcloud_to_livox(cloud, 0.1, output, error)) << error;
    ASSERT_EQ(output.points.size(), 2U);
    EXPECT_EQ(output.header.frame_id, "livox_frame");
    EXPECT_EQ(output.timebase, 12000000034ULL);
    EXPECT_FLOAT_EQ(output.points[0].x, 1.0F);
    EXPECT_EQ(output.points[0].reflectivity, 42U);
    EXPECT_EQ(output.points[0].offset_time, 0U);
    EXPECT_EQ(output.points[1].reflectivity, 255U);
    EXPECT_EQ(output.points[1].offset_time, 100000000U);
  }
}

TEST(LivoxPointcloudBridge, FiltersNonFinitePoints)
{
  auto cloud = make_cloud(24);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  std::memcpy(cloud.data.data() + cloud.point_step, &nan, sizeof(float));
  livox_ros_driver2::msg::CustomMsg output;
  std::string error;
  ASSERT_TRUE(
    simulator::pointcloud_to_livox(cloud, 0.1, output, error)) << error;
  ASSERT_EQ(output.point_num, 1U);
  EXPECT_FLOAT_EQ(output.points[0].x, 1.0F);
}

TEST(LivoxPointcloudBridge, AcceptsEmptyCloud)
{
  auto cloud = make_cloud(24);
  cloud.width = 0;
  cloud.row_step = 0;
  cloud.data.clear();
  livox_ros_driver2::msg::CustomMsg output;
  std::string error;
  ASSERT_TRUE(
    simulator::pointcloud_to_livox(cloud, 0.1, output, error)) << error;
  EXPECT_TRUE(output.points.empty());
}

TEST(LivoxPointcloudBridge, RejectsMissingCoordinates)
{
  auto cloud = make_cloud(24);
  cloud.fields.pop_back();
  cloud.fields.pop_back();
  livox_ros_driver2::msg::CustomMsg output;
  std::string error;
  EXPECT_FALSE(
    simulator::pointcloud_to_livox(cloud, 0.1, output, error));
  EXPECT_FALSE(error.empty());
}
