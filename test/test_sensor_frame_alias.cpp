// Copyright 2026 Omni Robotics
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "omni_tf_manager/sensor_frame_alias.hpp"

namespace omni_tf_manager
{
namespace
{

TEST(SensorFrameAlias, RewritesPointCloudHeaderWithoutChangingPayload)
{
  sensor_msgs::msg::PointCloud2 input;
  input.header.frame_id = "livox_frame";
  input.header.stamp.sec = 42;
  input.height = 1U;
  input.width = 2U;
  input.data = {1U, 2U, 3U, 4U};
  sensor_msgs::msg::PointCloud2 output;

  const auto status = applyIdentityFrameAlias(
    input, "livox_frame", "omni_lidar_link", true, output);

  EXPECT_EQ(status, SensorFrameAliasStatus::kAccepted);
  EXPECT_EQ(output.header.frame_id, "omni_lidar_link");
  EXPECT_EQ(output.header.stamp, input.header.stamp);
  EXPECT_EQ(output.height, input.height);
  EXPECT_EQ(output.width, input.width);
  EXPECT_EQ(output.data, input.data);
  EXPECT_EQ(input.header.frame_id, "livox_frame");
}

TEST(SensorFrameAlias, RewritesImuHeaderWithoutChangingMeasurement)
{
  sensor_msgs::msg::Imu input;
  input.header.frame_id = "livox_frame";
  input.orientation.w = 1.0;
  input.angular_velocity.z = 0.25;
  input.linear_acceleration.x = 9.8;
  input.angular_velocity_covariance[0] = 0.01;
  sensor_msgs::msg::Imu output;

  const auto status = applyIdentityFrameAlias(
    input, "livox_frame", "omni_imu_link", true, output);

  EXPECT_EQ(status, SensorFrameAliasStatus::kAccepted);
  EXPECT_EQ(output.header.frame_id, "omni_imu_link");
  EXPECT_DOUBLE_EQ(output.orientation.w, input.orientation.w);
  EXPECT_DOUBLE_EQ(output.angular_velocity.z, input.angular_velocity.z);
  EXPECT_DOUBLE_EQ(output.linear_acceleration.x, input.linear_acceleration.x);
  EXPECT_EQ(output.angular_velocity_covariance, input.angular_velocity_covariance);
}

TEST(SensorFrameAlias, RejectsUnexpectedInputFrame)
{
  sensor_msgs::msg::PointCloud2 input;
  input.header.frame_id = "unexpected_frame";
  sensor_msgs::msg::PointCloud2 output;
  output.header.frame_id = "unchanged";

  const auto status = applyIdentityFrameAlias(
    input, "livox_frame", "omni_lidar_link", true, output);

  EXPECT_EQ(status, SensorFrameAliasStatus::kInputFrameMismatch);
  EXPECT_EQ(output.header.frame_id, "unchanged");
}

TEST(SensorFrameAlias, BlocksUnreviewedAlias)
{
  sensor_msgs::msg::Imu input;
  input.header.frame_id = "lidar_imu";
  sensor_msgs::msg::Imu output;
  output.header.frame_id = "unchanged";

  const auto status = applyIdentityFrameAlias(
    input, "lidar_imu", "omni_imu_link", false, output);

  EXPECT_EQ(status, SensorFrameAliasStatus::kUnverified);
  EXPECT_EQ(output.header.frame_id, "unchanged");
}

}  // namespace
}  // namespace omni_tf_manager
