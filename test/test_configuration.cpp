// Copyright 2026 Omni Robotics
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "omni_tf_manager/configuration.hpp"

namespace omni_tf_manager
{
namespace
{

StaticTransformSpec makeTransform(
  const std::string & id,
  const std::string & role,
  const std::string & parent,
  const std::string & child,
  const Eigen::Vector3d & translation = Eigen::Vector3d::Zero())
{
  StaticTransformSpec result;
  result.id = id;
  result.role = role;
  result.parent_frame = parent;
  result.child_frame = child;
  result.transform.translation = translation;
  return result;
}

TEST(Configuration, AcceptsConnectedFourSensorTree)
{
  const std::vector<StaticTransformSpec> transforms{
    makeTransform("lidar", "lidar", "base", "lidar_frame", {0.1, 0.0, 0.2}),
    makeTransform("imu", "imu", "lidar_frame", "imu_frame"),
    makeTransform("depth", "depth_camera", "base", "depth_frame"),
    makeTransform("rgb", "rgb_camera", "base", "rgb_frame")};

  const auto result = validateStaticTree(
    transforms, "base", "lidar_frame",
    {"lidar", "imu", "depth_camera", "rgb_camera"});
  EXPECT_TRUE(result.valid) << result.error;

  const auto base_to_imu = resolveTransform(transforms, "base", "imu_frame");
  EXPECT_NEAR(base_to_imu.translation.x(), 0.1, 1.0e-12);
  EXPECT_NEAR(base_to_imu.translation.z(), 0.2, 1.0e-12);
}

TEST(Configuration, RejectsDuplicateChildAuthority)
{
  const std::vector<StaticTransformSpec> transforms{
    makeTransform("lidar_a", "lidar", "base", "lidar_frame"),
    makeTransform("lidar_b", "imu", "other", "lidar_frame")};
  const auto result = validateStaticTree(
    transforms, "base", "lidar_frame", {"lidar", "imu"});
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error.find("multiple static parents"), std::string::npos);
}

TEST(Configuration, RejectsDisconnectedSensor)
{
  const std::vector<StaticTransformSpec> transforms{
    makeTransform("lidar", "lidar", "base", "lidar_frame"),
    makeTransform("imu", "imu", "unknown_parent", "imu_frame")};
  const auto result = validateStaticTree(
    transforms, "base", "lidar_frame", {"lidar", "imu"});
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error.find("not connected"), std::string::npos);
}

TEST(Configuration, RejectsMissingRequiredRole)
{
  const std::vector<StaticTransformSpec> transforms{
    makeTransform("lidar", "lidar", "base", "lidar_frame")};
  const auto result = validateStaticTree(
    transforms, "base", "lidar_frame", {"lidar", "imu"});
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error.find("required sensor role"), std::string::npos);
}

TEST(Configuration, ValidatesTfFrameIds)
{
  EXPECT_TRUE(validFrameId("robot_1/lidar_link"));
  EXPECT_FALSE(validFrameId("/lidar_link"));
  EXPECT_FALSE(validFrameId("robot//lidar_link"));
  EXPECT_FALSE(validFrameId("lidar link"));
}

}  // namespace
}  // namespace omni_tf_manager
