// Copyright 2026 Omni Robotics
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>

#include "omni_tf_manager/transform_math.hpp"

namespace omni_tf_manager
{
namespace
{

constexpr double kTolerance = 1.0e-9;

TEST(TransformMath, ComposeWithInverseReturnsIdentity)
{
  RigidTransform transform;
  transform.translation = Eigen::Vector3d(1.2, -0.4, 0.7);
  transform.rotation = quaternionFromRpy(0.2, -0.1, 0.8);

  const auto result = compose(transform, inverse(transform));
  EXPECT_NEAR(result.translation.norm(), 0.0, kTolerance);
  EXPECT_NEAR(rotationDistance(result.rotation, Eigen::Quaterniond::Identity()), 0.0, kTolerance);
}

TEST(TransformMath, ConvertsTrackingPoseToBodyPose)
{
  RigidTransform body_to_sensor;
  body_to_sensor.translation = Eigen::Vector3d(0.13, -0.02, 0.18);
  body_to_sensor.rotation = Eigen::Quaterniond::Identity();

  RigidTransform odom_to_body;
  odom_to_body.translation = Eigen::Vector3d(4.0, 2.0, 0.4);
  odom_to_body.rotation = quaternionFromRpy(0.0, 0.0, 0.7);
  const auto odom_to_sensor = compose(odom_to_body, body_to_sensor);

  const auto recovered_body = compose(odom_to_sensor, inverse(body_to_sensor));
  EXPECT_NEAR((recovered_body.translation - odom_to_body.translation).norm(), 0.0, kTolerance);
  EXPECT_NEAR(
    rotationDistance(recovered_body.rotation, odom_to_body.rotation), 0.0, kTolerance);
}

TEST(TransformMath, ConvertsIcpSensorPoseToLocalizationOrigin)
{
  RigidTransform map_to_tracking;
  map_to_tracking.translation = Eigen::Vector3d(8.0, -1.5, 0.7);
  map_to_tracking.rotation = quaternionFromRpy(0.2, -0.1, 1.1);

  RigidTransform tracking_to_lidar;
  tracking_to_lidar.translation = Eigen::Vector3d(0.13, -0.02, 0.18);
  tracking_to_lidar.rotation = quaternionFromRpy(-0.03, 0.04, 0.01);
  const auto map_to_lidar = compose(map_to_tracking, tracking_to_lidar);

  const auto recovered_map_to_odom = compose(
    map_to_lidar, inverse(tracking_to_lidar));
  EXPECT_NEAR(
    (recovered_map_to_odom.translation - map_to_tracking.translation).norm(),
    0.0, kTolerance);
  EXPECT_NEAR(
    rotationDistance(recovered_map_to_odom.rotation, map_to_tracking.rotation),
    0.0, kTolerance);
}

TEST(TransformMath, RotatesCovarianceIntoTargetWorldFrame)
{
  Covariance6d covariance{};
  covariance[0] = 4.0;
  covariance[7] = 1.0;
  covariance[14] = 2.0;
  covariance[21] = 0.4;
  covariance[28] = 0.1;
  covariance[35] = 0.2;
  const auto rotation = quaternionFromRpy(0.0, 0.0, M_PI_2).toRotationMatrix();

  const auto result = rotateCovariance(covariance, rotation);
  EXPECT_NEAR(result[0], 1.0, kTolerance);
  EXPECT_NEAR(result[7], 4.0, kTolerance);
  EXPECT_NEAR(result[21], 0.1, kTolerance);
  EXPECT_NEAR(result[28], 0.4, kTolerance);
}

}  // namespace
}  // namespace omni_tf_manager
