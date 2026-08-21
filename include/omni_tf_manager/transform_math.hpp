// Copyright 2026 Omni Robotics
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_TF_MANAGER__TRANSFORM_MATH_HPP_
#define OMNI_TF_MANAGER__TRANSFORM_MATH_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>

namespace omni_tf_manager
{

using Covariance6d = std::array<double, 36>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

struct RigidTransform
{
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
};

RigidTransform identityTransform();
RigidTransform compose(const RigidTransform & parent_to_mid, const RigidTransform & mid_to_child);
RigidTransform inverse(const RigidTransform & transform);
RigidTransform fromPose(const geometry_msgs::msg::Pose & pose);
RigidTransform fromTransform(const geometry_msgs::msg::Transform & transform);
geometry_msgs::msg::Pose toPose(const RigidTransform & transform);
geometry_msgs::msg::Transform toTransform(const RigidTransform & transform);
Eigen::Quaterniond quaternionFromRpy(double roll, double pitch, double yaw);
double rotationDistance(const Eigen::Quaterniond & lhs, const Eigen::Quaterniond & rhs);
bool isFinite(const RigidTransform & transform);
bool normalizeQuaternion(Eigen::Quaterniond & quaternion, double minimum_norm = 1.0e-12);

Matrix6d covarianceToMatrix(const Covariance6d & covariance);
Covariance6d covarianceFromMatrix(const Matrix6d & covariance);
Covariance6d transformPoseCovarianceSensorToBase(
  const Covariance6d & covariance,
  const Eigen::Matrix3d & base_to_world_rotation,
  const Eigen::Vector3d & body_to_sensor_translation);
Covariance6d transformTwistCovarianceSensorToBase(
  const Covariance6d & covariance,
  const RigidTransform & body_to_sensor);
Covariance6d rotateCovariance(const Covariance6d & covariance, const Eigen::Matrix3d & rotation);

}  // namespace omni_tf_manager

#endif  // OMNI_TF_MANAGER__TRANSFORM_MATH_HPP_
