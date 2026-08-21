// Copyright 2026 Omni Robotics
// SPDX-License-Identifier: Apache-2.0

#include "omni_tf_manager/transform_math.hpp"

#include <algorithm>
#include <cmath>

namespace omni_tf_manager
{
namespace
{

Eigen::Matrix3d skew(const Eigen::Vector3d & vector)
{
  Eigen::Matrix3d matrix;
  matrix << 0.0, -vector.z(), vector.y(),
    vector.z(), 0.0, -vector.x(),
    -vector.y(), vector.x(), 0.0;
  return matrix;
}

}  // namespace

RigidTransform identityTransform()
{
  return RigidTransform{};
}

RigidTransform compose(
  const RigidTransform & parent_to_mid,
  const RigidTransform & mid_to_child)
{
  RigidTransform result;
  result.rotation = parent_to_mid.rotation * mid_to_child.rotation;
  result.rotation.normalize();
  result.translation =
    parent_to_mid.translation + parent_to_mid.rotation * mid_to_child.translation;
  return result;
}

RigidTransform inverse(const RigidTransform & transform)
{
  RigidTransform result;
  result.rotation = transform.rotation.conjugate();
  result.rotation.normalize();
  result.translation = -(result.rotation * transform.translation);
  return result;
}

RigidTransform fromPose(const geometry_msgs::msg::Pose & pose)
{
  RigidTransform result;
  result.translation = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  result.rotation = Eigen::Quaterniond(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  return result;
}

RigidTransform fromTransform(const geometry_msgs::msg::Transform & transform)
{
  RigidTransform result;
  result.translation = Eigen::Vector3d(
    transform.translation.x, transform.translation.y, transform.translation.z);
  result.rotation = Eigen::Quaterniond(
    transform.rotation.w, transform.rotation.x,
    transform.rotation.y, transform.rotation.z);
  return result;
}

geometry_msgs::msg::Pose toPose(const RigidTransform & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation.x();
  pose.position.y = transform.translation.y();
  pose.position.z = transform.translation.z();
  pose.orientation.x = transform.rotation.x();
  pose.orientation.y = transform.rotation.y();
  pose.orientation.z = transform.rotation.z();
  pose.orientation.w = transform.rotation.w();
  return pose;
}

geometry_msgs::msg::Transform toTransform(const RigidTransform & transform)
{
  geometry_msgs::msg::Transform message;
  message.translation.x = transform.translation.x();
  message.translation.y = transform.translation.y();
  message.translation.z = transform.translation.z();
  message.rotation.x = transform.rotation.x();
  message.rotation.y = transform.rotation.y();
  message.rotation.z = transform.rotation.z();
  message.rotation.w = transform.rotation.w();
  return message;
}

Eigen::Quaterniond quaternionFromRpy(double roll, double pitch, double yaw)
{
  const Eigen::AngleAxisd roll_rotation(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_rotation(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_rotation(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond quaternion = yaw_rotation * pitch_rotation * roll_rotation;
  quaternion.normalize();
  return quaternion;
}

double rotationDistance(const Eigen::Quaterniond & lhs, const Eigen::Quaterniond & rhs)
{
  Eigen::Quaterniond delta = lhs.conjugate() * rhs;
  delta.normalize();
  return Eigen::AngleAxisd(delta).angle();
}

bool isFinite(const RigidTransform & transform)
{
  return transform.translation.allFinite() && transform.rotation.coeffs().allFinite();
}

bool normalizeQuaternion(Eigen::Quaterniond & quaternion, double minimum_norm)
{
  const double squared_norm = quaternion.squaredNorm();
  if (!std::isfinite(squared_norm) || squared_norm < minimum_norm * minimum_norm) {
    return false;
  }
  quaternion.normalize();
  return quaternion.coeffs().allFinite();
}

Matrix6d covarianceToMatrix(const Covariance6d & covariance)
{
  Matrix6d result;
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      result(row, column) = covariance[static_cast<std::size_t>(row * 6 + column)];
    }
  }
  return result;
}

Covariance6d covarianceFromMatrix(const Matrix6d & covariance)
{
  Covariance6d result{};
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      result[static_cast<std::size_t>(row * 6 + column)] = covariance(row, column);
    }
  }
  return result;
}

Covariance6d transformPoseCovarianceSensorToBase(
  const Covariance6d & covariance,
  const Eigen::Matrix3d & base_to_world_rotation,
  const Eigen::Vector3d & body_to_sensor_translation)
{
  Matrix6d jacobian = Matrix6d::Identity();
  jacobian.block<3, 3>(0, 3) =
    base_to_world_rotation * skew(body_to_sensor_translation);
  return covarianceFromMatrix(jacobian * covarianceToMatrix(covariance) * jacobian.transpose());
}

Covariance6d transformTwistCovarianceSensorToBase(
  const Covariance6d & covariance,
  const RigidTransform & body_to_sensor)
{
  const Eigen::Matrix3d rotation = body_to_sensor.rotation.toRotationMatrix();
  Matrix6d jacobian = Matrix6d::Zero();
  jacobian.block<3, 3>(0, 0) = rotation;
  jacobian.block<3, 3>(0, 3) = skew(body_to_sensor.translation) * rotation;
  jacobian.block<3, 3>(3, 3) = rotation;
  return covarianceFromMatrix(jacobian * covarianceToMatrix(covariance) * jacobian.transpose());
}

Covariance6d rotateCovariance(
  const Covariance6d & covariance,
  const Eigen::Matrix3d & rotation)
{
  Matrix6d jacobian = Matrix6d::Zero();
  jacobian.block<3, 3>(0, 0) = rotation;
  jacobian.block<3, 3>(3, 3) = rotation;
  return covarianceFromMatrix(jacobian * covarianceToMatrix(covariance) * jacobian.transpose());
}

}  // namespace omni_tf_manager
