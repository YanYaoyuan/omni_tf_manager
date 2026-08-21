// Copyright 2026 Omni Robotics
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_TF_MANAGER__CONFIGURATION_HPP_
#define OMNI_TF_MANAGER__CONFIGURATION_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "omni_tf_manager/transform_math.hpp"

namespace omni_tf_manager
{

struct StaticTransformSpec
{
  std::string id;
  std::string role;
  std::string parent_frame;
  std::string child_frame;
  RigidTransform transform;
};

struct TreeValidationResult
{
  bool valid{false};
  std::string error;
};

bool validFrameId(const std::string & frame_id);

TreeValidationResult validateStaticTree(
  const std::vector<StaticTransformSpec> & transforms,
  const std::string & base_frame,
  const std::string & tracking_frame,
  const std::vector<std::string> & required_roles);

RigidTransform resolveTransform(
  const std::vector<StaticTransformSpec> & transforms,
  const std::string & root_frame,
  const std::string & child_frame);

}  // namespace omni_tf_manager

#endif  // OMNI_TF_MANAGER__CONFIGURATION_HPP_
