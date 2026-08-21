// Copyright 2026 Omni Robotics
// SPDX-License-Identifier: Apache-2.0

#include "omni_tf_manager/configuration.hpp"

#include <cctype>
#include <stdexcept>
#include <unordered_set>

namespace omni_tf_manager
{
namespace
{

bool validIdentifier(const std::string & identifier)
{
  if (identifier.empty()) {
    return false;
  }
  for (const char character : identifier) {
    const bool valid =
      (character >= 'a' && character <= 'z') ||
      (character >= 'A' && character <= 'Z') ||
      (character >= '0' && character <= '9') || character == '_';
    if (!valid) {
      return false;
    }
  }
  return true;
}

}  // namespace

bool validFrameId(const std::string & frame_id)
{
  if (frame_id.empty() || frame_id.front() == '/' || frame_id.back() == '/' ||
    frame_id.find("//") != std::string::npos)
  {
    return false;
  }
  for (const unsigned char character : frame_id) {
    if (std::isspace(character) || std::iscntrl(character)) {
      return false;
    }
  }
  return true;
}

TreeValidationResult validateStaticTree(
  const std::vector<StaticTransformSpec> & transforms,
  const std::string & base_frame,
  const std::string & tracking_frame,
  const std::vector<std::string> & required_roles)
{
  if (!validFrameId(base_frame)) {
    return {false, "invalid base frame: " + base_frame};
  }
  if (!validFrameId(tracking_frame)) {
    return {false, "invalid tracking frame: " + tracking_frame};
  }

  std::unordered_map<std::string, const StaticTransformSpec *> by_child;
  std::unordered_set<std::string> ids;
  std::unordered_set<std::string> roles;
  for (const auto & transform : transforms) {
    if (!validIdentifier(transform.id)) {
      return {false, "invalid static transform id: " + transform.id};
    }
    if (!ids.insert(transform.id).second) {
      return {false, "duplicate static transform id: " + transform.id};
    }
    if (!validFrameId(transform.parent_frame) || !validFrameId(transform.child_frame)) {
      return {false, "invalid parent or child frame in transform: " + transform.id};
    }
    if (transform.parent_frame == transform.child_frame) {
      return {false, "self-referencing transform: " + transform.id};
    }
    if (transform.child_frame == base_frame) {
      return {false, "base frame must not be a static transform child"};
    }
    if (!isFinite(transform.transform)) {
      return {false, "non-finite transform: " + transform.id};
    }
    if (!by_child.emplace(transform.child_frame, &transform).second) {
      return {false, "multiple static parents for frame: " + transform.child_frame};
    }
    if (!transform.role.empty()) {
      roles.insert(transform.role);
    }
  }

  for (const auto & required_role : required_roles) {
    if (roles.count(required_role) == 0U) {
      return {false, "required sensor role is missing: " + required_role};
    }
  }

  if (tracking_frame != base_frame && by_child.count(tracking_frame) == 0U) {
    return {false, "tracking frame is not in the static tree: " + tracking_frame};
  }

  for (const auto & transform : transforms) {
    std::unordered_set<std::string> visited;
    std::string frame = transform.child_frame;
    while (frame != base_frame) {
      if (!visited.insert(frame).second) {
        return {false, "cycle detected at frame: " + frame};
      }
      const auto iterator = by_child.find(frame);
      if (iterator == by_child.end()) {
        return {false, "frame is not connected to base '" + base_frame + "': " + frame};
      }
      frame = iterator->second->parent_frame;
    }
  }

  return {true, {}};
}

RigidTransform resolveTransform(
  const std::vector<StaticTransformSpec> & transforms,
  const std::string & root_frame,
  const std::string & child_frame)
{
  if (root_frame == child_frame) {
    return identityTransform();
  }

  std::unordered_map<std::string, const StaticTransformSpec *> by_child;
  for (const auto & transform : transforms) {
    by_child.emplace(transform.child_frame, &transform);
  }

  std::vector<const StaticTransformSpec *> chain;
  std::unordered_set<std::string> visited;
  std::string frame = child_frame;
  while (frame != root_frame) {
    if (!visited.insert(frame).second) {
      throw std::invalid_argument("cycle while resolving frame: " + frame);
    }
    const auto iterator = by_child.find(frame);
    if (iterator == by_child.end()) {
      throw std::invalid_argument(
              "cannot resolve transform from '" + root_frame + "' to '" + child_frame + "'");
    }
    chain.push_back(iterator->second);
    frame = iterator->second->parent_frame;
  }

  RigidTransform result = identityTransform();
  for (auto iterator = chain.rbegin(); iterator != chain.rend(); ++iterator) {
    result = compose(result, (*iterator)->transform);
  }
  return result;
}

}  // namespace omni_tf_manager
