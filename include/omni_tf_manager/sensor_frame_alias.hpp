// Copyright 2026 Omni Robotics
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_TF_MANAGER__SENSOR_FRAME_ALIAS_HPP_
#define OMNI_TF_MANAGER__SENSOR_FRAME_ALIAS_HPP_

#include <cstdint>
#include <string>

namespace omni_tf_manager
{

// A frame alias is a metadata correction only. It never rotates or translates
// the sensor payload. The reviewer must verify that the source payload is
// already expressed in the physical frame represented by output_frame.
enum class SensorFrameAliasStatus : std::uint8_t
{
  kAccepted,
  kUnverified,
  kInputFrameMismatch,
};

template<typename MessageT>
SensorFrameAliasStatus applyIdentityFrameAlias(
  const MessageT & input, const std::string & expected_input_frame,
  const std::string & output_frame, bool alias_verified, MessageT & output)
{
  if (!alias_verified) {
    return SensorFrameAliasStatus::kUnverified;
  }
  if (input.header.frame_id != expected_input_frame) {
    return SensorFrameAliasStatus::kInputFrameMismatch;
  }
  output = input;
  output.header.frame_id = output_frame;
  return SensorFrameAliasStatus::kAccepted;
}

}  // namespace omni_tf_manager

#endif  // OMNI_TF_MANAGER__SENSOR_FRAME_ALIAS_HPP_
