// Copyright 2026 Omni Robotics
// SPDX-License-Identifier: Apache-2.0

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <omni_slam_interfaces/msg/slam_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

#include "omni_tf_manager/configuration.hpp"
#include "omni_tf_manager/transform_math.hpp"

namespace omni_tf_manager
{
namespace
{

constexpr std::uint8_t kDiagnosticOk = 0U;
constexpr std::uint8_t kDiagnosticWarn = 1U;
constexpr std::uint8_t kDiagnosticError = 2U;

void requireFiniteNonnegative(double value, const std::string & parameter_name)
{
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(parameter_name + " must be finite and non-negative");
  }
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

std::string boolString(bool value)
{
  return value ? "true" : "false";
}

std::string doubleString(double value)
{
  std::ostringstream stream;
  stream.precision(6);
  stream << std::fixed << value;
  return stream.str();
}

bool validMode(const std::string & mode)
{
  return mode == "shadow" || mode == "authority";
}

enum class SlamMode : std::uint8_t
{
  kStopped = omni_slam_interfaces::msg::SlamStatus::MODE_STOPPED,
  kMapping = omni_slam_interfaces::msg::SlamStatus::MODE_MAPPING,
  kLocalization = omni_slam_interfaces::msg::SlamStatus::MODE_LOCALIZATION,
};

bool validSlamMode(std::uint8_t mode)
{
  return mode <= omni_slam_interfaces::msg::SlamStatus::MODE_LOCALIZATION;
}

SlamMode parseSlamMode(const std::string & mode)
{
  if (mode == "stopped") {
    return SlamMode::kStopped;
  }
  if (mode == "mapping") {
    return SlamMode::kMapping;
  }
  if (mode == "localization") {
    return SlamMode::kLocalization;
  }
  throw std::invalid_argument(
          "slam_state.fallback_mode must be stopped, mapping or localization");
}

std::string slamModeName(SlamMode mode)
{
  switch (mode) {
    case SlamMode::kMapping:
      return "mapping";
    case SlamMode::kLocalization:
      return "localization";
    case SlamMode::kStopped:
    default:
      return "stopped";
  }
}

}  // namespace

class OmniTfManagerNode : public rclcpp::Node
{
public:
  explicit OmniTfManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("omni_tf_manager", options)
  {
    loadConfiguration();

    const auto tree_result = validateStaticTree(
      static_transforms_, base_frame_, tracking_frame_, required_sensor_roles_);
    if (!tree_result.valid) {
      throw std::invalid_argument("invalid static TF profile: " + tree_result.error);
    }
    body_to_tracking_ = resolveTransform(static_transforms_, base_frame_, tracking_frame_);
    tracking_to_icp_sensor_ = resolveTransform(
      static_transforms_, tracking_frame_, icp_sensor_frame_);

    if (authorityMode()) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    }

    local_body_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      local_body_odom_topic_, rclcpp::SensorDataQoS());
    global_body_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      global_body_odom_topic_, rclcpp::SensorDataQoS());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10).reliable());
    ready_pub_ = create_publisher<std_msgs::msg::Bool>(
      ready_topic_, rclcpp::QoS(1).reliable().transient_local());

    configureSensorRelays();

    sensor_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      sensor_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&OmniTfManagerNode::sensorOdomCallback, this, std::placeholders::_1));

    icp_pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      icp_pose_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&OmniTfManagerNode::icpPoseCallback, this, std::placeholders::_1));
    slam_status_sub_ = create_subscription<omni_slam_interfaces::msg::SlamStatus>(
      slam_status_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&OmniTfManagerNode::slamStatusCallback, this, std::placeholders::_1));

    if (!require_slam_status_) {
      transitionToSlamMode(fallback_slam_mode_, 0U, "configured fallback");
    }

    if (authorityMode()) {
      publishSensorStaticTransforms();
    }

    const auto period = std::chrono::duration<double>(diagnostics_period_s_);
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&OmniTfManagerNode::diagnosticsTimerCallback, this));

    logOwnershipSummary();
    publishReadiness(false);
  }

private:
  void loadConfiguration()
  {
    mode_ = declare_parameter<std::string>("mode", "shadow");
    profile_name_ = declare_parameter<std::string>("profile_name", "unconfigured");
    profile_version_ = declare_parameter<std::string>("profile_version", "0.0.0");
    calibration_id_ = declare_parameter<std::string>("calibration_id", "unverified");
    require_omni_prefix_ = declare_parameter<bool>("naming.require_omni_prefix", true);
    if (!validMode(mode_)) {
      throw std::invalid_argument("mode must be 'shadow' or 'authority'");
    }
    if (profile_name_.empty() || profile_version_.empty() || calibration_id_.empty()) {
      throw std::invalid_argument("profile identity parameters must not be empty");
    }

    map_frame_ = declare_parameter<std::string>("frames.map", "omni_map");
    odom_frame_ = declare_parameter<std::string>("frames.odom", "omni_odom");
    base_frame_ = declare_parameter<std::string>("frames.base", "omni_base_link");
    tracking_frame_ = declare_parameter<std::string>("frames.tracking", "omni_imu_link");
    icp_sensor_frame_ = declare_parameter<std::string>(
      "frames.icp_sensor", "omni_lidar_link");
    if (!validFrameId(map_frame_) || !validFrameId(odom_frame_) ||
      !validFrameId(base_frame_) || !validFrameId(tracking_frame_) ||
      !validFrameId(icp_sensor_frame_))
    {
      throw std::invalid_argument(
              "core frame names must be valid relative TF frame IDs without whitespace");
    }
    if (require_omni_prefix_ &&
      (map_frame_.rfind("omni_", 0U) != 0U || odom_frame_.rfind("omni_", 0U) != 0U ||
      base_frame_.rfind("omni_", 0U) != 0U || tracking_frame_.rfind("omni_", 0U) != 0U ||
      icp_sensor_frame_.rfind("omni_", 0U) != 0U))
    {
      throw std::invalid_argument(
              "core frame names must begin with 'omni_' when naming.require_omni_prefix=true");
    }
    if (map_frame_ == odom_frame_ || odom_frame_ == base_frame_ || map_frame_ == base_frame_) {
      throw std::invalid_argument("map, odom and base frame roles must be distinct");
    }
    if (tracking_frame_ != base_frame_ &&
      (tracking_frame_ == map_frame_ || tracking_frame_ == odom_frame_))
    {
      throw std::invalid_argument("tracking frame must not alias map or odom");
    }

    sensor_odom_topic_ = declare_parameter<std::string>(
      "topics.sensor_odom", "/state_estimation");
    icp_pose_topic_ = declare_parameter<std::string>("topics.icp_pose", "/icp_result");
    slam_status_topic_ = declare_parameter<std::string>(
      "topics.slam_status", "/omni/slam/status");
    local_body_odom_topic_ = declare_parameter<std::string>(
      "topics.local_body_odom", "/omni/tf_manager/body_odom");
    global_body_odom_topic_ = declare_parameter<std::string>(
      "topics.global_body_odom", "/omni/tf_manager/body_odom_global");
    diagnostics_topic_ = declare_parameter<std::string>("topics.diagnostics", "/diagnostics");
    ready_topic_ = declare_parameter<std::string>("topics.ready", "/omni/tf_manager/ready");
    if (sensor_odom_topic_.empty() || icp_pose_topic_.empty() || slam_status_topic_.empty() ||
      local_body_odom_topic_.empty() || global_body_odom_topic_.empty() ||
      diagnostics_topic_.empty() || ready_topic_.empty())
    {
      throw std::invalid_argument("topic names must not be empty");
    }
    if (sensor_odom_topic_ == local_body_odom_topic_ ||
      sensor_odom_topic_ == global_body_odom_topic_ ||
      local_body_odom_topic_ == global_body_odom_topic_)
    {
      throw std::invalid_argument("sensor, local-body and global-body topics must be distinct");
    }

    publish_local_body_odom_ = declare_parameter<bool>("publish.local_body_odom", true);
    publish_global_body_odom_ = declare_parameter<bool>("publish.global_body_odom", true);
    add_map_covariance_ = declare_parameter<bool>("publish.add_map_covariance", false);

    require_slam_status_ = declare_parameter<bool>("slam_state.required", true);
    fallback_slam_mode_ = parseSlamMode(
      declare_parameter<std::string>("slam_state.fallback_mode", "stopped"));
    slam_status_timeout_s_ = declare_parameter<double>("timeouts.slam_status_s", 2.0);
    allow_map_reinitialization_ = declare_parameter<bool>(
      "map_to_odom.allow_reinitialization", false);
    map_translation_jump_limit_m_ = declare_parameter<double>(
      "map_to_odom.translation_jump_limit_m", 2.0);
    map_rotation_jump_limit_rad_ = declare_parameter<double>(
      "map_to_odom.rotation_jump_limit_rad", 0.7853981633974483);

    require_nonzero_stamp_ = declare_parameter<bool>("validation.require_nonzero_stamp", true);
    require_monotonic_stamp_ = declare_parameter<bool>(
      "validation.require_monotonic_stamp", true);
    max_input_age_s_ = declare_parameter<double>("validation.max_input_age_s", 1.0);
    max_icp_age_s_ = declare_parameter<double>("validation.max_icp_age_s", 30.0);
    max_future_offset_s_ = declare_parameter<double>(
      "validation.max_future_offset_s", 0.1);
    quaternion_norm_tolerance_ = declare_parameter<double>(
      "validation.quaternion_norm_tolerance", 0.05);
    max_linear_speed_mps_ = declare_parameter<double>(
      "validation.max_linear_speed_mps", 5.0);
    max_angular_speed_radps_ = declare_parameter<double>(
      "validation.max_angular_speed_radps", 8.0);
    position_jump_margin_m_ = declare_parameter<double>(
      "validation.position_jump_margin_m", 0.5);
    rotation_jump_margin_rad_ = declare_parameter<double>(
      "validation.rotation_jump_margin_rad", 0.35);
    sensor_timeout_s_ = declare_parameter<double>("timeouts.sensor_odom_s", 0.5);
    diagnostics_period_s_ = declare_parameter<double>("diagnostics.period_s", 0.5);
    requireFiniteNonnegative(
      map_translation_jump_limit_m_, "map_to_odom.translation_jump_limit_m");
    requireFiniteNonnegative(
      map_rotation_jump_limit_rad_, "map_to_odom.rotation_jump_limit_rad");
    requireFiniteNonnegative(slam_status_timeout_s_, "timeouts.slam_status_s");
    requireFiniteNonnegative(max_input_age_s_, "validation.max_input_age_s");
    requireFiniteNonnegative(max_icp_age_s_, "validation.max_icp_age_s");
    requireFiniteNonnegative(max_future_offset_s_, "validation.max_future_offset_s");
    requireFiniteNonnegative(max_linear_speed_mps_, "validation.max_linear_speed_mps");
    requireFiniteNonnegative(max_angular_speed_radps_, "validation.max_angular_speed_radps");
    requireFiniteNonnegative(position_jump_margin_m_, "validation.position_jump_margin_m");
    requireFiniteNonnegative(rotation_jump_margin_rad_, "validation.rotation_jump_margin_rad");
    if (!std::isfinite(quaternion_norm_tolerance_) || quaternion_norm_tolerance_ < 0.0 ||
      quaternion_norm_tolerance_ >= 0.5)
    {
      throw std::invalid_argument(
              "validation.quaternion_norm_tolerance must be in [0.0, 0.5)");
    }
    if (!std::isfinite(diagnostics_period_s_) || !std::isfinite(sensor_timeout_s_) ||
      diagnostics_period_s_ <= 0.0 || sensor_timeout_s_ <= 0.0 ||
      (require_slam_status_ && slam_status_timeout_s_ <= 0.0))
    {
      throw std::invalid_argument("diagnostics and timeout periods must be positive");
    }

    required_sensor_roles_ = declare_parameter<std::vector<std::string>>(
      "required_sensor_roles", {"lidar", "imu", "depth_camera", "rgb_camera"});
    const auto transform_ids = declare_parameter<std::vector<std::string>>(
      "static_transforms", std::vector<std::string>{});
    if (transform_ids.empty()) {
      throw std::invalid_argument("static_transforms must contain the sensor tree");
    }

    static_transforms_.reserve(transform_ids.size());
    for (const auto & id : transform_ids) {
      const std::string prefix = "static_transform." + id + ".";
      StaticTransformSpec spec;
      spec.id = id;
      spec.role = declare_parameter<std::string>(prefix + "role", "");
      spec.parent_frame = declare_parameter<std::string>(prefix + "parent_frame", "");
      spec.child_frame = declare_parameter<std::string>(prefix + "child_frame", "");
      if (require_omni_prefix_ &&
        (spec.parent_frame.rfind("omni_", 0U) != 0U ||
        spec.child_frame.rfind("omni_", 0U) != 0U))
      {
        throw std::invalid_argument(
                "static TF frames must begin with 'omni_': " + spec.parent_frame + " -> " +
                spec.child_frame);
      }
      if (spec.child_frame == map_frame_ || spec.child_frame == odom_frame_) {
        throw std::invalid_argument(
                "static transform child must not claim map/odom frame: " + spec.child_frame);
      }
      const auto translation = declare_parameter<std::vector<double>>(
        prefix + "translation", {0.0, 0.0, 0.0});
      const auto rotation_rpy = declare_parameter<std::vector<double>>(
        prefix + "rotation_rpy", {0.0, 0.0, 0.0});
      requireVectorSize(translation, 3U, prefix + "translation");
      requireVectorSize(rotation_rpy, 3U, prefix + "rotation_rpy");
      spec.transform.translation = Eigen::Vector3d(
        translation[0], translation[1], translation[2]);
      spec.transform.rotation = quaternionFromRpy(
        rotation_rpy[0], rotation_rpy[1], rotation_rpy[2]);
      static_transforms_.push_back(std::move(spec));
    }
  }

  template<typename MessageT>
  void addHeaderRelay(
    const std::string & id, const std::string & input_topic,
    const std::string & output_topic, const std::string & expected_input_frame,
    const std::string & output_frame)
  {
    auto publisher = create_publisher<MessageT>(output_topic, rclcpp::SensorDataQoS());
    auto subscription = create_subscription<MessageT>(
      input_topic, rclcpp::SensorDataQoS(),
      [this, id, publisher, expected_input_frame, output_frame](
        const typename MessageT::ConstSharedPtr message)
      {
        if (!message) {
          return;
        }
        if (!expected_input_frame.empty() &&
          message->header.frame_id != expected_input_frame)
        {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Sensor relay '%s' rejected frame_id '%s'; expected '%s'",
            id.c_str(), message->header.frame_id.c_str(), expected_input_frame.c_str());
          return;
        }
        auto output = *message;
        output.header.frame_id = output_frame;
        publisher->publish(output);
      });
    sensor_relay_publishers_.push_back(publisher);
    sensor_relay_subscriptions_.push_back(subscription);
  }

  void configureSensorRelays()
  {
    const auto relay_ids = declare_parameter<std::vector<std::string>>(
      "sensor_relays", std::vector<std::string>{});
    std::set<std::string> input_topics;
    std::set<std::string> output_topics;
    for (const auto & id : relay_ids) {
      const std::string prefix = "sensor_relay." + id + ".";
      const auto type = declare_parameter<std::string>(prefix + "type", "");
      const auto input_topic = declare_parameter<std::string>(prefix + "input_topic", "");
      const auto output_topic = declare_parameter<std::string>(prefix + "output_topic", "");
      const auto input_frame = declare_parameter<std::string>(prefix + "input_frame", "");
      const auto output_frame = declare_parameter<std::string>(prefix + "output_frame", "");
      if (id.empty() || type.empty() || input_topic.empty() || output_topic.empty() ||
        output_frame.empty())
      {
        throw std::invalid_argument("sensor relay fields must not be empty: " + id);
      }
      if (input_topic == output_topic || !input_topics.insert(input_topic).second ||
        !output_topics.insert(output_topic).second)
      {
        throw std::invalid_argument("sensor relay topics must be unique: " + id);
      }
      if (!validFrameId(output_frame) ||
        (require_omni_prefix_ && output_frame.rfind("omni_", 0U) != 0U))
      {
        throw std::invalid_argument("invalid sensor relay output frame: " + output_frame);
      }
      // Require the rewritten frame to exist in the reviewed static tree.
      (void)resolveTransform(static_transforms_, base_frame_, output_frame);

      if (type == "pointcloud2") {
        addHeaderRelay<sensor_msgs::msg::PointCloud2>(
          id, input_topic, output_topic, input_frame, output_frame);
      } else if (type == "imu") {
        addHeaderRelay<sensor_msgs::msg::Imu>(
          id, input_topic, output_topic, input_frame, output_frame);
      } else if (type == "image") {
        addHeaderRelay<sensor_msgs::msg::Image>(
          id, input_topic, output_topic, input_frame, output_frame);
      } else if (type == "compressed_image") {
        addHeaderRelay<sensor_msgs::msg::CompressedImage>(
          id, input_topic, output_topic, input_frame, output_frame);
      } else if (type == "camera_info") {
        addHeaderRelay<sensor_msgs::msg::CameraInfo>(
          id, input_topic, output_topic, input_frame, output_frame);
      } else {
        throw std::invalid_argument("unsupported sensor relay type '" + type + "': " + id);
      }
      RCLCPP_INFO(
        get_logger(), "Sensor header relay [%s/%s]: %s (%s) -> %s (%s)",
        id.c_str(), type.c_str(), input_topic.c_str(), input_frame.c_str(),
        output_topic.c_str(), output_frame.c_str());
    }
    sensor_relay_count_ = relay_ids.size();
  }

  static void requireVectorSize(
    const std::vector<double> & values,
    std::size_t expected_size,
    const std::string & parameter_name)
  {
    if (values.size() != expected_size) {
      throw std::invalid_argument(
              parameter_name + " must contain exactly " +
              std::to_string(expected_size) + " values");
    }
    if (!std::all_of(
        values.begin(), values.end(), [](double value) {
          return std::isfinite(value);
        }))
    {
      throw std::invalid_argument(parameter_name + " contains a non-finite value");
    }
  }

  bool authorityMode() const
  {
    return mode_ == "authority";
  }

  bool slamStatusFresh() const
  {
    if (!require_slam_status_) {
      return true;
    }
    if (!received_slam_status_) {
      return false;
    }
    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_slam_status_receive_time_).count();
    return age <= slam_status_timeout_s_;
  }

  SlamMode effectiveSlamMode() const
  {
    return slamStatusFresh() ? slam_mode_ : SlamMode::kStopped;
  }

  void transitionToSlamMode(
    SlamMode next_mode, std::uint64_t status_sequence, const std::string & source)
  {
    const SlamMode previous_mode = slam_mode_;
    const bool mode_changed = !slam_mode_initialized_ || previous_mode != next_mode;
    slam_mode_ = next_mode;
    slam_mode_initialized_ = true;
    slam_status_sequence_ = status_sequence;
    if (!mode_changed) {
      return;
    }

    received_sensor_odom_ = false;
    have_previous_body_pose_ = false;
    map_initialized_ = false;
    map_to_odom_covariance_.fill(0.0);
    last_error_.clear();
    if (next_mode == SlamMode::kMapping) {
      map_to_odom_ = identityTransform();
      map_initialized_ = true;
    }
    publishReadiness(false);
    const std::string sequence_text = std::to_string(status_sequence);
    RCLCPP_INFO(
      get_logger(), "SLAM mode transition: %s -> %s (%s, sequence=%s)",
      slamModeName(previous_mode).c_str(), slamModeName(next_mode).c_str(), source.c_str(),
      sequence_text.c_str());
  }

  void slamStatusCallback(
    const omni_slam_interfaces::msg::SlamStatus::ConstSharedPtr message)
  {
    if (!message) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!validSlamMode(message->mode)) {
      last_error_ = "invalid SLAM mode " + std::to_string(message->mode);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "%s", last_error_.c_str());
      return;
    }
    const bool current_status_fresh = slamStatusFresh();
    if (current_status_fresh && message->status_sequence < slam_status_sequence_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring out-of-order SLAM status sequence %s; current sequence is %s",
        std::to_string(message->status_sequence).c_str(),
        std::to_string(slam_status_sequence_).c_str());
      return;
    }
    const bool recovered_from_timeout = !current_status_fresh;
    received_slam_status_ = true;
    last_slam_status_receive_time_ = std::chrono::steady_clock::now();
    slam_state_ = message->state;
    if (recovered_from_timeout) {
      slam_mode_initialized_ = false;
    }
    transitionToSlamMode(
      static_cast<SlamMode>(message->mode), message->status_sequence, slam_status_topic_);
  }

  bool validateStamp(
    const builtin_interfaces::msg::Time & stamp_message,
    double maximum_age_s,
    std::string & reason) const
  {
    const rclcpp::Time stamp(stamp_message, get_clock()->get_clock_type());
    if (require_nonzero_stamp_ && stamp.nanoseconds() == 0) {
      reason = "zero timestamp";
      return false;
    }
    if (stamp.nanoseconds() == 0) {
      return true;
    }
    const double age = (now() - stamp).seconds();
    if (maximum_age_s > 0.0 && age > maximum_age_s) {
      reason = "message is stale by " + doubleString(age) + " s";
      return false;
    }
    if (max_future_offset_s_ >= 0.0 && age < -max_future_offset_s_) {
      reason = "message timestamp is in the future by " + doubleString(-age) + " s";
      return false;
    }
    return true;
  }

  bool validatePoseTransform(RigidTransform & transform, std::string & reason) const
  {
    if (!isFinite(transform)) {
      reason = "pose contains a non-finite value";
      return false;
    }
    const double norm = transform.rotation.norm();
    if (!std::isfinite(norm) || norm < 1.0e-12) {
      reason = "quaternion has zero or invalid norm";
      return false;
    }
    if (std::abs(norm - 1.0) > quaternion_norm_tolerance_) {
      reason = "quaternion norm differs from one by " + doubleString(std::abs(norm - 1.0));
      return false;
    }
    transform.rotation.normalize();
    return true;
  }

  bool validateOdomContinuity(
    const RigidTransform & odom_to_body,
    const rclcpp::Time & stamp,
    std::string & reason) const
  {
    if (!have_previous_body_pose_) {
      return true;
    }
    const double delta_time = (stamp - previous_body_stamp_).seconds();
    if (require_monotonic_stamp_ && delta_time <= 0.0) {
      reason = "sensor odometry timestamp is not strictly increasing";
      return false;
    }
    if (delta_time <= 0.0) {
      return true;
    }
    const double translation_delta =
      (odom_to_body.translation - previous_odom_to_body_.translation).norm();
    const double rotation_delta = rotationDistance(
      previous_odom_to_body_.rotation, odom_to_body.rotation);
    const double allowed_translation =
      position_jump_margin_m_ + max_linear_speed_mps_ * delta_time;
    const double allowed_rotation =
      rotation_jump_margin_rad_ + max_angular_speed_radps_ * delta_time;
    if (translation_delta > allowed_translation) {
      reason = "body pose translation jump " + doubleString(translation_delta) +
        " m exceeds limit " + doubleString(allowed_translation) + " m";
      return false;
    }
    if (rotation_delta > allowed_rotation) {
      reason = "body pose rotation jump " + doubleString(rotation_delta) +
        " rad exceeds limit " + doubleString(allowed_rotation) + " rad";
      return false;
    }
    return true;
  }

  void rejectSensorOdom(const std::string & reason)
  {
    ++rejected_sensor_samples_;
    last_error_ = reason;
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "Rejecting sensor odometry: %s", reason.c_str());
  }

  void sensorOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    if (!message) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);

    const SlamMode active_mode = effectiveSlamMode();
    if (active_mode == SlamMode::kStopped) {
      ++ignored_inactive_sensor_samples_;
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring sensor odometry while SLAM is stopped or its status is stale");
      return;
    }

    if (message->header.frame_id != odom_frame_) {
      rejectSensorOdom(
        "header.frame_id='" + message->header.frame_id + "', expected '" + odom_frame_ + "'");
      return;
    }
    if (message->child_frame_id != tracking_frame_) {
      rejectSensorOdom(
        "child_frame_id='" + message->child_frame_id + "', expected '" + tracking_frame_ + "'");
      return;
    }
    std::string reason;
    if (!validateStamp(message->header.stamp, max_input_age_s_, reason)) {
      rejectSensorOdom(reason);
      return;
    }

    RigidTransform odom_to_tracking = fromPose(message->pose.pose);
    if (!validatePoseTransform(odom_to_tracking, reason)) {
      rejectSensorOdom(reason);
      return;
    }
    const RigidTransform odom_to_body = compose(odom_to_tracking, inverse(body_to_tracking_));
    const rclcpp::Time stamp(message->header.stamp, get_clock()->get_clock_type());
    if (!validateOdomContinuity(odom_to_body, stamp, reason)) {
      rejectSensorOdom(reason);
      return;
    }

    nav_msgs::msg::Odometry local_body = *message;
    local_body.header.frame_id = odom_frame_;
    local_body.child_frame_id = base_frame_;
    local_body.pose.pose = toPose(odom_to_body);
    local_body.pose.covariance = transformPoseCovarianceSensorToBase(
      message->pose.covariance,
      odom_to_body.rotation.toRotationMatrix(),
      body_to_tracking_.translation);
    transformTwistToBody(*message, local_body);

    if (publish_local_body_odom_) {
      local_body_pub_->publish(local_body);
    }
    if (authorityMode()) {
      publishDynamicTransform(odom_frame_, base_frame_, odom_to_body, message->header.stamp);
      if (globalTransformAvailable()) {
        publishMapToOdomTransform(message->header.stamp);
      }
    }

    if (globalTransformAvailable()) {
      const RigidTransform map_to_body =
        map_frame_ == odom_frame_ ? odom_to_body : compose(map_to_odom_, odom_to_body);
      nav_msgs::msg::Odometry global_body = local_body;
      global_body.header.frame_id = map_frame_;
      global_body.pose.pose = toPose(map_to_body);
      if (map_frame_ != odom_frame_) {
        global_body.pose.covariance = rotateCovariance(
          local_body.pose.covariance, map_to_odom_.rotation.toRotationMatrix());
        if (add_map_covariance_) {
          for (std::size_t index = 0; index < global_body.pose.covariance.size(); ++index) {
            global_body.pose.covariance[index] += map_to_odom_covariance_[index];
          }
        }
      }
      if (publish_global_body_odom_) {
        global_body_pub_->publish(global_body);
      }
    }

    previous_odom_to_body_ = odom_to_body;
    previous_body_stamp_ = stamp;
    have_previous_body_pose_ = true;
    last_sensor_receive_time_ = std::chrono::steady_clock::now();
    received_sensor_odom_ = true;
    ++accepted_sensor_samples_;
    last_error_.clear();
    publishReadiness(computeReady());
  }

  void transformTwistToBody(
    const nav_msgs::msg::Odometry & sensor_message,
    nav_msgs::msg::Odometry & body_message) const
  {
    const Eigen::Vector3d sensor_linear(
      sensor_message.twist.twist.linear.x,
      sensor_message.twist.twist.linear.y,
      sensor_message.twist.twist.linear.z);
    const Eigen::Vector3d sensor_angular(
      sensor_message.twist.twist.angular.x,
      sensor_message.twist.twist.angular.y,
      sensor_message.twist.twist.angular.z);
    const Eigen::Matrix3d rotation = body_to_tracking_.rotation.toRotationMatrix();
    const Eigen::Vector3d body_angular = rotation * sensor_angular;
    const Eigen::Vector3d body_linear =
      rotation * sensor_linear + body_to_tracking_.translation.cross(body_angular);
    body_message.twist.twist.linear.x = body_linear.x();
    body_message.twist.twist.linear.y = body_linear.y();
    body_message.twist.twist.linear.z = body_linear.z();
    body_message.twist.twist.angular.x = body_angular.x();
    body_message.twist.twist.angular.y = body_angular.y();
    body_message.twist.twist.angular.z = body_angular.z();
    body_message.twist.covariance = transformTwistCovarianceSensorToBase(
      sensor_message.twist.covariance, body_to_tracking_);
  }

  void icpPoseCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    if (!message) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (effectiveSlamMode() != SlamMode::kLocalization) {
      ++ignored_inactive_map_samples_;
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring ICP pose outside localization mode");
      return;
    }
    if (message->header.frame_id != map_frame_) {
      rejectMapPose(
        "header.frame_id='" + message->header.frame_id + "', expected '" + map_frame_ + "'");
      return;
    }
    if (message->child_frame_id != icp_sensor_frame_) {
      rejectMapPose(
        "child_frame_id='" + message->child_frame_id + "', expected '" +
        icp_sensor_frame_ + "'");
      return;
    }
    std::string reason;
    if (!validateStamp(message->header.stamp, max_icp_age_s_, reason)) {
      rejectMapPose(reason);
      return;
    }
    RigidTransform map_to_icp_sensor = fromPose(message->pose.pose);
    if (!validatePoseTransform(map_to_icp_sensor, reason)) {
      rejectMapPose(reason);
      return;
    }
    // At relocalization startup FAST-LIO's odom->tracking state is identity.
    // ICP provides T_map_icp_sensor, so normalize it through the reviewed
    // tracking->ICP-sensor extrinsic before claiming T_map_odom.
    const RigidTransform candidate = compose(
      map_to_icp_sensor, inverse(tracking_to_icp_sensor_));
    if (map_initialized_) {
      if (!allow_map_reinitialization_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignoring later ICP pose because map_to_odom.allow_reinitialization=false");
        return;
      }
      const double translation_jump =
        (candidate.translation - map_to_odom_.translation).norm();
      const double rotation_jump = rotationDistance(candidate.rotation, map_to_odom_.rotation);
      if (translation_jump > map_translation_jump_limit_m_ ||
        rotation_jump > map_rotation_jump_limit_rad_)
      {
        rejectMapPose(
          "reinitialization jump exceeds configured map-to-odom limits");
        return;
      }
    }

    map_to_odom_ = candidate;
    map_to_odom_covariance_ = transformPoseCovarianceSensorToBase(
      message->pose.covariance, candidate.rotation.toRotationMatrix(),
      tracking_to_icp_sensor_.translation);
    map_initialized_ = true;
    ++accepted_map_samples_;
    last_error_.clear();
    if (authorityMode()) {
      publishMapToOdomTransform(message->header.stamp);
    }
    publishReadiness(computeReady());
    RCLCPP_INFO(
      get_logger(), "Accepted map alignment %s -> %s from %s",
      map_frame_.c_str(), odom_frame_.c_str(), icp_pose_topic_.c_str());
  }

  void rejectMapPose(const std::string & reason)
  {
    ++rejected_map_samples_;
    last_error_ = reason;
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "Rejecting map-to-odom pose: %s", reason.c_str());
  }

  bool globalTransformAvailable() const
  {
    return effectiveSlamMode() != SlamMode::kStopped && map_initialized_;
  }

  bool computeReady() const
  {
    if (effectiveSlamMode() == SlamMode::kStopped) {
      return false;
    }
    if (!received_sensor_odom_) {
      return false;
    }
    if (publish_global_body_odom_ && !globalTransformAvailable()) {
      return false;
    }
    const double receive_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_sensor_receive_time_).count();
    return receive_age <= sensor_timeout_s_;
  }

  void publishDynamicTransform(
    const std::string & parent,
    const std::string & child,
    const RigidTransform & transform,
    const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = parent;
    message.child_frame_id = child;
    message.transform = toTransform(transform);
    tf_broadcaster_->sendTransform(message);
  }

  void publishMapToOdomTransform(const builtin_interfaces::msg::Time & stamp)
  {
    if (!authorityMode() || !map_initialized_ || effectiveSlamMode() == SlamMode::kStopped) {
      return;
    }
    publishDynamicTransform(map_frame_, odom_frame_, map_to_odom_, stamp);
  }

  void publishSensorStaticTransforms()
  {
    if (!authorityMode()) {
      return;
    }
    std::vector<geometry_msgs::msg::TransformStamped> messages;
    messages.reserve(static_transforms_.size());
    const auto stamp = now();
    for (const auto & spec : static_transforms_) {
      geometry_msgs::msg::TransformStamped message;
      message.header.stamp = stamp;
      message.header.frame_id = spec.parent_frame;
      message.child_frame_id = spec.child_frame;
      message.transform = toTransform(spec.transform);
      messages.push_back(std::move(message));
    }
    static_tf_broadcaster_->sendTransform(messages);
  }

  void publishReadiness(bool ready)
  {
    if (ready == last_published_ready_ && have_published_ready_) {
      return;
    }
    std_msgs::msg::Bool message;
    message.data = ready;
    ready_pub_->publish(message);
    last_published_ready_ = ready;
    have_published_ready_ = true;
  }

  void diagnosticsTimerCallback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool ready = computeReady();
    publishReadiness(ready);

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "omni_tf_manager/core";
    status.hardware_id = profile_name_;
    if (ready) {
      status.level = kDiagnosticOk;
      status.message = authorityMode() ? "TF authority ready" : "shadow candidate ready";
    } else if (!last_error_.empty()) {
      status.level = kDiagnosticError;
      status.message = last_error_;
    } else {
      status.level = kDiagnosticWarn;
      if (!slamStatusFresh()) {
        status.message = received_slam_status_ ?
          "SLAM status timed out" : "waiting for SLAM status";
      } else if (effectiveSlamMode() == SlamMode::kStopped) {
        status.message = "SLAM stopped; dynamic TF disabled";
      } else {
        status.message = received_sensor_odom_ ?
          "waiting for fresh odometry or map alignment" : "waiting for sensor odometry";
      }
    }
    const double sensor_age = received_sensor_odom_ ?
      std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_sensor_receive_time_).count() : -1.0;
    status.values.push_back(keyValue("mode", mode_));
    status.values.push_back(keyValue("slam_mode", slamModeName(effectiveSlamMode())));
    status.values.push_back(keyValue("slam_state", std::to_string(slam_state_)));
    status.values.push_back(
      keyValue("slam_status_sequence", std::to_string(slam_status_sequence_)));
    status.values.push_back(keyValue("slam_status_fresh", boolString(slamStatusFresh())));
    status.values.push_back(keyValue("profile", profile_name_));
    status.values.push_back(keyValue("profile_version", profile_version_));
    status.values.push_back(keyValue("calibration_id", calibration_id_));
    status.values.push_back(keyValue("ready", boolString(ready)));
    status.values.push_back(keyValue("map_frame", map_frame_));
    status.values.push_back(keyValue("odom_frame", odom_frame_));
    status.values.push_back(keyValue("base_frame", base_frame_));
    status.values.push_back(keyValue("tracking_frame", tracking_frame_));
    status.values.push_back(keyValue("icp_sensor_frame", icp_sensor_frame_));
    status.values.push_back(keyValue("map_initialized", boolString(map_initialized_)));
    status.values.push_back(keyValue("sensor_age_s", doubleString(sensor_age)));
    status.values.push_back(
      keyValue(
        "accepted_sensor_samples", std::to_string(accepted_sensor_samples_)));
    status.values.push_back(
      keyValue(
        "rejected_sensor_samples", std::to_string(rejected_sensor_samples_)));
    status.values.push_back(
      keyValue(
        "ignored_inactive_sensor_samples",
        std::to_string(ignored_inactive_sensor_samples_)));
    status.values.push_back(
      keyValue(
        "accepted_map_samples", std::to_string(accepted_map_samples_)));
    status.values.push_back(
      keyValue(
        "rejected_map_samples", std::to_string(rejected_map_samples_)));
    status.values.push_back(
      keyValue(
        "ignored_inactive_map_samples", std::to_string(ignored_inactive_map_samples_)));
    status.values.push_back(
      keyValue(
        "static_transform_count", std::to_string(static_transforms_.size())));
    status.values.push_back(
      keyValue("sensor_relay_count", std::to_string(sensor_relay_count_)));
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
  }

  void logOwnershipSummary() const
  {
    RCLCPP_INFO(
      get_logger(), "omni_tf_manager profile='%s' mode='%s'",
      profile_name_.c_str(), mode_.c_str());
    RCLCPP_INFO(
      get_logger(), "SLAM state source: %s%s",
      require_slam_status_ ? slam_status_topic_.c_str() : "fallback parameter: ",
      require_slam_status_ ? "" : slamModeName(fallback_slam_mode_).c_str());
    RCLCPP_INFO(
      get_logger(), "Candidate dynamic chain: %s -> %s -> %s (tracking input: %s)",
      map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str(), tracking_frame_.c_str());
    for (const auto & transform : static_transforms_) {
      RCLCPP_INFO(
        get_logger(), "Static TF [%s/%s]: %s -> %s",
        transform.id.c_str(), transform.role.c_str(),
        transform.parent_frame.c_str(), transform.child_frame.c_str());
    }
    if (!authorityMode()) {
      RCLCPP_WARN(
        get_logger(),
        "Shadow mode: no /tf or /tf_static transforms will be published. "
        "Use authority mode only after disabling legacy publishers for the same edges.");
    }
  }

  std::string mode_;
  std::string profile_name_;
  std::string profile_version_;
  std::string calibration_id_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string tracking_frame_;
  std::string icp_sensor_frame_;
  std::string sensor_odom_topic_;
  std::string icp_pose_topic_;
  std::string slam_status_topic_;
  std::string local_body_odom_topic_;
  std::string global_body_odom_topic_;
  std::string diagnostics_topic_;
  std::string ready_topic_;

  bool publish_local_body_odom_{true};
  bool publish_global_body_odom_{true};
  bool add_map_covariance_{false};
  bool require_omni_prefix_{true};
  bool require_slam_status_{true};
  bool allow_map_reinitialization_{false};
  bool require_nonzero_stamp_{true};
  bool require_monotonic_stamp_{true};

  double map_translation_jump_limit_m_{2.0};
  double map_rotation_jump_limit_rad_{0.7853981633974483};
  double slam_status_timeout_s_{2.0};
  double max_input_age_s_{1.0};
  double max_icp_age_s_{30.0};
  double max_future_offset_s_{0.1};
  double quaternion_norm_tolerance_{0.05};
  double max_linear_speed_mps_{5.0};
  double max_angular_speed_radps_{8.0};
  double position_jump_margin_m_{0.5};
  double rotation_jump_margin_rad_{0.35};
  double sensor_timeout_s_{0.5};
  double diagnostics_period_s_{0.5};

  std::vector<std::string> required_sensor_roles_;
  std::vector<StaticTransformSpec> static_transforms_;
  RigidTransform body_to_tracking_;
  RigidTransform tracking_to_icp_sensor_;
  RigidTransform map_to_odom_;
  Covariance6d map_to_odom_covariance_{};
  bool map_initialized_{false};

  SlamMode fallback_slam_mode_{SlamMode::kStopped};
  SlamMode slam_mode_{SlamMode::kStopped};
  std::uint8_t slam_state_{omni_slam_interfaces::msg::SlamStatus::STATE_STOPPED};
  std::uint64_t slam_status_sequence_{0U};
  std::chrono::steady_clock::time_point last_slam_status_receive_time_{};
  bool slam_mode_initialized_{false};
  bool received_slam_status_{false};

  RigidTransform previous_odom_to_body_;
  rclcpp::Time previous_body_stamp_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_sensor_receive_time_{};
  bool have_previous_body_pose_{false};
  bool received_sensor_odom_{false};
  bool last_published_ready_{false};
  bool have_published_ready_{false};
  std::uint64_t accepted_sensor_samples_{0U};
  std::uint64_t rejected_sensor_samples_{0U};
  std::uint64_t ignored_inactive_sensor_samples_{0U};
  std::uint64_t accepted_map_samples_{0U};
  std::uint64_t rejected_map_samples_{0U};
  std::uint64_t ignored_inactive_map_samples_{0U};
  std::size_t sensor_relay_count_{0U};
  std::string last_error_;

  std::mutex mutex_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_body_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr global_body_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sensor_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr icp_pose_sub_;
  rclcpp::Subscription<omni_slam_interfaces::msg::SlamStatus>::SharedPtr slam_status_sub_;
  std::vector<rclcpp::PublisherBase::SharedPtr> sensor_relay_publishers_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> sensor_relay_subscriptions_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace omni_tf_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<omni_tf_manager::OmniTfManagerNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("omni_tf_manager"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
