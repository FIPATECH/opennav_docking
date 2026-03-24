// Copyright (c) 2024 Open Navigation LLC
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

#include <algorithm>
#include <cmath>

#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#include "opennav_docking/simple_charging_dock.hpp"

namespace opennav_docking
{

namespace
{

bool transformPoseWithLatestFallback(
  const std::shared_ptr<tf2_ros::Buffer> & tf2_buffer,
  const geometry_msgs::msg::PoseStamped & input,
  geometry_msgs::msg::PoseStamped & output,
  const std::string & target_frame,
  const rclcpp::Duration & timeout)
{
  if (!tf2_buffer) {
    return false;
  }

  if (input.header.frame_id == target_frame) {
    output = input;
    return true;
  }

  try {
    if (tf2_buffer->canTransform(
        target_frame, input.header.frame_id, input.header.stamp, timeout))
    {
      tf2_buffer->transform(input, output, target_frame);
      return true;
    }
  } catch (const tf2::TransformException &) {
    // Fall back to the latest available transform below.
  }

  try {
    const auto transform = tf2_buffer->lookupTransform(
      target_frame, input.header.frame_id, tf2::TimePointZero,
      tf2::durationFromSec(timeout.seconds()));
    tf2::doTransform(input, output, transform);
    output.header.frame_id = target_frame;
    output.header.stamp = input.header.stamp;
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

}  // namespace

void SimpleChargingDock::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf)
{
  name_ = name;
  tf2_buffer_ = tf;
  is_charging_ = false;
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Optionally use battery info to check when charging, else say charging if docked
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".use_battery_status", rclcpp::ParameterValue(true));

  // Parameters for optional external detection of dock pose
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".use_external_detection_pose", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_timeout", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_translation_x", rclcpp::ParameterValue(-0.20));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_translation_y", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_use_relative_target_pose", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_target_x", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_target_y", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_target_yaw", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_yaw", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_pitch", rclcpp::ParameterValue(1.57));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_roll", rclcpp::ParameterValue(-1.57));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".filter_coef", rclcpp::ParameterValue(0.1));

  // Charging threshold from BatteryState message
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".charging_threshold", rclcpp::ParameterValue(0.5));

  // Optionally determine if docked via stall detection using joint_states
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".use_stall_detection", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".stall_joint_names", rclcpp::PARAMETER_STRING_ARRAY);
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".stall_velocity_threshold", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".stall_effort_threshold", rclcpp::ParameterValue(1.0));

  // If not using stall detection, this is how close robot should get to pose
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".docking_threshold", rclcpp::ParameterValue(0.05));

  // Staging pose configuration
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".staging_x_offset", rclcpp::ParameterValue(-0.7));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".staging_yaw_offset", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, "base_frame", rclcpp::ParameterValue("base_link"));

  node_->get_parameter(name + ".use_battery_status", use_battery_status_);
  node_->get_parameter(name + ".use_external_detection_pose", use_external_detection_pose_);
  node_->get_parameter(name + ".external_detection_timeout", external_detection_timeout_);
  node_->get_parameter(
    name + ".external_detection_translation_x", external_detection_translation_x_);
  node_->get_parameter(
    name + ".external_detection_translation_y", external_detection_translation_y_);
  node_->get_parameter(
    name + ".external_detection_use_relative_target_pose",
    external_detection_use_relative_target_pose_);
  node_->get_parameter(name + ".external_detection_target_x", external_detection_target_x_);
  node_->get_parameter(name + ".external_detection_target_y", external_detection_target_y_);
  node_->get_parameter(name + ".external_detection_target_yaw", external_detection_target_yaw_);
  double yaw, pitch, roll;
  node_->get_parameter(name + ".external_detection_rotation_yaw", yaw);
  node_->get_parameter(name + ".external_detection_rotation_pitch", pitch);
  node_->get_parameter(name + ".external_detection_rotation_roll", roll);
  external_detection_rotation_.setEuler(pitch, roll, yaw);
  node_->get_parameter(name + ".charging_threshold", charging_threshold_);
  node_->get_parameter(name + ".stall_velocity_threshold", stall_velocity_threshold_);
  node_->get_parameter(name + ".stall_effort_threshold", stall_effort_threshold_);
  node_->get_parameter(name + ".docking_threshold", docking_threshold_);
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".docking_threshold_x", rclcpp::ParameterValue(-1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".docking_threshold_y", rclcpp::ParameterValue(-1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".docking_threshold_yaw", rclcpp::ParameterValue(-1.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".docking_settle_hits_required", rclcpp::ParameterValue(1));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".docking_settle_duration_s", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name + ".docking_threshold_x", docking_threshold_x_);
  node_->get_parameter(name + ".docking_threshold_y", docking_threshold_y_);
  node_->get_parameter(name + ".docking_threshold_yaw", docking_threshold_yaw_);
  node_->get_parameter(name + ".docking_settle_hits_required", docking_settle_hits_required_);
  node_->get_parameter(name + ".docking_settle_duration_s", docking_settle_duration_s_);
  node_->get_parameter(name + ".staging_x_offset", staging_x_offset_);
  node_->get_parameter(name + ".staging_yaw_offset", staging_yaw_offset_);
  node_->get_parameter("base_frame", base_frame_id_);  // Get server base frame ID
  docking_candidate_hits_ = 0;
  docking_candidate_first_stamp_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());

  // Setup filter
  double filter_coef;
  node_->get_parameter(name + ".filter_coef", filter_coef);
  filter_ = std::make_unique<PoseFilter>(filter_coef, external_detection_timeout_);

  if (use_battery_status_) {
    battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
      "battery_state", 1,
      [this](const sensor_msgs::msg::BatteryState::SharedPtr state) {
        is_charging_ = state->current > charging_threshold_;
      });
  }

  if (use_external_detection_pose_) {
    dock_pose_.header.stamp = rclcpp::Time(0);
    dock_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "detected_dock_pose", 1,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
        detected_dock_pose_ = *pose;
      });
  }

  bool use_stall_detection;
  node_->get_parameter(name + ".use_stall_detection", use_stall_detection);
  if (use_stall_detection) {
    is_stalled_ = false;
    node_->get_parameter(name + ".stall_joint_names", stall_joint_names_);
    if (stall_joint_names_.size() < 1) {
      RCLCPP_ERROR(node_->get_logger(), "stall_joint_names cannot be empty!");
    }
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 1,
      std::bind(&SimpleChargingDock::jointStateCallback, this, std::placeholders::_1));
  }

  dock_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("dock_pose", 1);
  filtered_dock_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "filtered_dock_pose", 1);
  staging_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("staging_pose", 1);
}

geometry_msgs::msg::PoseStamped SimpleChargingDock::getStagingPose(
  const geometry_msgs::msg::Pose & pose, const std::string & frame)
{
  // If not using detection, set the dock pose as the given dock pose estimate
  if (!use_external_detection_pose_) {
    // This gets called at the start of docking
    // Reset our internally tracked dock pose
    dock_pose_.header.frame_id = frame;
    dock_pose_.pose = pose;
  }

  // Compute the staging pose with given offsets
  const double yaw = tf2::getYaw(pose.orientation);
  geometry_msgs::msg::PoseStamped staging_pose;
  staging_pose.header.frame_id = frame;
  staging_pose.header.stamp = node_->now();
  staging_pose.pose = pose;
  staging_pose.pose.position.x += cos(yaw) * staging_x_offset_;
  staging_pose.pose.position.y += sin(yaw) * staging_x_offset_;
  tf2::Quaternion orientation;
  orientation.setEuler(0.0, 0.0, yaw + staging_yaw_offset_);
  staging_pose.pose.orientation = tf2::toMsg(orientation);

  // Publish staging pose for debugging purposes
  staging_pose_pub_->publish(staging_pose);
  return staging_pose;
}

bool SimpleChargingDock::getRefinedPose(geometry_msgs::msg::PoseStamped & pose)
{
  // If using not detection, set the dock pose to the static fixed-frame version
  if (!use_external_detection_pose_) {
    dock_pose_pub_->publish(pose);
    dock_pose_ = pose;
    return true;
  }

  // If using detections, get current detections, transform to frame, and apply offsets
  geometry_msgs::msg::PoseStamped detected = detected_dock_pose_;

  // Validate that external pose is new enough
  auto timeout = rclcpp::Duration::from_seconds(external_detection_timeout_);
  if (node_->now() - detected.header.stamp > timeout) {
    RCLCPP_WARN(
      node_->get_logger(), "Lost detection or did not detect: "
      "timeout exceeded (is %2.2f seconds old)",
      static_cast<float>((node_->now() - detected.header.stamp).seconds()));
    return false;
  }

  // Transform detected pose into fixed frame. Note that the argument pose
  // is the output of detection, but also acts as the initial estimate
  // and contains the frame_id of docking
  if (detected.header.frame_id != pose.header.frame_id) {
    if (!transformPoseWithLatestFallback(
        tf2_buffer_, detected, detected, pose.header.frame_id,
        rclcpp::Duration::from_seconds(0.2)))
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "Failed to transform detected dock pose from %s to %s at stamp %.2f",
        detected.header.frame_id.c_str(),
        pose.header.frame_id.c_str(),
        static_cast<float>(detected.header.stamp.sec + detected.header.stamp.nanosec * 1e-9));
      return false;
    }
  }

  // Filter the detected pose
  detected = filter_->update(detected);
  filtered_dock_pose_pub_->publish(detected);

  // Rotate the just the orientation, then remove roll/pitch
  geometry_msgs::msg::PoseStamped just_orientation;
  just_orientation.pose.orientation = tf2::toMsg(external_detection_rotation_);
  geometry_msgs::msg::TransformStamped transform;
  transform.transform.rotation = detected.pose.orientation;
  tf2::doTransform(just_orientation, just_orientation, transform);

  tf2::Quaternion orientation;
  const double adjusted_marker_yaw = tf2::getYaw(just_orientation.pose.orientation);

  // Construct dock_pose_ by applying translation/rotation
  dock_pose_.header = detected.header;
  dock_pose_.pose.position = detected.pose.position;
  double dock_yaw = adjusted_marker_yaw;
  if (external_detection_use_relative_target_pose_) {
    dock_yaw = angles::normalize_angle(adjusted_marker_yaw - external_detection_target_yaw_);
    dock_pose_.pose.position.x -=
      std::cos(dock_yaw) * external_detection_target_x_ -
      std::sin(dock_yaw) * external_detection_target_y_;
    dock_pose_.pose.position.y -=
      std::sin(dock_yaw) * external_detection_target_x_ +
      std::cos(dock_yaw) * external_detection_target_y_;
  } else {
    dock_yaw = adjusted_marker_yaw;
    dock_pose_.pose.position.x += cos(dock_yaw) * external_detection_translation_x_ -
      sin(dock_yaw) * external_detection_translation_y_;
    dock_pose_.pose.position.y += sin(dock_yaw) * external_detection_translation_x_ +
      cos(dock_yaw) * external_detection_translation_y_;
  }
  orientation.setEuler(0.0, 0.0, dock_yaw);
  dock_pose_.pose.orientation = tf2::toMsg(orientation);
  dock_pose_.pose.position.z = 0.0;

  // Publish & return dock pose for debugging purposes
  dock_pose_pub_->publish(dock_pose_);
  pose = dock_pose_;
  return true;
}

bool SimpleChargingDock::getCurrentRelativeMarkerPose(geometry_msgs::msg::PoseStamped & pose) const
{
  if (!use_external_detection_pose_) {
    return false;
  }

  geometry_msgs::msg::PoseStamped detected = detected_dock_pose_;
  if (detected.header.frame_id.empty()) {
    return false;
  }

  const auto timeout = rclcpp::Duration::from_seconds(external_detection_timeout_);
  if (node_->now() - detected.header.stamp > timeout) {
    return false;
  }

  if (detected.header.frame_id == base_frame_id_) {
    pose = detected;
    return true;
  }

  if (!tf2_buffer_) {
    return false;
  }

  try {
    if (!tf2_buffer_->canTransform(
        base_frame_id_, detected.header.frame_id,
        detected.header.stamp, rclcpp::Duration::from_seconds(0.2)))
    {
      return false;
    }
    tf2_buffer_->transform(detected, pose, base_frame_id_);
    return true;
  } catch (const tf2::TransformException &) {
    try {
      const auto transform = tf2_buffer_->lookupTransform(
        base_frame_id_, detected.header.frame_id, tf2::TimePointZero,
        tf2::durationFromSec(0.2));
      tf2::doTransform(detected, pose, transform);
      pose.header.frame_id = base_frame_id_;
      pose.header.stamp = detected.header.stamp;
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }
}

bool SimpleChargingDock::isDocked()
{
  if (joint_state_sub_) {
    // Using stall detection
    return is_stalled_;
  }

  if (dock_pose_.header.frame_id.empty()) {
    // Dock pose is not yet valid
    return false;
  }

  const bool inside_window = isInsideDockingWindowRaw();

  if (!inside_window) {
    docking_candidate_hits_ = 0;
    docking_candidate_first_stamp_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    return false;
  }

  if (docking_candidate_hits_ == 0) {
    docking_candidate_first_stamp_ = node_->now();
  }
  ++docking_candidate_hits_;

  const bool hits_ok = docking_candidate_hits_ >= std::max(1, docking_settle_hits_required_);
  const bool duration_ok =
    docking_settle_duration_s_ <= 0.0 ||
    (node_->now() - docking_candidate_first_stamp_).seconds() >= docking_settle_duration_s_;
  return hits_ok && duration_ok;
}

bool SimpleChargingDock::isInsideDockingWindowRaw() const
{
  if (dock_pose_.header.frame_id.empty()) {
    return false;
  }

  if (use_external_detection_pose_ && external_detection_use_relative_target_pose_) {
    double x_error = 0.0;
    double y_error = 0.0;
    double yaw_error = 0.0;
    if (!getRelativeTargetErrorsRaw(x_error, y_error, yaw_error)) {
      return false;
    }

    if (docking_threshold_x_ > 0.0 && docking_threshold_y_ > 0.0 && docking_threshold_yaw_ > 0.0) {
      return
        std::abs(x_error) < docking_threshold_x_ &&
        std::abs(y_error) < docking_threshold_y_ &&
        std::abs(yaw_error) < docking_threshold_yaw_;
    }
    return std::hypot(x_error, y_error) < docking_threshold_;
  }

  geometry_msgs::msg::PoseStamped base_pose;
  base_pose.header.stamp = rclcpp::Time(0);
  base_pose.header.frame_id = base_frame_id_;
  base_pose.pose.orientation.w = 1.0;
  try {
    tf2_buffer_->transform(base_pose, base_pose, dock_pose_.header.frame_id);
  } catch (const tf2::TransformException &) {
    return false;
  }

  const double dx_world = base_pose.pose.position.x - dock_pose_.pose.position.x;
  const double dy_world = base_pose.pose.position.y - dock_pose_.pose.position.y;
  const double dock_yaw = tf2::getYaw(dock_pose_.pose.orientation);
  const double base_yaw = tf2::getYaw(base_pose.pose.orientation);
  const double cos_yaw = std::cos(dock_yaw);
  const double sin_yaw = std::sin(dock_yaw);
  const double x_error = cos_yaw * dx_world + sin_yaw * dy_world;
  const double y_error = -sin_yaw * dx_world + cos_yaw * dy_world;
  const double yaw_error = angles::shortest_angular_distance(base_yaw, dock_yaw);

  if (docking_threshold_x_ > 0.0 && docking_threshold_y_ > 0.0 && docking_threshold_yaw_ > 0.0) {
    return
      std::abs(x_error) < docking_threshold_x_ &&
      std::abs(y_error) < docking_threshold_y_ &&
      std::abs(yaw_error) < docking_threshold_yaw_;
  }
  return std::hypot(dx_world, dy_world) < docking_threshold_;
}

bool SimpleChargingDock::getRelativeTargetErrorsRaw(
  double & x_error, double & y_error, double & yaw_error) const
{
  if (!use_external_detection_pose_ || !external_detection_use_relative_target_pose_) {
    return false;
  }

  geometry_msgs::msg::PoseStamped marker_pose_in_base;
  if (!getCurrentRelativeMarkerPose(marker_pose_in_base)) {
    return false;
  }

  x_error = marker_pose_in_base.pose.position.x - external_detection_target_x_;
  y_error = marker_pose_in_base.pose.position.y - external_detection_target_y_;
  const double marker_yaw = tf2::getYaw(marker_pose_in_base.pose.orientation);
  yaw_error = angles::shortest_angular_distance(marker_yaw, external_detection_target_yaw_);
  return true;
}

bool SimpleChargingDock::isCharging()
{
  return use_battery_status_ ? is_charging_ : isDocked();
}

bool SimpleChargingDock::disableCharging()
{
  return true;
}

bool SimpleChargingDock::hasStoppedCharging()
{
  return !isCharging();
}

void SimpleChargingDock::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr state)
{
  double velocity = 0.0;
  double effort = 0.0;
  for (size_t i = 0; i < state->name.size(); ++i) {
    for (auto & name : stall_joint_names_) {
      if (state->name[i] == name) {
        // Tracking this joint
        velocity += abs(state->velocity[i]);
        effort += abs(state->effort[i]);
      }
    }
  }

  // Take average
  effort /= stall_joint_names_.size();
  velocity /= stall_joint_names_.size();

  is_stalled_ = (velocity < stall_velocity_threshold_) && (effort > stall_effort_threshold_);
}

}  // namespace opennav_docking

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(opennav_docking::SimpleChargingDock, opennav_docking_core::ChargingDock)
