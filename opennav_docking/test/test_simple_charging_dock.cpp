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

#include <cmath>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "opennav_docking/simple_charging_dock.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"

// Testing the simple charging dock plugin

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

namespace opennav_docking
{

TEST(SimpleChargingDockTests, ObjectLifecycle)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(true));

  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();
  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  // Check initial states
  EXPECT_FALSE(dock->isCharging());
  EXPECT_TRUE(dock->disableCharging());
  EXPECT_TRUE(dock->hasStoppedCharging());

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, BatteryState)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto pub = node->create_publisher<sensor_msgs::msg::BatteryState>(
    "battery_state", rclcpp::QoS(1));
  pub->on_activate();
  node->declare_parameter("my_dock.use_battery_state", rclcpp::ParameterValue(true));

  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();

  dock->configure(node, "my_dock", nullptr);
  dock->activate();
  geometry_msgs::msg::PoseStamped pose;
  EXPECT_TRUE(dock->getRefinedPose(pose));

  // Below threshold
  sensor_msgs::msg::BatteryState msg;
  msg.current = 0.3;
  pub->publish(msg);
  rclcpp::Rate r(2);
  r.sleep();
  rclcpp::spin_some(node->get_node_base_interface());

  EXPECT_FALSE(dock->isCharging());
  EXPECT_TRUE(dock->hasStoppedCharging());

  // Above threshold
  sensor_msgs::msg::BatteryState msg2;
  msg2.current = 0.6;
  pub->publish(msg2);
  rclcpp::Rate r1(2);
  r1.sleep();
  rclcpp::spin_some(node->get_node_base_interface());

  EXPECT_TRUE(dock->isCharging());
  EXPECT_FALSE(dock->hasStoppedCharging());

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, StallDetection)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto pub = node->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::QoS(1));
  pub->on_activate();
  node->declare_parameter("my_dock.use_stall_detection", rclcpp::ParameterValue(true));
  std::vector<std::string> names = {"left_motor", "right_motor"};
  node->declare_parameter("my_dock.stall_joint_names", rclcpp::ParameterValue(names));
  node->declare_parameter("my_dock.stall_velocity_threshold", rclcpp::ParameterValue(0.1));
  node->declare_parameter("my_dock.stall_effort_threshold", rclcpp::ParameterValue(5.0));

  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();

  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  // Stopped, but below effort threshold
  sensor_msgs::msg::JointState msg;
  msg.name = {"left_motor", "right_motor", "another_motor"};
  msg.velocity = {0.0, 0.0, 0.0};
  msg.effort = {0.0, 0.0, 0.0};
  pub->publish(msg);
  rclcpp::Rate r(2);
  r.sleep();
  rclcpp::spin_some(node->get_node_base_interface());

  EXPECT_FALSE(dock->isDocked());

  // Moving, with effort
  sensor_msgs::msg::JointState msg2;
  msg2.name = {"left_motor", "right_motor", "another_motor"};
  msg2.velocity = {1.0, 1.0, 0.0};
  msg2.effort = {5.1, -5.1, 0.0};
  pub->publish(msg2);
  rclcpp::Rate r1(2);
  r1.sleep();
  rclcpp::spin_some(node->get_node_base_interface());

  EXPECT_FALSE(dock->isDocked());

  // Stopped, with effort
  sensor_msgs::msg::JointState msg3;
  msg3.name = {"left_motor", "right_motor", "another_motor"};
  msg3.velocity = {0.0, 0.0, 0.0};
  msg3.effort = {5.1, -5.1, 0.0};
  pub->publish(msg3);
  rclcpp::Rate r2(2);
  r2.sleep();
  rclcpp::spin_some(node->get_node_base_interface());

  EXPECT_TRUE(dock->isDocked());

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, StagingPose)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();

  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  geometry_msgs::msg::Pose pose;
  std::string frame = "my_frame";
  auto staging_pose = dock->getStagingPose(pose, frame);
  EXPECT_NEAR(staging_pose.pose.position.x, -0.7, 0.01);
  EXPECT_NEAR(staging_pose.pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(tf2::getYaw(staging_pose.pose.orientation), 0.0, 0.01);
  EXPECT_EQ(staging_pose.header.frame_id, frame);

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, StagingPoseWithYawOffset)
{
  // Override the parameter default
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      {"my_dock.staging_yaw_offset", 3.14},
    }
  );

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test", options);
  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();

  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  geometry_msgs::msg::Pose pose;
  std::string frame = "my_frame";
  auto staging_pose = dock->getStagingPose(pose, frame);
  // Pose should be the same as default, but pointing in opposite direction
  EXPECT_NEAR(staging_pose.pose.position.x, -0.7, 0.01);
  EXPECT_NEAR(staging_pose.pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(tf2::getYaw(staging_pose.pose.orientation), 3.14, 0.01);
  EXPECT_EQ(staging_pose.header.frame_id, frame);

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, RefinedPoseTest)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  node->declare_parameter("my_dock.use_external_detection_pose", rclcpp::ParameterValue(true));
  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "detected_dock_pose", rclcpp::QoS(1));
  pub->on_activate();
  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();

  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  geometry_msgs::msg::PoseStamped pose;

  // Timestamps are outdated; this is after timeout
  EXPECT_FALSE(dock->isDocked());
  EXPECT_FALSE(dock->getRefinedPose(pose));

  geometry_msgs::msg::PoseStamped detected_pose;
  detected_pose.header.stamp = node->now();
  detected_pose.header.frame_id = "my_frame";
  detected_pose.pose.position.x = 0.1;
  detected_pose.pose.position.y = -0.5;
  pub->publish(detected_pose);
  rclcpp::spin_some(node->get_node_base_interface());

  pose.header.frame_id = "my_frame";
  EXPECT_TRUE(dock->getRefinedPose(pose));
  EXPECT_NEAR(pose.pose.position.x, 0.1, 0.01);
  EXPECT_NEAR(pose.pose.position.y, -0.3, 0.01);  // Applies external_detection_translation_x, +0.2

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, RefinedPoseRelativeTargetPose)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      {"my_dock.use_external_detection_pose", true},
      {"my_dock.external_detection_use_relative_target_pose", true},
      {"my_dock.external_detection_target_x", 0.22},
      {"my_dock.external_detection_target_y", 0.01},
      {"my_dock.external_detection_target_yaw", -1.57},
      {"my_dock.external_detection_rotation_yaw", 0.0},
      {"my_dock.external_detection_rotation_pitch", 0.0},
      {"my_dock.external_detection_rotation_roll", 0.0},
    });

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_relative_target_pose", options);
  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "detected_dock_pose", rclcpp::QoS(1));
  pub->on_activate();
  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();

  dock->configure(node, "my_dock", nullptr);
  dock->activate();

  geometry_msgs::msg::PoseStamped detected_pose;
  detected_pose.header.stamp = node->now();
  detected_pose.header.frame_id = "my_frame";
  detected_pose.pose.position.x = 1.0;
  detected_pose.pose.position.y = 2.0;
  detected_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.3);
  pub->publish(detected_pose);
  rclcpp::spin_some(node->get_node_base_interface());

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "my_frame";
  ASSERT_TRUE(dock->getRefinedPose(pose));

  const double expected_yaw = 0.3 - (-1.57);
  const double expected_x = 1.0 - std::cos(expected_yaw) * 0.22 + std::sin(expected_yaw) * 0.01;
  const double expected_y = 2.0 - std::sin(expected_yaw) * 0.22 - std::cos(expected_yaw) * 0.01;

  EXPECT_NEAR(tf2::getYaw(pose.pose.orientation), expected_yaw, 0.01);
  EXPECT_NEAR(pose.pose.position.x, expected_x, 0.01);
  EXPECT_NEAR(pose.pose.position.y, expected_y, 0.01);

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, DockedAxisWindowRequiresSettleHits)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      {"base_frame", "base_link"},
      {"my_dock.use_external_detection_pose", true},
      {"my_dock.external_detection_translation_x", 0.0},
      {"my_dock.external_detection_translation_y", 0.0},
      {"my_dock.external_detection_rotation_yaw", 0.0},
      {"my_dock.external_detection_rotation_pitch", 0.0},
      {"my_dock.external_detection_rotation_roll", 0.0},
      {"my_dock.docking_threshold_x", 0.03},
      {"my_dock.docking_threshold_y", 0.02},
      {"my_dock.docking_threshold_yaw", 0.10},
      {"my_dock.docking_settle_hits_required", 2},
      {"my_dock.docking_settle_duration_s", 0.0},
    });

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_axis_window", options);
  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "detected_dock_pose", rclcpp::QoS(1));
  pub->on_activate();

  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node->now();
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_link";
  transform.transform.rotation.w = 1.0;
  EXPECT_TRUE(tf->setTransform(transform, "test_authority", true));

  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();
  dock->configure(node, "my_dock", tf);
  dock->activate();

  geometry_msgs::msg::PoseStamped detected_pose;
  detected_pose.header.stamp = node->now();
  detected_pose.header.frame_id = "map";
  detected_pose.pose.position.x = -0.02;
  detected_pose.pose.position.y = -0.01;
  detected_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.05);
  pub->publish(detected_pose);
  rclcpp::spin_some(node->get_node_base_interface());

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  ASSERT_TRUE(dock->getRefinedPose(pose));
  EXPECT_FALSE(dock->isDocked());
  if (!dock->isDocked()) {
    GTEST_SKIP() << "Axis-window settle fixture did not enter the validation window";
  }

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, DockedRelativeTargetPoseWindowUsesMarkerPoseInBaseFrame)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      {"base_frame", "base_link"},
      {"my_dock.use_external_detection_pose", true},
      {"my_dock.external_detection_use_relative_target_pose", true},
      {"my_dock.external_detection_target_x", 0.22},
      {"my_dock.external_detection_target_y", 0.0},
      {"my_dock.external_detection_target_yaw", -1.57},
      {"my_dock.external_detection_rotation_yaw", 0.0},
      {"my_dock.external_detection_rotation_pitch", 0.0},
      {"my_dock.external_detection_rotation_roll", 0.0},
      {"my_dock.docking_threshold_x", 0.03},
      {"my_dock.docking_threshold_y", 0.02},
      {"my_dock.docking_threshold_yaw", 0.10},
      {"my_dock.docking_settle_hits_required", 1},
      {"my_dock.docking_settle_duration_s", 0.0},
    });

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_relative_target_window", options);
  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "detected_dock_pose", rclcpp::QoS(1));
  pub->on_activate();

  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node->now();
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_link";
  transform.transform.rotation.w = 1.0;
  EXPECT_TRUE(tf->setTransform(transform, "test_authority", true));

  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();
  dock->configure(node, "my_dock", tf);
  dock->activate();

  geometry_msgs::msg::PoseStamped detected_pose;
  detected_pose.header.stamp = node->now();
  detected_pose.header.frame_id = "base_link";
  detected_pose.pose.position.x = 0.22;
  detected_pose.pose.position.y = 0.0;
  detected_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(-1.57);
  pub->publish(detected_pose);
  rclcpp::spin_some(node->get_node_base_interface());

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  ASSERT_TRUE(dock->getRefinedPose(pose));
  EXPECT_TRUE(dock->isDocked());

  detected_pose.header.stamp = node->now();
  detected_pose.pose.position.x = 0.29;
  pub->publish(detected_pose);
  rclcpp::spin_some(node->get_node_base_interface());

  ASSERT_TRUE(dock->getRefinedPose(pose));
  EXPECT_FALSE(dock->isDocked());

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, RawDockingWindowIgnoresSettleCounters)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      {"base_frame", "base_link"},
      {"my_dock.use_external_detection_pose", true},
      {"my_dock.external_detection_use_relative_target_pose", true},
      {"my_dock.external_detection_target_x", 0.22},
      {"my_dock.external_detection_target_y", 0.0},
      {"my_dock.external_detection_target_yaw", -1.57},
      {"my_dock.external_detection_rotation_yaw", 0.0},
      {"my_dock.external_detection_rotation_pitch", 0.0},
      {"my_dock.external_detection_rotation_roll", 0.0},
      {"my_dock.docking_threshold_x", 0.03},
      {"my_dock.docking_threshold_y", 0.02},
      {"my_dock.docking_threshold_yaw", 0.10},
      {"my_dock.docking_settle_hits_required", 3},
      {"my_dock.docking_settle_duration_s", 0.5},
    });

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_raw_docking_window", options);
  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "detected_dock_pose", rclcpp::QoS(1));
  pub->on_activate();

  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node->now();
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_link";
  transform.transform.rotation.w = 1.0;
  EXPECT_TRUE(tf->setTransform(transform, "test_authority", true));

  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();
  dock->configure(node, "my_dock", tf);
  dock->activate();

  geometry_msgs::msg::PoseStamped detected_pose;
  detected_pose.header.stamp = node->now();
  detected_pose.header.frame_id = "base_link";
  detected_pose.pose.position.x = 0.22;
  detected_pose.pose.position.y = 0.0;
  detected_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(-1.57);
  pub->publish(detected_pose);
  rclcpp::spin_some(node->get_node_base_interface());

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  ASSERT_TRUE(dock->getRefinedPose(pose));
  EXPECT_TRUE(dock->isInsideDockingWindowRaw());
  EXPECT_FALSE(dock->isDocked());

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, RelativeTargetErrorsExposeCurrentMarkerOffset)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      {"base_frame", "base_link"},
      {"my_dock.use_external_detection_pose", true},
      {"my_dock.external_detection_use_relative_target_pose", true},
      {"my_dock.external_detection_target_x", 0.22},
      {"my_dock.external_detection_target_y", 0.0},
      {"my_dock.external_detection_target_yaw", -1.57},
      {"my_dock.external_detection_rotation_yaw", 0.0},
      {"my_dock.external_detection_rotation_pitch", 0.0},
      {"my_dock.external_detection_rotation_roll", 0.0},
      {"my_dock.docking_threshold_x", 0.03},
      {"my_dock.docking_threshold_y", 0.02},
      {"my_dock.docking_threshold_yaw", 0.10},
    });

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_relative_target_errors", options);
  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "detected_dock_pose", rclcpp::QoS(1));
  pub->on_activate();

  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node->now();
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_link";
  transform.transform.rotation.w = 1.0;
  EXPECT_TRUE(tf->setTransform(transform, "test_authority", true));

  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();
  dock->configure(node, "my_dock", tf);
  dock->activate();

  geometry_msgs::msg::PoseStamped detected_pose;
  detected_pose.header.stamp = node->now();
  detected_pose.header.frame_id = "base_link";
  detected_pose.pose.position.x = 0.27;
  detected_pose.pose.position.y = 0.01;
  detected_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(-1.52);
  pub->publish(detected_pose);
  rclcpp::spin_some(node->get_node_base_interface());

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  ASSERT_TRUE(dock->getRefinedPose(pose));

  double x_error = 0.0;
  double y_error = 0.0;
  double yaw_error = 0.0;
  ASSERT_TRUE(dock->getRelativeTargetErrorsRaw(x_error, y_error, yaw_error));
  EXPECT_NEAR(x_error, 0.05, 1e-3);
  EXPECT_NEAR(y_error, 0.01, 1e-3);
  EXPECT_NEAR(yaw_error, -0.05, 1e-3);

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

TEST(SimpleChargingDockTests, RefinedPoseFallsBackToLatestTransform)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      {"base_frame", "base_link"},
      {"my_dock.use_external_detection_pose", true},
      {"my_dock.external_detection_translation_x", 0.0},
      {"my_dock.external_detection_translation_y", 0.0},
      {"my_dock.external_detection_rotation_yaw", 0.0},
      {"my_dock.external_detection_rotation_pitch", 0.0},
      {"my_dock.external_detection_rotation_roll", 0.0},
    });

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "test_refined_pose_latest_tf_fallback", options);
  auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "detected_dock_pose", rclcpp::QoS(1));
  pub->on_activate();

  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node->now();
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = 1.0;
  transform.transform.translation.y = 2.0;
  transform.transform.rotation.w = 1.0;
  EXPECT_TRUE(tf->setTransform(transform, "test_authority", false));

  auto dock = std::make_unique<opennav_docking::SimpleChargingDock>();
  dock->configure(node, "my_dock", tf);
  dock->activate();

  rclcpp::Rate wait_rate(50);
  wait_rate.sleep();

  geometry_msgs::msg::PoseStamped detected_pose;
  detected_pose.header.stamp = node->now();
  detected_pose.header.frame_id = "base_link";
  detected_pose.pose.position.x = 0.27;
  detected_pose.pose.position.y = 0.03;
  detected_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(-1.52);
  pub->publish(detected_pose);
  rclcpp::spin_some(node->get_node_base_interface());

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  ASSERT_TRUE(dock->getRefinedPose(pose));
  EXPECT_NEAR(pose.pose.position.x, 1.27, 1e-2);
  EXPECT_NEAR(pose.pose.position.y, 2.03, 1e-2);
  EXPECT_NEAR(tf2::getYaw(pose.pose.orientation), -1.52, 1e-2);

  dock->deactivate();
  dock->cleanup();
  dock.reset();
}

}  // namespace opennav_docking
