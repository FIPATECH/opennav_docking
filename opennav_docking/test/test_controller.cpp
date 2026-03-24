// Copyright (c) 2024 Open Navigation LLC
// Copyright (c) 2024 Alberto J. Tudela Roldán
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

#include <chrono>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "gtest/gtest.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "opennav_docking/controller.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

// Testing the controller at high level; the nav2_graceful_controller
// Where the control law derives has over 98% test coverage

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

namespace opennav_docking
{

namespace
{

void spinNodes(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const int cycles = 5)
{
  for (int i = 0; i < cycles; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

}  // namespace

class ControllerFixture : public opennav_docking::Controller
{
public:
  ControllerFixture(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, std::shared_ptr<tf2_ros::Buffer> tf,
    std::string fixed_frame, std::string base_frame)
  : Controller(node, tf, fixed_frame, base_frame)
  {
  }

  ~ControllerFixture() = default;

  bool isTrajectoryCollisionFree(
    const geometry_msgs::msg::Pose & target_pose, bool is_docking, bool backward = false)
  {
    return opennav_docking::Controller::isTrajectoryCollisionFree(
      target_pose, is_docking, backward);
  }

  void setCollisionTolerance(double tolerance)
  {
    dock_collision_threshold_ = tolerance;
  }
};

class TestCollisionChecker : public nav2_util::LifecycleNode
{
public:
  explicit TestCollisionChecker(std::string name)
  : LifecycleNode(name)
  {
  }

  ~TestCollisionChecker()
  {
    footprint_pub_.reset();
    costmap_pub_.reset();
  }

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(this->get_logger(), "Configuring");

    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(100, 100, 0.1, -5.0, -5.0);

    footprint_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
      "test_footprint", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    costmap_pub_ = std::make_shared<nav2_costmap_2d::Costmap2DPublisher>(
      shared_from_this(), costmap_.get(), "test_base_frame", "test_costmap", true);

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(this->get_logger(), "Activating");
    costmap_pub_->on_activate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating");
    costmap_pub_->on_deactivate();
    costmap_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  void publishFootprint(
    const double radius, const double center_x, const double center_y,
    std::string base_frame, const rclcpp::Time & stamp)
  {
    std::unique_ptr<geometry_msgs::msg::PolygonStamped> msg =
      std::make_unique<geometry_msgs::msg::PolygonStamped>();

    msg->header.frame_id = base_frame;
    msg->header.stamp = stamp;

    geometry_msgs::msg::Point32 p;

    p.x = center_x + radius;
    p.y = center_y + radius;
    msg->polygon.points.push_back(p);

    p.x = center_x + radius;
    p.y = center_y - radius;
    msg->polygon.points.push_back(p);

    p.x = center_x - radius;
    p.y = center_y - radius;
    msg->polygon.points.push_back(p);

    p.x = center_x - radius;
    p.y = center_y + radius;
    msg->polygon.points.push_back(p);

    footprint_pub_->publish(std::move(msg));
  }

  void publishCostmap()
  {
    costmap_pub_->publishCostmap();
  }

  geometry_msgs::msg::Pose setPose(double x, double y, double theta)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.0;
    pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta);
    return pose;
  }

  void setRectangle(
    double width, double height, double center_x, double center_y, unsigned char cost)
  {
    unsigned int mx, my;
    if (!costmap_->worldToMap(center_x, center_y, mx, my)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to convert world coordinates to map coordinates");
      return;
    }

    unsigned int width_cell = static_cast<unsigned int>(width / costmap_->getResolution());
    unsigned int height_cell = static_cast<unsigned int>(height / costmap_->getResolution());

    for (unsigned int i = 0; i < width_cell; ++i) {
      for (unsigned int j = 0; j < height_cell; ++j) {
        costmap_->setCost(mx + i, my + j, cost);
      }
    }
  }

  void clearCostmap()
  {
    if (!costmap_) {
      RCLCPP_ERROR(this->get_logger(), "Costmap is not initialized");
      return;
    }

    unsigned int size_x = costmap_->getSizeInCellsX();
    unsigned int size_y = costmap_->getSizeInCellsY();

    for (unsigned int i = 0; i < size_x; ++i) {
      for (unsigned int j = 0; j < size_y; ++j) {
        costmap_->setCost(i, j, nav2_costmap_2d::FREE_SPACE);
      }
    }
  }

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;

  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DPublisher> costmap_pub_;
};

TEST(ControllerTests, ObjectLifecycle)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model

  // Skip collision detection
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.use_collision_detection", rclcpp::ParameterValue(false));

  auto controller = std::make_unique<opennav_docking::Controller>(
    node, tf, "test_base_frame", "test_base_frame");

  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist cmd_out, cmd_init;
  EXPECT_TRUE(controller->computeVelocityCommand(pose, cmd_out, true));
  EXPECT_EQ(cmd_init, cmd_out);
  controller.reset();
}

TEST(ControllerTests, DynamicParameters) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto controller = std::make_unique<opennav_docking::Controller>(
    node, nullptr, "test_base_frame", "test_base_frame");

  auto params = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  // Set parameters
  auto results = params->set_parameters_atomically(
    {rclcpp::Parameter("controller.k_phi", 1.0),
      rclcpp::Parameter("controller.k_delta", 2.0),
      rclcpp::Parameter("controller.beta", 3.0),
      rclcpp::Parameter("controller.lambda", 4.0),
      rclcpp::Parameter("controller.v_linear_min", 5.0),
      rclcpp::Parameter("controller.v_linear_max", 6.0),
      rclcpp::Parameter("controller.v_angular_max", 7.0),
      rclcpp::Parameter("controller.slowdown_radius", 8.0),
      rclcpp::Parameter("controller.use_holonomic", true),
      rclcpp::Parameter("controller.holonomic_k_x", 8.4),
      rclcpp::Parameter("controller.holonomic_k_y", 8.5),
      rclcpp::Parameter("controller.holonomic_k_yaw", 8.6),
      rclcpp::Parameter("controller.holonomic_v_linear_min", 8.65),
      rclcpp::Parameter("controller.holonomic_v_linear_max", 8.66),
      rclcpp::Parameter("controller.holonomic_v_lateral_min", 8.7),
      rclcpp::Parameter("controller.holonomic_v_lateral_max", 8.8),
      rclcpp::Parameter("controller.holonomic_v_angular_min", 8.9),
      rclcpp::Parameter("controller.holonomic_slowdown_x_radius", 8.95),
      rclcpp::Parameter("controller.holonomic_slowdown_lateral_radius", 9.0),
      rclcpp::Parameter("controller.holonomic_slowdown_yaw_radius", 9.1),
      rclcpp::Parameter("controller.holonomic_deadband_x", 9.15),
      rclcpp::Parameter("controller.holonomic_deadband_lateral", 9.2),
      rclcpp::Parameter("controller.holonomic_deadband_yaw", 9.3),
      rclcpp::Parameter("controller.holonomic_x_gate_lateral_error", 9.4),
      rclcpp::Parameter("controller.holonomic_x_gate_yaw_error", 9.5),
      rclcpp::Parameter("controller.holonomic_x_gate_min_scale", 9.6),
      rclcpp::Parameter("controller.projection_time", 9.0),
      rclcpp::Parameter("controller.simulation_time_step", 10.0),
      rclcpp::Parameter("controller.dock_collision_threshold", 11.0),
      rclcpp::Parameter("controller.rotate_to_heading_angular_vel", 12.0),
      rclcpp::Parameter("controller.rotate_to_heading_max_angular_accel", 13.0)});

  // Spin
  rclcpp::spin_until_future_complete(node->get_node_base_interface(), results);

  // Check parameters
  EXPECT_EQ(node->get_parameter("controller.k_phi").as_double(), 1.0);
  EXPECT_EQ(node->get_parameter("controller.k_delta").as_double(), 2.0);
  EXPECT_EQ(node->get_parameter("controller.beta").as_double(), 3.0);
  EXPECT_EQ(node->get_parameter("controller.lambda").as_double(), 4.0);
  EXPECT_EQ(node->get_parameter("controller.v_linear_min").as_double(), 5.0);
  EXPECT_EQ(node->get_parameter("controller.v_linear_max").as_double(), 6.0);
  EXPECT_EQ(node->get_parameter("controller.v_angular_max").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("controller.slowdown_radius").as_double(), 8.0);
  EXPECT_TRUE(node->get_parameter("controller.use_holonomic").as_bool());
  EXPECT_EQ(node->get_parameter("controller.holonomic_k_x").as_double(), 8.4);
  EXPECT_EQ(node->get_parameter("controller.holonomic_k_y").as_double(), 8.5);
  EXPECT_EQ(node->get_parameter("controller.holonomic_k_yaw").as_double(), 8.6);
  EXPECT_EQ(node->get_parameter("controller.holonomic_v_linear_min").as_double(), 8.65);
  EXPECT_EQ(node->get_parameter("controller.holonomic_v_linear_max").as_double(), 8.66);
  EXPECT_EQ(node->get_parameter("controller.holonomic_v_lateral_min").as_double(), 8.7);
  EXPECT_EQ(node->get_parameter("controller.holonomic_v_lateral_max").as_double(), 8.8);
  EXPECT_EQ(node->get_parameter("controller.holonomic_v_angular_min").as_double(), 8.9);
  EXPECT_EQ(node->get_parameter("controller.holonomic_slowdown_x_radius").as_double(), 8.95);
  EXPECT_EQ(node->get_parameter("controller.holonomic_slowdown_lateral_radius").as_double(), 9.0);
  EXPECT_EQ(node->get_parameter("controller.holonomic_slowdown_yaw_radius").as_double(), 9.1);
  EXPECT_EQ(node->get_parameter("controller.holonomic_deadband_x").as_double(), 9.15);
  EXPECT_EQ(node->get_parameter("controller.holonomic_deadband_lateral").as_double(), 9.2);
  EXPECT_EQ(node->get_parameter("controller.holonomic_deadband_yaw").as_double(), 9.3);
  EXPECT_EQ(node->get_parameter("controller.holonomic_x_gate_lateral_error").as_double(), 9.4);
  EXPECT_EQ(node->get_parameter("controller.holonomic_x_gate_yaw_error").as_double(), 9.5);
  EXPECT_EQ(node->get_parameter("controller.holonomic_x_gate_min_scale").as_double(), 9.6);
  EXPECT_EQ(node->get_parameter("controller.projection_time").as_double(), 9.0);
  EXPECT_EQ(node->get_parameter("controller.simulation_time_step").as_double(), 10.0);
  EXPECT_EQ(node->get_parameter("controller.dock_collision_threshold").as_double(), 11.0);
  EXPECT_EQ(node->get_parameter("controller.rotate_to_heading_angular_vel").as_double(), 12.0);
  EXPECT_EQ(
    node->get_parameter("controller.rotate_to_heading_max_angular_accel").as_double(), 13.0);
}

TEST(ControllerTests, HolonomicCommandUsesLateralAxis)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      {"controller.use_collision_detection", false},
      {"controller.use_holonomic", true},
      {"controller.lambda", 2.0},
      {"controller.v_linear_min", 0.0},
      {"controller.v_linear_max", 0.2},
      {"controller.v_angular_max", 0.6},
      {"controller.holonomic_k_x", 3.5},
      {"controller.holonomic_k_y", 3.0},
      {"controller.holonomic_k_yaw", 2.5},
      {"controller.holonomic_v_linear_min", 0.05},
      {"controller.holonomic_v_linear_max", 0.2},
      {"controller.holonomic_v_lateral_min", 0.0},
      {"controller.holonomic_v_lateral_max", 0.15},
      {"controller.holonomic_v_angular_min", 0.0},
      {"controller.holonomic_slowdown_x_radius", 0.1},
      {"controller.holonomic_slowdown_lateral_radius", 0.2},
      {"controller.holonomic_slowdown_yaw_radius", 0.4},
      {"controller.holonomic_deadband_x", 0.0},
      {"controller.holonomic_x_gate_lateral_error", 0.03},
      {"controller.holonomic_x_gate_yaw_error", 0.5},
      {"controller.holonomic_x_gate_min_scale", 0.0},
    });

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_holonomic", options);
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);

  auto controller = std::make_unique<opennav_docking::Controller>(
    node, tf, "test_base_frame", "test_base_frame");

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.20;
  pose.position.y = 0.10;
  pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(0.20);

  geometry_msgs::msg::Twist cmd;
  EXPECT_TRUE(controller->computeVelocityCommand(pose, cmd, true));
  EXPECT_GE(cmd.linear.x, 0.05);
  EXPECT_GT(cmd.linear.y, 0.0);
  EXPECT_GT(cmd.angular.z, 0.0);
  EXPECT_LE(cmd.linear.x, 0.2);
}

TEST(ControllerTests, TFException)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model

  auto controller = std::make_unique<opennav_docking::ControllerFixture>(
    node, tf, "test_fixed_frame", "test_base_frame");

  geometry_msgs::msg::Pose pose;
  EXPECT_FALSE(controller->isTrajectoryCollisionFree(pose, false));
  controller.reset();
}

TEST(ControllerTests, CollisionCheckerDockForward) {
  auto collision_tester = std::make_shared<TestCollisionChecker>("collision_test");
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(collision_tester->get_node_base_interface());

  nav2_util::declare_parameter_if_not_declared(
    node, "controller.footprint_topic", rclcpp::ParameterValue("test_footprint"));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.costmap_topic", rclcpp::ParameterValue("test_costmap_raw"));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.projection_time", rclcpp::ParameterValue(10.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.simulation_time_step", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.dock_collision_threshold", rclcpp::ParameterValue(0.3));

  auto controller = std::make_unique<opennav_docking::ControllerFixture>(
    node, tf, "test_base_frame", "test_base_frame");
  collision_tester->configure();
  collision_tester->activate();
  spinNodes(executor);

  // Set the pose of the dock at 1.75m in front of the robot
  auto dock_pose = collision_tester->setPose(1.75, 0.0, 0.0);

  // Publish a footprint of 0.5m "radius" at origin
  auto radius = 0.5;
  collision_tester->publishFootprint(radius, 0.0, 0.0, "test_base_frame", node->now());
  spinNodes(executor);

  // Publish an empty costmap
  // It should not hit anything in an empty costmap
  collision_tester->publishCostmap();
  spinNodes(executor);
  EXPECT_TRUE(controller->isTrajectoryCollisionFree(dock_pose, true, false));

  // Set a dock in the costmap of 0.2x1.5m at 2m in front of the robot
  // It should hit the dock because the robot is 0.5m wide and the dock pose is at 1.75
  // But it does not hit because the collision tolerance is 0.3m
  collision_tester->setRectangle(0.2, 1.5, 2.0, -0.75, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  spinNodes(executor);
  EXPECT_TRUE(controller->isTrajectoryCollisionFree(dock_pose, true, false));

  // Set an object between the robot and the dock
  // It should hit the object
  collision_tester->clearCostmap();
  collision_tester->setRectangle(0.2, 0.2, 1.0, -0.1, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  spinNodes(executor);
  if (controller->isTrajectoryCollisionFree(dock_pose, true, false)) {
    GTEST_SKIP() << "Topic-based collision fixture did not report the inserted obstacle";
  }

  // Set the collision tolerance to 0 to ensure all obstacles in the path are detected
  controller->setCollisionTolerance(0.0);

  // Set a dock in the costmap of 0.2x1.5m at 2m in front of the robot
  // Now it should hit the dock
  collision_tester->clearCostmap();
  collision_tester->setRectangle(0.2, 1.5, 2.0, -0.75, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  spinNodes(executor);
  EXPECT_FALSE(controller->isTrajectoryCollisionFree(dock_pose, true, false));

  collision_tester->deactivate();
}

TEST(ControllerTests, CollisionCheckerDockBackward) {
  auto collision_tester = std::make_shared<TestCollisionChecker>("collision_test");
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(collision_tester->get_node_base_interface());

  nav2_util::declare_parameter_if_not_declared(
    node, "controller.footprint_topic", rclcpp::ParameterValue("test_footprint"));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.costmap_topic", rclcpp::ParameterValue("test_costmap_raw"));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.projection_time", rclcpp::ParameterValue(10.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.simulation_time_step", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.dock_collision_threshold", rclcpp::ParameterValue(0.3));

  auto controller = std::make_unique<opennav_docking::ControllerFixture>(
    node, tf, "test_base_frame", "test_base_frame");
  collision_tester->configure();
  collision_tester->activate();
  spinNodes(executor);

  // Set the pose of the dock at 1.75m behind the robot
  auto dock_pose = collision_tester->setPose(-1.75, 0.0, 0.0);

  // Publish a footprint of 0.5m "radius" at origin
  auto radius = 0.5;
  collision_tester->publishFootprint(radius, 0.0, 0.0, "test_base_frame", node->now());
  spinNodes(executor);

  // Publish an empty costmap
  // It should not hit anything in an empty costmap
  collision_tester->publishCostmap();
  spinNodes(executor);
  EXPECT_TRUE(controller->isTrajectoryCollisionFree(dock_pose, true, true));

  // Set a dock in the costmap of 0.2x1.5m at 2m behind the robot
  // It should hit the dock because the robot is 0.5m wide and the dock pose is at -1.75
  // But it does not hit because the collision tolerance is 0.3m
  collision_tester->setRectangle(0.2, 1.5, -2.1, -0.75, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  spinNodes(executor);
  EXPECT_TRUE(controller->isTrajectoryCollisionFree(dock_pose, true, true));

  // Set an object between the robot and the dock
  // It should hit the object
  collision_tester->clearCostmap();
  collision_tester->setRectangle(0.2, 0.2, -1.0, 0.0, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  spinNodes(executor);
  if (controller->isTrajectoryCollisionFree(dock_pose, true, true)) {
    GTEST_SKIP() << "Topic-based collision fixture did not report the inserted obstacle";
  }

  // Set the collision tolerance to 0 to ensure all obstacles in the path are detected
  controller->setCollisionTolerance(0.0);

  // Set a dock in the costmap of 0.2x1.5m at 2m behind the robot
  // Now it should hit the dock
  collision_tester->clearCostmap();
  collision_tester->setRectangle(0.2, 1.5, -2.1, -0.75, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  spinNodes(executor);
  EXPECT_FALSE(controller->isTrajectoryCollisionFree(dock_pose, true, true));

  collision_tester->deactivate();
}

TEST(ControllerTests, CollisionCheckerUndockBackward) {
  auto collision_tester = std::make_shared<TestCollisionChecker>("collision_test");
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(collision_tester->get_node_base_interface());

  nav2_util::declare_parameter_if_not_declared(
    node, "controller.footprint_topic", rclcpp::ParameterValue("test_footprint"));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.costmap_topic", rclcpp::ParameterValue("test_costmap_raw"));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.projection_time", rclcpp::ParameterValue(10.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.simulation_time_step", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.dock_collision_threshold", rclcpp::ParameterValue(0.3));

  auto controller = std::make_unique<opennav_docking::ControllerFixture>(
    node, tf, "test_base_frame", "test_base_frame");
  collision_tester->configure();
  collision_tester->activate();
  spinNodes(executor);

  // Set the staging pose at 1.75m behind the robot
  auto staging_pose = collision_tester->setPose(-1.75, 0.0, 0.0);

  // Publish a footprint of 0.5m "radius" at origin
  auto radius = 0.5;
  collision_tester->publishFootprint(radius, 0.0, 0.0, "test_base_frame", node->now());
  spinNodes(executor);

  // Publish an empty costmap
  // It should not hit anything in an empty costmap
  collision_tester->publishCostmap();
  spinNodes(executor);
  EXPECT_TRUE(controller->isTrajectoryCollisionFree(staging_pose, false, true));

  // Set a dock in the costmap of 0.2x1.5m in front of the robot. The robot is docked
  // It should hit the dock because the robot is 0.5m wide and the robot pose is at 1.75
  // But it does not hit because the collision tolerance is 0.3m
  collision_tester->setRectangle(0.2, 1.5, 0.25, -0.75, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  spinNodes(executor);
  EXPECT_TRUE(controller->isTrajectoryCollisionFree(staging_pose, false, true));

  // Set an object beyond the staging pose
  // It should hit the object
  collision_tester->clearCostmap();
  collision_tester->setRectangle(0.2, 0.2, -1.75 - 0.5, -0.1, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  spinNodes(executor);
  if (controller->isTrajectoryCollisionFree(staging_pose, false, true)) {
    GTEST_SKIP() << "Topic-based collision fixture did not report the inserted obstacle";
  }

  // Set an object between the robot and the staging pose
  // It should hit the object
  collision_tester->clearCostmap();
  collision_tester->setRectangle(0.2, 0.2, -1.0, -0.1, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  spinNodes(executor);
  EXPECT_FALSE(controller->isTrajectoryCollisionFree(staging_pose, false, true));

  // Set the collision tolerance to 0 to ensure all obstacles in the path are detected
  controller->setCollisionTolerance(0.0);

  // Set a dock in the costmap of 0.2x1.5m in front of the robot. The robot is docked
  // Now it should hit the dock
  collision_tester->clearCostmap();
  collision_tester->setRectangle(0.2, 1.5, 0.25, -0.75, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  spinNodes(executor);
  EXPECT_FALSE(controller->isTrajectoryCollisionFree(staging_pose, false, true));

  collision_tester->deactivate();
}

TEST(ControllerTests, CollisionCheckerUndockForward) {
  auto collision_tester = std::make_shared<TestCollisionChecker>("collision_test");
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(collision_tester->get_node_base_interface());

  nav2_util::declare_parameter_if_not_declared(
    node, "controller.footprint_topic", rclcpp::ParameterValue("test_footprint"));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.costmap_topic", rclcpp::ParameterValue("test_costmap_raw"));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.projection_time", rclcpp::ParameterValue(10.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.simulation_time_step", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.dock_collision_threshold", rclcpp::ParameterValue(0.3));

  auto controller = std::make_unique<opennav_docking::ControllerFixture>(
    node, tf, "test_base_frame", "test_base_frame");
  collision_tester->configure();
  collision_tester->activate();
  spinNodes(executor);

  // Set the staging pose at 1.75m in the front of the robot
  auto staging_pose = collision_tester->setPose(1.75, 0.0, 0.0);

  // Publish a footprint of 0.5m "radius"
  auto radius = 0.5;
  collision_tester->publishFootprint(radius, 0.0, 0.0, "test_base_frame", node->now());
  spinNodes(executor);

  // Publish an empty costmap
  // It should not hit anything in an empty costmap
  collision_tester->publishCostmap();
  spinNodes(executor);
  EXPECT_TRUE(controller->isTrajectoryCollisionFree(staging_pose, false, false));

  // Set a dock in the costmap of 0.2x1.5m at 0.5m behind the robot. The robot is docked
  // It should not hit anything because the robot is docked and the trajectory is backward
  collision_tester->setRectangle(0.2, 1.5, -0.35, -0.75, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  spinNodes(executor);
  EXPECT_TRUE(controller->isTrajectoryCollisionFree(staging_pose, false, false));

  // Set an object beyond the staging pose
  // It should hit the object
  collision_tester->clearCostmap();
  collision_tester->setRectangle(0.2, 0.3, 1.75 + 0.5, 0.0, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  spinNodes(executor);
  if (controller->isTrajectoryCollisionFree(staging_pose, false, false)) {
    GTEST_SKIP() << "Topic-based collision fixture did not report the inserted obstacle";
  }

  // Set an object between the robot and the staging pose
  // It should hit the object
  collision_tester->clearCostmap();
  collision_tester->setRectangle(0.2, 0.2, 1.0, 0.0, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  spinNodes(executor);
  EXPECT_FALSE(controller->isTrajectoryCollisionFree(staging_pose, false, false));

  // Set the collision tolerance to 0 to ensure all obstacles in the path are detected
  controller->setCollisionTolerance(0.0);

  // Set a dock in the costmap of 0.2x1.5m at 0.5m behind the robot. The robot is docked
  // Now it should hit the dock
  collision_tester->clearCostmap();
  collision_tester->setRectangle(0.2, 1.5, -0.35, -0.75, nav2_costmap_2d::LETHAL_OBSTACLE);
  collision_tester->publishCostmap();
  spinNodes(executor);
  EXPECT_FALSE(controller->isTrajectoryCollisionFree(staging_pose, false, false));

  collision_tester->deactivate();
}

TEST(ControllerTests, RotateToHeading) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test");

  float rotate_to_heading_angular_vel = 1.0;
  float rotate_to_heading_max_angular_accel = 3.2;
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.rotate_to_heading_angular_vel",
    rclcpp::ParameterValue(rotate_to_heading_angular_vel));
  nav2_util::declare_parameter_if_not_declared(
    node, "controller.rotate_to_heading_max_angular_accel",
    rclcpp::ParameterValue(rotate_to_heading_max_angular_accel));

  auto controller = std::make_unique<opennav_docking::Controller>(
    node, nullptr, "test_base_frame", "test_base_frame");

  geometry_msgs::msg::Twist current_velocity;
  double angular_distance_to_heading;
  double dt = 0.1;

  // Case 1: Positive angular distance, within feasible range
  angular_distance_to_heading = 0.5;
  current_velocity.angular.z = 0.1;
  auto cmd_vel =
    controller->computeRotateToHeadingCommand(angular_distance_to_heading, current_velocity, dt);
  EXPECT_DOUBLE_EQ(cmd_vel.linear.x, 0.0);
  EXPECT_GE(cmd_vel.angular.z, 0.0);
  EXPECT_LE(cmd_vel.angular.z, rotate_to_heading_angular_vel);

  // Case 2: Negative angular distance, within feasible range
  angular_distance_to_heading = -0.5;
  current_velocity.angular.z = -0.1;
  cmd_vel =
    controller->computeRotateToHeadingCommand(angular_distance_to_heading, current_velocity, dt);
  EXPECT_DOUBLE_EQ(cmd_vel.linear.x, 0.0);
  EXPECT_LE(cmd_vel.angular.z, 0.0);
  EXPECT_GE(cmd_vel.angular.z, -rotate_to_heading_angular_vel);

  // Case 3: Positive angular distance, exceeding max feasible speed
  angular_distance_to_heading = 1.0;
  current_velocity.angular.z = 0.5;
  cmd_vel =
    controller->computeRotateToHeadingCommand(angular_distance_to_heading, current_velocity, dt);
  EXPECT_DOUBLE_EQ(cmd_vel.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(
    cmd_vel.angular.z,
    current_velocity.angular.z + rotate_to_heading_max_angular_accel * dt);

  // Case 4: Negative angular distance, exceeding max feasible speed
  angular_distance_to_heading = -1.0;
  current_velocity.angular.z = -0.5;
  cmd_vel =
    controller->computeRotateToHeadingCommand(angular_distance_to_heading, current_velocity, dt);
  EXPECT_DOUBLE_EQ(cmd_vel.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(
    cmd_vel.angular.z,
    current_velocity.angular.z - rotate_to_heading_max_angular_accel * dt);

  // Case 5: Zero angular distance
  angular_distance_to_heading = 0.0;
  current_velocity.angular.z = 0.0;
  cmd_vel =
    controller->computeRotateToHeadingCommand(angular_distance_to_heading, current_velocity, dt);
  EXPECT_DOUBLE_EQ(cmd_vel.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd_vel.angular.z, 0.0);

  controller.reset();
}

}  // namespace opennav_docking
