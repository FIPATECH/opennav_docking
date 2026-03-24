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

#include "angles/angles.h"
#include "opennav_docking/docking_server.hpp"
#include "opennav_docking/simple_charging_dock.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace opennav_docking
{

DockingServer::DockingServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("docking_server", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating %s", get_name());

  declare_parameter("controller_frequency", 50.0);
  declare_parameter("initial_perception_timeout", 5.0);
  declare_parameter("wait_charge_timeout", 5.0);
  declare_parameter("dock_approach_timeout", 30.0);
  declare_parameter("rotate_to_dock_timeout", 10.0);
  declare_parameter("undock_linear_tolerance", 0.05);
  declare_parameter("undock_angular_tolerance", 0.05);
  declare_parameter("max_retries", 3);
  declare_parameter("base_frame", "base_link");
  declare_parameter("fixed_frame", "odom");
  declare_parameter("dock_backwards", false);
  declare_parameter("dock_prestaging_tolerance", 0.5);
  declare_parameter("refined_dock_prestaging_tolerance", 0.1);
  declare_parameter("restage_on_refined_detection", true);
  declare_parameter("initial_stage_continue_max_x_error", 0.24);
  declare_parameter("initial_stage_continue_max_y_error", 0.05);
  declare_parameter("initial_stage_continue_max_yaw_error", 0.18);
  declare_parameter("refined_restaging_handoff_max_x_error", 0.08);
  declare_parameter("refined_restaging_handoff_max_y_error", 0.03);
  declare_parameter("refined_restaging_handoff_max_yaw_error", 0.12);
  declare_parameter("direct_control_handoff_max_x_error", 0.34);
  declare_parameter("direct_control_handoff_max_y_error", 0.03);
  declare_parameter("direct_control_handoff_max_yaw_error", 0.12);
  declare_parameter("wait_charge_reengage_max_x_error", 0.30);
  declare_parameter("wait_charge_reengage_max_y_error", 0.06);
  declare_parameter("wait_charge_reengage_max_yaw_error", 0.25);
  declare_parameter("odom_topic", "odom");
  declare_parameter("rotation_angular_tolerance", 0.05);
  declare_parameter("rotate_to_dock", false);
  declare_parameter("approach_target_projection", 0.25);
  declare_parameter("invert_cmd_vel_angular_z", false);
}

nav2_util::CallbackReturn
DockingServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring %s", get_name());
  auto node = shared_from_this();

  get_parameter("controller_frequency", controller_frequency_);
  get_parameter("initial_perception_timeout", initial_perception_timeout_);
  get_parameter("wait_charge_timeout", wait_charge_timeout_);
  get_parameter("dock_approach_timeout", dock_approach_timeout_);
  get_parameter("rotate_to_dock_timeout", rotate_to_dock_timeout_);
  get_parameter("undock_linear_tolerance", undock_linear_tolerance_);
  get_parameter("undock_angular_tolerance", undock_angular_tolerance_);
  get_parameter("max_retries", max_retries_);
  get_parameter("base_frame", base_frame_);
  get_parameter("fixed_frame", fixed_frame_);
  get_parameter("dock_backwards", dock_backwards_);
  get_parameter("dock_prestaging_tolerance", dock_prestaging_tolerance_);
  get_parameter("refined_dock_prestaging_tolerance", refined_dock_prestaging_tolerance_);
  get_parameter("restage_on_refined_detection", restage_on_refined_detection_);
  get_parameter("initial_stage_continue_max_x_error", initial_stage_continue_max_x_error_);
  get_parameter("initial_stage_continue_max_y_error", initial_stage_continue_max_y_error_);
  get_parameter("initial_stage_continue_max_yaw_error", initial_stage_continue_max_yaw_error_);
  get_parameter("refined_restaging_handoff_max_x_error", refined_restaging_handoff_max_x_error_);
  get_parameter("refined_restaging_handoff_max_y_error", refined_restaging_handoff_max_y_error_);
  get_parameter(
    "refined_restaging_handoff_max_yaw_error", refined_restaging_handoff_max_yaw_error_);
  get_parameter("direct_control_handoff_max_x_error", direct_control_handoff_max_x_error_);
  get_parameter("direct_control_handoff_max_y_error", direct_control_handoff_max_y_error_);
  get_parameter("direct_control_handoff_max_yaw_error", direct_control_handoff_max_yaw_error_);
  get_parameter("wait_charge_reengage_max_x_error", wait_charge_reengage_max_x_error_);
  get_parameter("wait_charge_reengage_max_y_error", wait_charge_reengage_max_y_error_);
  get_parameter("wait_charge_reengage_max_yaw_error", wait_charge_reengage_max_yaw_error_);
  get_parameter("rotation_angular_tolerance", rotation_angular_tolerance_);
  get_parameter("approach_target_projection", approach_target_projection_);
  get_parameter("invert_cmd_vel_angular_z", invert_cmd_vel_angular_z_);

  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);
  RCLCPP_INFO(
    get_logger(), "Docking cmd_vel angular.z mode: %s",
    invert_cmd_vel_angular_z_ ? "inverted" : "passthrough");
  RCLCPP_INFO(
    get_logger(), "Dock charge confirmation timeout=%.3f",
    wait_charge_timeout_);
  RCLCPP_INFO(
    get_logger(), "Dock refined re-staging: %s (tol=%.3f)",
    restage_on_refined_detection_ ? "enabled" : "disabled",
    refined_dock_prestaging_tolerance_);
  RCLCPP_INFO(
    get_logger(),
    "Dock initial-stage recovery window: |x|<=%.3f |y|<=%.3f |yaw|<=%.3f",
    initial_stage_continue_max_x_error_,
    initial_stage_continue_max_y_error_,
    initial_stage_continue_max_yaw_error_);
  RCLCPP_INFO(
    get_logger(),
    "Dock refined handoff window: |x|<=%.3f |y|<=%.3f |yaw|<=%.3f",
    refined_restaging_handoff_max_x_error_,
    refined_restaging_handoff_max_y_error_,
    refined_restaging_handoff_max_yaw_error_);
  RCLCPP_INFO(
    get_logger(),
    "Dock direct-control handoff window: |x|<=%.3f |y|<=%.3f |yaw|<=%.3f",
    direct_control_handoff_max_x_error_,
    direct_control_handoff_max_y_error_,
    direct_control_handoff_max_yaw_error_);
  RCLCPP_INFO(
    get_logger(),
    "Dock wait-charge re-engage window: |x|<=%.3f |y|<=%.3f |yaw|<=%.3f",
    wait_charge_reengage_max_x_error_,
    wait_charge_reengage_max_y_error_,
    wait_charge_reengage_max_yaw_error_);

  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  // Create odom subscriber for backward blind docking
  std::string odom_topic;
  get_parameter("odom_topic", odom_topic);
  odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(node, odom_topic);

  get_parameter("rotate_to_dock", rotate_to_dock_);
  if (rotate_to_dock_ && !dock_backwards_) {
    throw std::runtime_error{"Parameter rotate_to_dock is enabled but dock_backwards is not set."
            "Please set dock_backwards to true."};
  }

  double action_server_result_timeout;
  nav2_util::declare_parameter_if_not_declared(
    node, "action_server_result_timeout", rclcpp::ParameterValue(10.0));
  get_parameter("action_server_result_timeout", action_server_result_timeout);
  rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds = RCL_S_TO_NS(action_server_result_timeout);

  // Create the action servers for dock / undock
  docking_action_server_ = std::make_unique<DockingActionServer>(
    node, "dock_robot",
    std::bind(&DockingServer::dockRobot, this),
    nullptr, std::chrono::milliseconds(500),
    true, server_options);

  undocking_action_server_ = std::make_unique<UndockingActionServer>(
    node, "undock_robot",
    std::bind(&DockingServer::undockRobot, this),
    nullptr, std::chrono::milliseconds(500),
    true, server_options);

  // Create composed utilities
  mutex_ = std::make_shared<std::mutex>();
  controller_ = std::make_unique<Controller>(node, tf2_buffer_, fixed_frame_, base_frame_);
  navigator_ = std::make_unique<Navigator>(node);
  dock_db_ = std::make_unique<DockDatabase>(mutex_);
  if (!dock_db_->initialize(node, tf2_buffer_)) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating %s", get_name());

  auto node = shared_from_this();

  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);
  dock_db_->activate();
  navigator_->activate();
  vel_publisher_->on_activate();
  docking_action_server_->activate();
  undocking_action_server_->activate();
  curr_dock_type_.clear();

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&DockingServer::dynamicParametersCallback, this, _1));

  // Create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating %s", get_name());

  docking_action_server_->deactivate();
  undocking_action_server_->deactivate();
  dock_db_->deactivate();
  navigator_->deactivate();
  vel_publisher_->on_deactivate();

  dyn_params_handler_.reset();
  tf2_listener_.reset();

  // Destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up %s", get_name());
  tf2_buffer_.reset();
  docking_action_server_.reset();
  undocking_action_server_.reset();
  dock_db_.reset();
  navigator_.reset();
  curr_dock_type_.clear();
  controller_.reset();
  vel_publisher_.reset();
  odom_sub_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DockingServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down %s", get_name());
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename ActionT>
void DockingServer::getPreemptedGoalIfRequested(
  typename std::shared_ptr<const typename ActionT::Goal> goal,
  const std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

template<typename ActionT>
bool DockingServer::checkAndWarnIfCancelled(
  std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server,
  const std::string & name)
{
  if (action_server->is_cancel_requested()) {
    RCLCPP_WARN(get_logger(), "Goal was cancelled. Cancelling %s action", name.c_str());
    return true;
  }
  return false;
}

template<typename ActionT>
bool DockingServer::checkAndWarnIfPreempted(
  std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server,
  const std::string & name)
{
  if (action_server->is_preempt_requested()) {
    RCLCPP_WARN(get_logger(), "Goal was preempted. Cancelling %s action", name.c_str());
    return true;
  }
  return false;
}

void DockingServer::dockRobot()
{
  std::lock_guard<std::mutex> lock(*mutex_);
  action_start_time_ = this->now();
  rclcpp::Rate loop_rate(controller_frequency_);

  auto goal = docking_action_server_->get_current_goal();
  auto result = std::make_shared<DockRobot::Result>();
  result->success = false;

  if (!docking_action_server_ || !docking_action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return;
  }

  if (checkAndWarnIfCancelled(docking_action_server_, "dock_robot")) {
    docking_action_server_->terminate_all();
    return;
  }

  getPreemptedGoalIfRequested(goal, docking_action_server_);
  Dock * dock{nullptr};
  num_retries_ = 0;

  try {
    // Get dock (instance and plugin information) from request
    if (goal->use_dock_id) {
      RCLCPP_INFO(
        get_logger(),
        "Attempting to dock robot at charger %s.", goal->dock_id.c_str());
      dock = dock_db_->findDock(goal->dock_id);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Attempting to dock robot at charger at position (%0.2f, %0.2f).",
        goal->dock_pose.pose.position.x, goal->dock_pose.pose.position.y);
      dock = generateGoalDock(goal);
    }

    // Send robot to its staging pose
    publishDockingFeedback(DockRobot::Feedback::NAV_TO_STAGING_POSE);
    const auto initial_staging_pose = dock->getStagingPose();
    const auto robot_pose = getRobotPoseInFrame(initial_staging_pose.header.frame_id);
    const bool skip_initial_stage_from_live_target =
      goal->navigate_to_staging_pose && canProceedAfterFailedRefinedRestage(dock);
    if (skip_initial_stage_from_live_target) {
      RCLCPP_INFO(
        get_logger(),
        "Skipping initial staging because the live target is already inside the refined "
        "handoff window");
    } else if (
      !goal->navigate_to_staging_pose ||
      utils::l2Norm(robot_pose.pose, initial_staging_pose.pose) < dock_prestaging_tolerance_)
    {
      RCLCPP_INFO(get_logger(), "Robot already within pre-staging pose tolerance for dock");
    } else {
      try {
        navigator_->goToPose(
          initial_staging_pose, rclcpp::Duration::from_seconds(goal->max_staging_time));
        RCLCPP_INFO(get_logger(), "Successful navigation to staging pose");
      } catch (const opennav_docking_core::FailedToStage &) {
        if (canProceedAfterFailedRefinedRestage(dock)) {
          RCLCPP_WARN(
            get_logger(),
            "Initial staging navigation did not report success, but live target is already "
            "inside the docking handoff window; continuing to perception");
        } else if (canContinueAfterFailedInitialStage(dock)) {
          RCLCPP_WARN(
            get_logger(),
            "Initial staging navigation did not report success, but live target is "
            "inside the recovery window; continuing to perception and refined re-staging");
        } else {
          throw;
        }
      }
    }

    // Construct initial estimate of where the dock is located in fixed_frame
    auto dock_pose = utils::getDockPoseStamped(dock, rclcpp::Time(0));
    tf2_buffer_->transform(dock_pose, dock_pose, fixed_frame_);

    // Get initial detection of dock before proceeding to move.
    doInitialPerception(dock, dock_pose);
    RCLCPP_INFO(get_logger(), "Successful initial dock detection");
    maybeRestageOnRefinedDockPose(dock, dock_pose, goal, initial_staging_pose.header.frame_id);

    // If we performed a rotation before docking backward, we must rotate the staging pose
    // to match the robot orientation.
    auto staging_pose = computeStagingPoseFromRefinedDockPose(
      dock, dock_pose, initial_staging_pose.header.frame_id);
    if (rotate_to_dock_) {
      staging_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
        tf2::getYaw(staging_pose.pose.orientation) + M_PI);
    }

    // Docking control loop: while not docked, run controller
    rclcpp::Time dock_contact_time;
    while (rclcpp::ok()) {
      try {
        // Perform a 180º to face away from the dock if needed
        if (rotate_to_dock_) {
          rotateToDock(dock_pose);
        }
        // Approach the dock using control law
        if (approachDock(dock, dock_pose)) {
          // We are docked, wait for charging to begin
          RCLCPP_INFO(get_logger(), "Made contact with dock, waiting for charge to start");
          if (waitForCharge(dock, dock_pose)) {
            RCLCPP_INFO(get_logger(), "Robot is charging!");
            result->success = true;
            result->num_retries = num_retries_;
            stashDockData(goal->use_dock_id, dock, true);
            publishZeroVelocity();
            docking_action_server_->succeeded_current(result);
            return;
          }
        }

        // Cancelled, preempted, or shutting down (recoverable errors throw DockingException)
        stashDockData(goal->use_dock_id, dock, false);
        publishZeroVelocity();
        docking_action_server_->terminate_all(result);
        return;
      } catch (opennav_docking_core::DockingException & e) {
        if (++num_retries_ > max_retries_) {
          RCLCPP_ERROR(get_logger(), "Failed to dock, all retries have been used");
          throw;
        }
        RCLCPP_WARN(get_logger(), "Docking failed, will retry: %s", e.what());
      }

      // Reset to the latest perception-refined staging pose to try again.
      staging_pose = computeStagingPoseFromRefinedDockPose(
        dock, dock_pose, initial_staging_pose.header.frame_id);
      if (rotate_to_dock_) {
        staging_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
          tf2::getYaw(staging_pose.pose.orientation) + M_PI);
      }
      if (!resetApproach(staging_pose)) {
        // Cancelled, preempted, or shutting down
        stashDockData(goal->use_dock_id, dock, false);
        publishZeroVelocity();
        docking_action_server_->terminate_all(result);
        return;
      }
      RCLCPP_INFO(get_logger(), "Returned to staging pose, attempting docking again");
    }
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(get_logger(), "Transform error: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (opennav_docking_core::DockNotInDB & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::DOCK_NOT_IN_DB;
  } catch (opennav_docking_core::DockNotValid & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::DOCK_NOT_VALID;
  } catch (opennav_docking_core::FailedToStage & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_STAGE;
  } catch (opennav_docking_core::FailedToDetectDock & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_DETECT_DOCK;
  } catch (opennav_docking_core::FailedToControl & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_CONTROL;
  } catch (opennav_docking_core::FailedToCharge & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_CHARGE;
  } catch (opennav_docking_core::DockingException & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  }

  // Store dock state for later undocking and delete temp dock, if applicable
  stashDockData(goal->use_dock_id, dock, false);
  result->num_retries = num_retries_;
  publishZeroVelocity();
  docking_action_server_->terminate_current(result);
}

void DockingServer::stashDockData(bool use_dock_id, Dock * dock, bool successful)
{
  if (dock && successful) {
    curr_dock_type_ = dock->type;
  }

  if (!use_dock_id && dock) {
    delete dock;
    dock = nullptr;
  }
}

Dock * DockingServer::generateGoalDock(std::shared_ptr<const DockRobot::Goal> goal)
{
  auto dock = new Dock();
  dock->frame = goal->dock_pose.header.frame_id;
  dock->pose = goal->dock_pose.pose;
  dock->type = goal->dock_type;
  dock->plugin = dock_db_->findDockPlugin(dock->type);
  return dock;
}

void DockingServer::doInitialPerception(Dock * dock, geometry_msgs::msg::PoseStamped & dock_pose)
{
  publishDockingFeedback(DockRobot::Feedback::INITIAL_PERCEPTION);
  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(initial_perception_timeout_);
  while (!dock->plugin->getRefinedPose(dock_pose)) {
    if (this->now() - start > timeout) {
      throw opennav_docking_core::FailedToDetectDock("Failed initial dock detection");
    }

    if (checkAndWarnIfCancelled(docking_action_server_, "dock_robot") ||
      checkAndWarnIfPreempted(docking_action_server_, "dock_robot"))
    {
      return;
    }

    loop_rate.sleep();
  }
}

geometry_msgs::msg::PoseStamped DockingServer::computeStagingPoseFromRefinedDockPose(
  Dock * dock, const geometry_msgs::msg::PoseStamped & dock_pose, const std::string & frame)
{
  auto staging_source = dock_pose;
  const std::string target_frame = frame.empty() ? dock_pose.header.frame_id : frame;
  if (staging_source.header.frame_id != target_frame) {
    tf2_buffer_->transform(staging_source, staging_source, target_frame);
  }
  return dock->plugin->getStagingPose(staging_source.pose, staging_source.header.frame_id);
}

void DockingServer::maybeRestageOnRefinedDockPose(
  Dock * dock, geometry_msgs::msg::PoseStamped & dock_pose,
  std::shared_ptr<const DockRobot::Goal> goal, const std::string & staging_frame)
{
  if (!goal->navigate_to_staging_pose || !restage_on_refined_detection_) {
    return;
  }

  auto simple_dock = std::dynamic_pointer_cast<SimpleChargingDock>(dock->plugin);
  if (simple_dock && simple_dock->isInsideDockingWindowRaw()) {
    RCLCPP_INFO(
      get_logger(),
      "Skipping perception-refined staging because the latest target is already inside "
      "the raw docking window");
    return;
  }

  if (canProceedDirectlyAfterInitialPerception(dock)) {
    RCLCPP_INFO(
      get_logger(),
      "Skipping perception-refined staging because the latest target is already inside "
      "the direct holonomic control window");
    return;
  }

  auto refined_staging_pose = computeStagingPoseFromRefinedDockPose(dock, dock_pose, staging_frame);
  const auto robot_pose = getRobotPoseInFrame(refined_staging_pose.header.frame_id);
  const double refined_staging_error =
    utils::l2Norm(robot_pose.pose, refined_staging_pose.pose);
  if (refined_staging_error < refined_dock_prestaging_tolerance_) {
    RCLCPP_INFO(
      get_logger(),
      "Robot already within refined pre-staging tolerance for dock (error=%.3f < %.3f)",
      refined_staging_error, refined_dock_prestaging_tolerance_);
    return;
  }

  publishDockingFeedback(DockRobot::Feedback::NAV_TO_STAGING_POSE);
  try {
    navigator_->goToPose(
      refined_staging_pose, rclcpp::Duration::from_seconds(goal->max_staging_time));
    RCLCPP_INFO(
      get_logger(),
      "Successful navigation to perception-refined staging pose (error=%.3f)",
      refined_staging_error);
    doInitialPerception(dock, dock_pose);
    RCLCPP_INFO(get_logger(), "Successful dock re-detection after refined staging");
  } catch (const opennav_docking_core::FailedToStage &) {
    doInitialPerception(dock, dock_pose);
    if (canProceedDirectlyAfterInitialPerception(dock)) {
      RCLCPP_WARN(
        get_logger(),
        "Refined staging navigation did not report success, but live target is already "
        "inside the direct holonomic control window; continuing to local docking control");
      return;
    }
    if (canProceedAfterFailedRefinedRestage(dock)) {
      RCLCPP_WARN(
        get_logger(),
        "Refined staging navigation did not report success, but live target is already "
        "inside the docking handoff window; continuing to local docking control");
      return;
    }
    throw;
  }
}

bool DockingServer::canProceedAfterFailedRefinedRestage(Dock * dock)
{
  return isLiveTargetInsideWindow(
    dock,
    refined_restaging_handoff_max_x_error_,
    refined_restaging_handoff_max_y_error_,
    refined_restaging_handoff_max_yaw_error_,
    "Refined staging failure handoff");
}

bool DockingServer::canProceedDirectlyAfterInitialPerception(Dock * dock)
{
  return isLiveTargetInsideWindow(
    dock,
    direct_control_handoff_max_x_error_,
    direct_control_handoff_max_y_error_,
    direct_control_handoff_max_yaw_error_,
    "Direct-control handoff");
}

bool DockingServer::canContinueAfterFailedInitialStage(Dock * dock)
{
  return isLiveTargetInsideWindow(
    dock,
    initial_stage_continue_max_x_error_,
    initial_stage_continue_max_y_error_,
    initial_stage_continue_max_yaw_error_,
    "Initial staging failure recovery");
}

bool DockingServer::computeApproachCommand(
  const geometry_msgs::msg::PoseStamped & dock_pose, geometry_msgs::msg::Twist & command)
{
  // Transform target_pose into base_link frame
  geometry_msgs::msg::PoseStamped target_pose = dock_pose;
  target_pose.header.stamp = rclcpp::Time(0);

  // The control law can get jittery when close to the end when atan2's can explode.
  // Thus, we backward project the controller's target pose a little bit after the
  // dock so that the robot never gets to the end of the spiral before its in contact
  // with the dock to stop the docking procedure.
  const double backward_projection = approach_target_projection_;
  const double yaw = tf2::getYaw(target_pose.pose.orientation);
  target_pose.pose.position.x += cos(yaw) * backward_projection;
  target_pose.pose.position.y += sin(yaw) * backward_projection;
  tf2_buffer_->transform(target_pose, target_pose, base_frame_);

  // Make sure that the target pose is pointing at the robot when moving backwards.
  if (dock_backwards_) {
    target_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
      tf2::getYaw(target_pose.pose.orientation) + M_PI);
  }

  return controller_->computeVelocityCommand(target_pose.pose, command, true, dock_backwards_);
}

bool DockingServer::isLiveTargetInsideWindow(
  Dock * dock, double max_x, double max_y, double max_yaw, const char * label)
{
  auto simple_dock = std::dynamic_pointer_cast<SimpleChargingDock>(dock->plugin);
  if (!simple_dock) {
    return false;
  }

  double x_error = 0.0;
  double y_error = 0.0;
  double yaw_error = 0.0;
  if (!simple_dock->getRelativeTargetErrorsRaw(x_error, y_error, yaw_error)) {
    return false;
  }

  const bool inside_window =
    std::abs(x_error) <= max_x &&
    std::abs(y_error) <= max_y &&
    std::abs(yaw_error) <= max_yaw;

  RCLCPP_INFO(
    get_logger(),
    "%s check: x=%.3f y=%.3f yaw=%.3f limits=(%.3f, %.3f, %.3f) -> %s",
    label, x_error, y_error, yaw_error, max_x, max_y, max_yaw,
    inside_window ? "proceed" : "abort");
  return inside_window;
}

void DockingServer::rotateToDock(const geometry_msgs::msg::PoseStamped & dock_pose)
{
  const double dt = 1.0 / controller_frequency_;
  auto target_pose = dock_pose;
  target_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
    tf2::getYaw(target_pose.pose.orientation) + M_PI);

  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(rotate_to_dock_timeout_);

  while (rclcpp::ok()) {
    auto robot_pose = getRobotPoseInFrame(dock_pose.header.frame_id);
    auto angular_distance_to_heading = angles::shortest_angular_distance(
      tf2::getYaw(robot_pose.pose.orientation), tf2::getYaw(target_pose.pose.orientation));
    if (fabs(angular_distance_to_heading) < rotation_angular_tolerance_) {
      break;
    }

    geometry_msgs::msg::Twist current_vel;
    current_vel.angular.z = odom_sub_->getTwist().theta;

    auto command = controller_->computeRotateToHeadingCommand(
      angular_distance_to_heading, current_vel, dt);

    publishVelocityCommand(command);

    if (this->now() - start > timeout) {
      throw opennav_docking_core::FailedToControl("Timed out rotating to dock");
    }

    loop_rate.sleep();
  }
}

bool DockingServer::approachDock(Dock * dock, geometry_msgs::msg::PoseStamped & dock_pose)
{
  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(dock_approach_timeout_);
  while (rclcpp::ok()) {
    publishDockingFeedback(DockRobot::Feedback::CONTROLLING);

    // Stop if cancelled/preempted
    if (checkAndWarnIfCancelled(docking_action_server_, "dock_robot") ||
      checkAndWarnIfPreempted(docking_action_server_, "dock_robot"))
    {
      return false;
    }

    // Update perception
    if (!dock->plugin->getRefinedPose(dock_pose) && !rotate_to_dock_) {
      throw opennav_docking_core::FailedToDetectDock("Failed dock detection");
    }

    // Evaluate the success window using the freshest perception update.
    if (dock->plugin->isDocked() || dock->plugin->isCharging()) {
      return true;
    }

    // Compute and publish controls
    geometry_msgs::msg::Twist command;
    if (!computeApproachCommand(dock_pose, command)) {
      throw opennav_docking_core::FailedToControl("Failed to get control");
    }
    publishVelocityCommand(command);

    if (this->now() - start > timeout) {
      throw opennav_docking_core::FailedToControl(
              "Timed out approaching dock; dock nor charging detected");
    }

    loop_rate.sleep();
  }
  return false;
}

bool DockingServer::waitForCharge(Dock * dock, geometry_msgs::msg::PoseStamped & dock_pose)
{
  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(wait_charge_timeout_);
  while (rclcpp::ok()) {
    publishDockingFeedback(DockRobot::Feedback::WAIT_FOR_CHARGE);

    if (dock->plugin->isCharging()) {
      return true;
    }

    auto simple_dock = std::dynamic_pointer_cast<SimpleChargingDock>(dock->plugin);
    if (simple_dock && dock->plugin->getRefinedPose(dock_pose)) {
      if (!simple_dock->isInsideDockingWindowRaw()) {
        if (!isLiveTargetInsideWindow(
            dock,
            wait_charge_reengage_max_x_error_,
            wait_charge_reengage_max_y_error_,
            wait_charge_reengage_max_yaw_error_,
            "Wait-charge re-engage"))
        {
          throw opennav_docking_core::FailedToCharge(
                  "Lost the docking window while waiting for charge");
        }

        geometry_msgs::msg::Twist command;
        if (!computeApproachCommand(dock_pose, command)) {
          throw opennav_docking_core::FailedToControl(
                  "Failed to get control while waiting for charge");
        }
        publishVelocityCommand(command);
      } else {
        publishZeroVelocity();
      }
    }

    if (checkAndWarnIfCancelled(docking_action_server_, "dock_robot") ||
      checkAndWarnIfPreempted(docking_action_server_, "dock_robot"))
    {
      return false;
    }

    if (this->now() - start > timeout) {
      throw opennav_docking_core::FailedToCharge("Timed out waiting for charge to start");
    }

    loop_rate.sleep();
  }
  return false;
}

bool DockingServer::resetApproach(const geometry_msgs::msg::PoseStamped & staging_pose)
{
  rclcpp::Rate loop_rate(controller_frequency_);
  auto start = this->now();
  auto timeout = rclcpp::Duration::from_seconds(dock_approach_timeout_);
  while (rclcpp::ok()) {
    publishDockingFeedback(DockRobot::Feedback::INITIAL_PERCEPTION);

    // Stop if cancelled/preempted
    if (checkAndWarnIfCancelled(docking_action_server_, "dock_robot") ||
      checkAndWarnIfPreempted(docking_action_server_, "dock_robot"))
    {
      return false;
    }

    // Compute and publish command
    geometry_msgs::msg::Twist command;
    if (getCommandToPose(
        command, staging_pose, undock_linear_tolerance_, undock_angular_tolerance_, false,
        !dock_backwards_))
    {
      return true;
    }
    publishVelocityCommand(command);

    if (this->now() - start > timeout) {
      throw opennav_docking_core::FailedToControl("Timed out resetting dock approach");
    }

    loop_rate.sleep();
  }
  return false;
}

bool DockingServer::getCommandToPose(
  geometry_msgs::msg::Twist & cmd, const geometry_msgs::msg::PoseStamped & pose,
  double linear_tolerance, double angular_tolerance, bool is_docking, bool backward)
{
  // Reset command to zero velocity
  cmd.linear.x = 0;
  cmd.angular.z = 0;

  // Determine if we have reached pose yet & stop
  geometry_msgs::msg::PoseStamped robot_pose = getRobotPoseInFrame(pose.header.frame_id);
  const double dist = std::hypot(
    robot_pose.pose.position.x - pose.pose.position.x,
    robot_pose.pose.position.y - pose.pose.position.y);
  const double yaw = angles::shortest_angular_distance(
    tf2::getYaw(robot_pose.pose.orientation), tf2::getYaw(pose.pose.orientation));
  if (dist < linear_tolerance && abs(yaw) < angular_tolerance) {
    return true;
  }

  // Transform target_pose into base_link frame
  geometry_msgs::msg::PoseStamped target_pose = pose;
  target_pose.header.stamp = rclcpp::Time(0);
  tf2_buffer_->transform(target_pose, target_pose, base_frame_);

  // Compute velocity command
  if (!controller_->computeVelocityCommand(target_pose.pose, cmd, is_docking, backward)) {
    throw opennav_docking_core::FailedToControl("Failed to get control");
  }

  // Command is valid, but target is not reached
  return false;
}

void DockingServer::undockRobot()
{
  std::lock_guard<std::mutex> lock(*mutex_);
  action_start_time_ = this->now();
  rclcpp::Rate loop_rate(controller_frequency_);

  auto goal = undocking_action_server_->get_current_goal();
  auto result = std::make_shared<UndockRobot::Result>();
  result->success = false;

  if (!undocking_action_server_ || !undocking_action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return;
  }

  if (checkAndWarnIfCancelled(undocking_action_server_, "undock_robot")) {
    undocking_action_server_->terminate_all(result);
    return;
  }

  getPreemptedGoalIfRequested(goal, undocking_action_server_);
  auto max_duration = rclcpp::Duration::from_seconds(goal->max_undocking_time);

  try {
    // Get dock plugin information from request or docked state, reset state.
    std::string dock_type = curr_dock_type_;
    if (!goal->dock_type.empty()) {
      dock_type = goal->dock_type;
    }

    ChargingDock::Ptr dock = dock_db_->findDockPlugin(dock_type);
    if (!dock) {
      throw opennav_docking_core::DockNotValid("No dock information to undock from!");
    }
    RCLCPP_INFO(
      get_logger(),
      "Attempting to undock robot from charger of type %s.", dock->getName().c_str());

    // Get "dock pose" by finding the robot pose
    geometry_msgs::msg::PoseStamped dock_pose = getRobotPoseInFrame(fixed_frame_);

    // Make sure that the staging pose is pointing in the same direction when moving backwards
    if (dock_backwards_) {
      dock_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
        tf2::getYaw(dock_pose.pose.orientation) + M_PI);
    }

    // Get staging pose (in fixed frame)
    geometry_msgs::msg::PoseStamped staging_pose =
      dock->getStagingPose(dock_pose.pose, dock_pose.header.frame_id);

    // If we performed a rotation before docking backward, we must rotate the staging pose
    // to match the robot orientation
    if (rotate_to_dock_) {
      staging_pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
        tf2::getYaw(staging_pose.pose.orientation) + M_PI);
    }

    // Control robot to staging pose
    rclcpp::Time loop_start = this->now();
    while (rclcpp::ok()) {
      // Stop if we exceed max duration
      auto timeout = rclcpp::Duration::from_seconds(goal->max_undocking_time);
      if (this->now() - loop_start > timeout) {
        throw opennav_docking_core::FailedToControl("Undocking timed out");
      }

      // Stop if cancelled/preempted
      if (checkAndWarnIfCancelled(undocking_action_server_, "undock_robot") ||
        checkAndWarnIfPreempted(undocking_action_server_, "undock_robot"))
      {
        publishZeroVelocity();
        undocking_action_server_->terminate_all(result);
        return;
      }

      // Don't control the robot until charging is disabled
      if (!dock->disableCharging()) {
        loop_rate.sleep();
        continue;
      }

      // Get command to approach staging pose
      geometry_msgs::msg::Twist command;
      if (getCommandToPose(
          command, staging_pose, undock_linear_tolerance_, undock_angular_tolerance_, false,
          !dock_backwards_))
      {
        // Perform a 180º to the original staging pose
        if (rotate_to_dock_) {
          rotateToDock(staging_pose);
        }

        // Have reached staging_pose
        RCLCPP_INFO(get_logger(), "Robot has reached staging pose");
        publishVelocityCommand(command);
        if (dock->hasStoppedCharging()) {
          RCLCPP_INFO(get_logger(), "Robot has undocked!");
          result->success = true;
          curr_dock_type_.clear();
          publishZeroVelocity();
          undocking_action_server_->succeeded_current(result);
          return;
        }
        // Haven't stopped charging?
        throw opennav_docking_core::FailedToControl("Failed to control off dock, still charging");
      }

      // Publish command and sleep
      publishVelocityCommand(command);
      loop_rate.sleep();
    }
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(get_logger(), "Transform error: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (opennav_docking_core::DockNotValid & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::DOCK_NOT_VALID;
  } catch (opennav_docking_core::FailedToControl & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::FAILED_TO_CONTROL;
  } catch (opennav_docking_core::DockingException & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Internal error: %s", e.what());
    result->error_code = DockRobot::Result::UNKNOWN;
  }

  publishZeroVelocity();
  undocking_action_server_->terminate_current(result);
}

geometry_msgs::msg::PoseStamped DockingServer::getRobotPoseInFrame(const std::string & frame)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  robot_pose.header.frame_id = base_frame_;
  robot_pose.header.stamp = rclcpp::Time(0);
  tf2_buffer_->transform(robot_pose, robot_pose, frame);
  return robot_pose;
}

void DockingServer::publishZeroVelocity()
{
  publishVelocityCommand(geometry_msgs::msg::Twist());
}

void DockingServer::publishVelocityCommand(const geometry_msgs::msg::Twist & cmd)
{
  auto out = cmd;
  if (invert_cmd_vel_angular_z_) {
    out.angular.z = -out.angular.z;
  }
  vel_publisher_->publish(out);
}

void DockingServer::publishDockingFeedback(uint16_t state)
{
  auto feedback = std::make_shared<DockRobot::Feedback>();
  feedback->state = state;
  feedback->docking_time = this->now() - action_start_time_;
  feedback->num_retries = num_retries_;
  docking_action_server_->publish_feedback(feedback);
}

rcl_interfaces::msg::SetParametersResult
DockingServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(*mutex_);

  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "controller_frequency") {
        controller_frequency_ = parameter.as_double();
      } else if (name == "initial_perception_timeout") {
        initial_perception_timeout_ = parameter.as_double();
      } else if (name == "wait_charge_timeout") {
        wait_charge_timeout_ = parameter.as_double();
      } else if (name == "undock_linear_tolerance") {
        undock_linear_tolerance_ = parameter.as_double();
      } else if (name == "undock_angular_tolerance") {
        undock_angular_tolerance_ = parameter.as_double();
      } else if (name == "rotation_angular_tolerance") {
        rotation_angular_tolerance_ = parameter.as_double();
      } else if (name == "refined_dock_prestaging_tolerance") {
        refined_dock_prestaging_tolerance_ = parameter.as_double();
      } else if (name == "initial_stage_continue_max_x_error") {
        initial_stage_continue_max_x_error_ = parameter.as_double();
      } else if (name == "initial_stage_continue_max_y_error") {
        initial_stage_continue_max_y_error_ = parameter.as_double();
      } else if (name == "initial_stage_continue_max_yaw_error") {
        initial_stage_continue_max_yaw_error_ = parameter.as_double();
      } else if (name == "refined_restaging_handoff_max_x_error") {
        refined_restaging_handoff_max_x_error_ = parameter.as_double();
      } else if (name == "refined_restaging_handoff_max_y_error") {
        refined_restaging_handoff_max_y_error_ = parameter.as_double();
      } else if (name == "refined_restaging_handoff_max_yaw_error") {
        refined_restaging_handoff_max_yaw_error_ = parameter.as_double();
      } else if (name == "direct_control_handoff_max_x_error") {
        direct_control_handoff_max_x_error_ = parameter.as_double();
      } else if (name == "direct_control_handoff_max_y_error") {
        direct_control_handoff_max_y_error_ = parameter.as_double();
      } else if (name == "direct_control_handoff_max_yaw_error") {
        direct_control_handoff_max_yaw_error_ = parameter.as_double();
      } else if (name == "wait_charge_reengage_max_x_error") {
        wait_charge_reengage_max_x_error_ = parameter.as_double();
      } else if (name == "wait_charge_reengage_max_y_error") {
        wait_charge_reengage_max_y_error_ = parameter.as_double();
      } else if (name == "wait_charge_reengage_max_yaw_error") {
        wait_charge_reengage_max_yaw_error_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == "base_frame") {
        base_frame_ = parameter.as_string();
      } else if (name == "fixed_frame") {
        fixed_frame_ = parameter.as_string();
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == "max_retries") {
        max_retries_ = parameter.as_int();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == "invert_cmd_vel_angular_z") {
        invert_cmd_vel_angular_z_ = parameter.as_bool();
      } else if (name == "restage_on_refined_detection") {
        restage_on_refined_detection_ = parameter.as_bool();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace opennav_docking

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(opennav_docking::DockingServer)
