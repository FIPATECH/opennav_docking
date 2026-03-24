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

#ifndef OPENNAV_DOCKING__DOCKING_SERVER_HPP_
#define OPENNAV_DOCKING__DOCKING_SERVER_HPP_

#include <vector>
#include <memory>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav_2d_utils/odom_subscriber.hpp"
#include "opennav_docking/controller.hpp"
#include "opennav_docking/utils.hpp"
#include "opennav_docking/types.hpp"
#include "opennav_docking/dock_database.hpp"
#include "opennav_docking/navigator.hpp"
#include "opennav_docking_core/charging_dock.hpp"
#include "tf2_ros/transform_listener.h"

namespace opennav_docking
{
/**
 * @class opennav_docking::DockingServer
 * @brief An action server which implements charger docking node for AMRs
 */
class DockingServer : public nav2_util::LifecycleNode
{
public:
  using DockingActionServer = nav2_util::SimpleActionServer<DockRobot>;
  using UndockingActionServer = nav2_util::SimpleActionServer<UndockRobot>;

  /**
   * @brief A constructor for opennav_docking::DockingServer
   * @param options Additional options to control creation of the node.
   */
  explicit DockingServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief A destructor for opennav_docking::DockingServer
   */
  ~DockingServer() = default;

  /**
   * @brief Called at the conclusion of docking actions. Saves relevant
   *        docking data for later undocking action.
   */
  void stashDockData(bool use_dock_id, Dock * dock, bool successful);

  /**
   * @brief Publish feedback from a docking action.
   * @param state Current state - should be one of those defined in message.
   */
  void publishDockingFeedback(uint16_t state);

  /**
   * @brief Generate a dock from action goal
   * @param goal Action goal
   * @return Raw dock pointer to manage;
   */
  Dock * generateGoalDock(std::shared_ptr<const DockRobot::Goal> goal);

  /**
   * @brief Do initial perception, up to a timeout.
   * @param dock Dock instance, gets queried for refined pose.
   * @param dock_pose Initial dock pose, will be refined by perception.
   */
  void doInitialPerception(Dock * dock, geometry_msgs::msg::PoseStamped & dock_pose);

  /**
   * @brief Compute a staging pose from the latest refined dock pose, optionally
   * transformed into a requested frame for navigation or reset.
   * @param dock Dock instance whose plugin owns the staging geometry.
   * @param dock_pose Latest refined dock pose.
   * @param frame Target frame for the returned staging pose. If empty, the dock
   * pose frame is preserved.
   * @return Staging pose built from the refined dock pose.
   */
  geometry_msgs::msg::PoseStamped computeStagingPoseFromRefinedDockPose(
    Dock * dock, const geometry_msgs::msg::PoseStamped & dock_pose, const std::string & frame);

  /**
   * @brief If enabled, re-stage to the perception-refined staging pose before
   * entering the local docking loop.
   * @param dock Dock instance whose plugin owns the staging geometry.
   * @param dock_pose Latest refined dock pose, refreshed again if navigation occurs.
   * @param goal Current docking action goal.
   * @param staging_frame Frame to use for the staging pose.
   */
  void maybeRestageOnRefinedDockPose(
    Dock * dock, geometry_msgs::msg::PoseStamped & dock_pose,
    std::shared_ptr<const DockRobot::Goal> goal, const std::string & staging_frame);

  /**
   * @brief Check whether the current live target geometry is good enough to
   * continue into perception and refined restaging even if the initial Nav2
   * staging request did not report success.
   * @param dock Dock instance whose plugin can expose current target errors.
   * @return True if the current live target is inside the configured recovery window.
   */
  bool canContinueAfterFailedInitialStage(Dock * dock);

  /**
   * @brief Check whether the current live target geometry is already good
   * enough to hand over to the local docking controller even if Nav2 staging
   * did not report success.
   * @param dock Dock instance whose plugin can expose current target errors.
   * @return True if the current live target is inside the configured handoff window.
   */
  bool canProceedAfterFailedRefinedRestage(Dock * dock);

  /**
   * @brief Check whether the current live target is already inside a direct
   * holonomic control window right after the first perception update.
   * @param dock Dock instance whose plugin can expose current target errors.
   * @return True if refined re-staging can be skipped and the local controller
   * should take over immediately.
   */
  bool canProceedDirectlyAfterInitialPerception(Dock * dock);

  /**
   * @brief Evaluate the current live target against an explicit x/y/yaw window.
   * @param dock Dock instance whose plugin can expose current target errors.
   * @param max_x Maximum tolerated absolute x error.
   * @param max_y Maximum tolerated absolute y error.
   * @param max_yaw Maximum tolerated absolute yaw error.
   * @param label Human-readable label for logs.
   * @return True if the live target is inside the supplied window.
   */
  bool isLiveTargetInsideWindow(
    Dock * dock, double max_x, double max_y, double max_yaw, const char * label);

  /**
   * @brief Compute a local approach command from the latest refined dock pose.
   * @param dock_pose Latest refined dock pose.
   * @param command Output velocity command.
   * @return True if a valid command could be produced.
   */
  bool computeApproachCommand(
    const geometry_msgs::msg::PoseStamped & dock_pose, geometry_msgs::msg::Twist & command);

  /**
   * @brief Use control law and dock perception to approach the charge dock.
   * @param dock Dock instance, gets queried for refined pose and docked state.
   * @param dock_pose Initial dock pose, will be refined by perception.
   * @returns True if dock successfully approached, False if cancelled. For
   *          any internal error, will throw.
   */
  bool approachDock(Dock * dock, geometry_msgs::msg::PoseStamped & dock_pose);

/**
   * @brief Perform a pure rotation to dock orientation.
   * @param dock_pose The target pose that will be used to rotate.
   */
  void rotateToDock(const geometry_msgs::msg::PoseStamped & dock_pose);

  /**
   * @brief Wait for charging to begin.
   * @param dock Dock instance, used to query isCharging().
   * @returns True if charging successfully started within alloted time.
   */
  bool waitForCharge(Dock * dock, geometry_msgs::msg::PoseStamped & dock_pose);

  /**
   * @brief Reset the robot for another approach by controlling back to staging pose.
   * @param staging_pose The target pose that will reset for another approach.
   * @returns True if reset is successful.
   */
  bool resetApproach(const geometry_msgs::msg::PoseStamped & staging_pose);

  /**
   * @brief Run a single iteration of the control loop to approach a pose.
   * @param cmd The return command.
   * @param pose The pose to command towards.
   * @param linear_tolerance Pose is reached when linear distance is within this tolerance.
   * @param angular_tolerance Pose is reached when angular distance is within this tolerance.
   * @param is_docking If true, the robot is docking. If false, the robot is undocking.
   * @param backward If true, the robot will drive backwards.
   * @returns True if pose is reached.
   */
  bool getCommandToPose(
    geometry_msgs::msg::Twist & cmd, const geometry_msgs::msg::PoseStamped & pose,
    double linear_tolerance, double angular_tolerance, bool is_docking, bool backward);

  /**
   * @brief Get the robot pose (aka base_frame pose) in another frame.
   * @param frame The frame_id to get the robot pose in.
   * @returns Computed robot pose, throws TF2 error if failure.
   */
  virtual geometry_msgs::msg::PoseStamped getRobotPoseInFrame(const std::string & frame);

  /**
   * @brief Gets a preempted goal if immediately requested
   * @param Goal goal to check or replace if required with preemption
   * @param action_server Action server to check for preemptions on
   * @return SUCCESS or FAILURE
   */
  template<typename ActionT>
  void getPreemptedGoalIfRequested(
    typename std::shared_ptr<const typename ActionT::Goal> goal,
    const std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server);

  /**
   * @brief Checks and logs warning if action canceled
   * @param action_server Action server to check for cancellation on
   * @param name Name of action to put in warning message
   * @return True if action has been cancelled
   */
  template<typename ActionT>
  bool checkAndWarnIfCancelled(
    std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server,
    const std::string & name);

  /**
   * @brief Checks and logs warning if action preempted
   * @param action_server Action server to check for preemption on
   * @param name Name of action to put in warning message
   * @return True if action has been preempted
   */
  template<typename ActionT>
  bool checkAndWarnIfPreempted(
    std::unique_ptr<nav2_util::SimpleActionServer<ActionT>> & action_server,
    const std::string & name);

  /**
   * @brief Configure member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Publish zero velocity at terminal condition
   */
  void publishZeroVelocity();

  /**
   * @brief Publish a velocity command using the configured output axis conventions.
   * @param cmd Velocity command in ROS standard body-frame conventions.
   */
  void publishVelocityCommand(const geometry_msgs::msg::Twist & cmd);

protected:
  /**
   * @brief Main action callback method to complete docking request
   */
  void dockRobot();

  /**
   * @brief Main action callback method to complete undocking request
   */
  void undockRobot();

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // Mutex for dynamic parameters and dock database
  std::shared_ptr<std::mutex> mutex_;

  // Frequency to run control loops
  double controller_frequency_;
  // Timeout for initially detecting the charge dock
  double initial_perception_timeout_;
  // Timeout after making contact with dock for charging to start
  // If this is exceeded, the robot returns to the staging pose and retries
  double wait_charge_timeout_;
  // Timeout to approach into the dock and reset its approach is retrying
  double dock_approach_timeout_;
  // Timeout to rotate to the dock
  double rotate_to_dock_timeout_;
  // When undocking, these are the tolerances for arriving at the staging pose
  double undock_linear_tolerance_, undock_angular_tolerance_;
  // Maximum number of times the robot will return to staging pose and retry docking
  int max_retries_, num_retries_;
  // This is the root frame of the robot - typically "base_link"
  std::string base_frame_;
  // This is our fixed frame for controlling - typically "odom"
  std::string fixed_frame_;
  // Does the robot drive backwards onto the dock? Default is forwards
  bool dock_backwards_;
  // Whether to rotate to the dock before docking
  bool rotate_to_dock_;
  // The tolerance to the dock's staging pose not requiring navigation
  double dock_prestaging_tolerance_;
  // Tighter tolerance used after initial perception to decide whether a
  // perception-refined staging navigation is still needed.
  double refined_dock_prestaging_tolerance_;
  // Whether to re-stage on the live perceived dock pose before the final approach.
  bool restage_on_refined_detection_;
  // If the initial Nav2 staging request fails but the live target is still within
  // a broader engagement envelope, continue into perception and refined restaging
  // instead of aborting immediately.
  double initial_stage_continue_max_x_error_;
  double initial_stage_continue_max_y_error_;
  double initial_stage_continue_max_yaw_error_;
  // If Nav2 staging fails but the live target is already close enough for the
  // local docking controller, continue instead of aborting immediately.
  double refined_restaging_handoff_max_x_error_;
  double refined_restaging_handoff_max_y_error_;
  double refined_restaging_handoff_max_yaw_error_;
  // If the first perception update is already inside this window, skip refined
  // re-staging and enter the local docking controller directly.
  double direct_control_handoff_max_x_error_;
  double direct_control_handoff_max_y_error_;
  double direct_control_handoff_max_yaw_error_;
  // While waiting for charge after a transient contact, keep nudging the robot
  // locally as long as the target stays inside this broader recovery window.
  double wait_charge_reengage_max_x_error_;
  double wait_charge_reengage_max_y_error_;
  double wait_charge_reengage_max_yaw_error_;
  // Angular tolerance to exit the rotation loop when rotate_to_dock is enabled
  double rotation_angular_tolerance_;
  // Projection applied past the nominal dock pose to smooth the final approach
  double approach_target_projection_;
  // Match the local controller convention used elsewhere in the stack
  bool invert_cmd_vel_angular_z_;

  // This is a class member so it can be accessed in publish feedback
  rclcpp::Time action_start_time_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  std::unique_ptr<nav_2d_utils::OdomSubscriber> odom_sub_;

  std::unique_ptr<DockingActionServer> docking_action_server_;
  std::unique_ptr<UndockingActionServer> undocking_action_server_;

  std::unique_ptr<DockDatabase> dock_db_;
  std::unique_ptr<Navigator> navigator_;
  std::unique_ptr<Controller> controller_;
  std::string curr_dock_type_;

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
};

}  // namespace opennav_docking

#endif  // OPENNAV_DOCKING__DOCKING_SERVER_HPP_
