#pragma once

#include <sstream>
#include <typeinfo>
#include <chrono>
#include <thread>
#include <math.h>
#include <future>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include "modproft_robot_control_msgs/action/move_joint.hpp"
#include "modproft_robot_control_msgs/action/move_pose.hpp"
#include "modproft_robot_control_msgs/action/move_cartesian.hpp"
#include "modproft_robot_control_msgs/action/move_tcp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

template <typename T>
std::string vector2string(const std::vector<T> &v);

void logCurrentRobotState(moveit::planning_interface::MoveGroupInterface &move_group,
                          const rclcpp::Logger &logger);

bool move2joints(moveit::planning_interface::MoveGroupInterface &move_group,
                 const rclcpp::Logger &logger,
                 const std::vector<double> &joint_values);

bool move2pose(moveit::planning_interface::MoveGroupInterface &move_group,
               const rclcpp::Logger &logger,
               const std::vector<double> &pose_values);

bool moveCartesian(moveit::planning_interface::MoveGroupInterface &move_group,
                   const rclcpp::Logger &logger,
                   const std::vector<double> &pose_values);

class RobotController : public rclcpp::Node
{
  const std::string PLANNING_GROUP = "ur_manipulator";
  const std::string ENDEFFECTOR = "";

public:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using MoveJointAction = modproft_robot_control_msgs::action::MoveJoint;
  using MovePoseAction = modproft_robot_control_msgs::action::MovePose;
  using MoveCartesianAction = modproft_robot_control_msgs::action::MoveCartesian;
  using MoveTCPAction = modproft_robot_control_msgs::action::MoveTCP; //################
  using GoalHandleMoveJoint = rclcpp_action::ServerGoalHandle<MoveJointAction>;
  using GoalHandleMovePose = rclcpp_action::ServerGoalHandle<MovePoseAction>;
  using GoalHandleMoveCartesian = rclcpp_action::ServerGoalHandle<MoveCartesianAction>;
  using GoalHandleMoveTCP = rclcpp_action::ServerGoalHandle<MoveTCPAction>;

  // Create the MoveIt MoveGroupInterface and the pointer to this node with below syntax,
  // creating them in the constructor body didn't work
  RobotController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("robotcontrol", options), pnode(this), move_group(MoveGroupInterface(pnode, PLANNING_GROUP))
  {
    using namespace std::placeholders;

    this->action_server_movejoint_ = rclcpp_action::create_server<MoveJointAction>(
        this,
        "movejoint",
        std::bind(&RobotController::handle_goal_joint, this, _1, _2),
        std::bind(&RobotController::handle_cancel_joint, this, _1),
        std::bind(&RobotController::handle_accepted_joint, this, _1));

    this->action_server_movepose_ = rclcpp_action::create_server<MovePoseAction>(
        this,
        "movepose",
        std::bind(&RobotController::handle_goal_pose, this, _1, _2),
        std::bind(&RobotController::handle_cancel_pose, this, _1),
        std::bind(&RobotController::handle_accepted_pose, this, _1));

    this->action_server_movecartesian_ = rclcpp_action::create_server<MoveCartesianAction>(
        this,
        "movecartesian",
        std::bind(&RobotController::handle_goal_cartesian, this, _1, _2),
        std::bind(&RobotController::handle_cancel_cartesian, this, _1),
        std::bind(&RobotController::handle_accepted_cartesian, this, _1));

    /*this->action_server_movetcp_ = rclcpp_action::create_server<MoveTCPAction>(
        this,
        "movetcp",
        std::bind(&RobotController::handle_goal_tcp, this, _1, _2),
        std::bind(&RobotController::handle_cancel_tcp, this, _1),
        std::bind(&RobotController::handle_accepted_tcp, this, _1));*/

    if (ENDEFFECTOR != "")
    {
      move_group.setEndEffectorLink(ENDEFFECTOR);
    }

    // Print robot status info
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "Robot info:");
    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "Reference frame: %s", move_group.getPoseReferenceFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());
  }
  void logRobotState();

private:
  std::shared_ptr<rclcpp::Node> pnode;
  moveit::planning_interface::MoveGroupInterface move_group;
  rclcpp_action::Server<MoveJointAction>::SharedPtr action_server_movejoint_;
  rclcpp_action::Server<MovePoseAction>::SharedPtr action_server_movepose_;
  rclcpp_action::Server<MoveCartesianAction>::SharedPtr action_server_movecartesian_;
  //rclcpp_action::Server<MoveTCPAction>::SharedPtr action_server_movetcp_;

  // =================================================================================================================

  rclcpp_action::GoalResponse handle_goal_joint(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const MoveJointAction::Goal> goal);

  rclcpp_action::GoalResponse handle_goal_pose(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const MovePoseAction::Goal> goal);

  rclcpp_action::GoalResponse handle_goal_cartesian(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const MoveCartesianAction::Goal> goal);

  rclcpp_action::GoalResponse handle_goal_tcp(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const MoveTCPAction::Goal> goal);

  // =================================================================================================================

  rclcpp_action::CancelResponse handle_cancel_joint(
      const std::shared_ptr<GoalHandleMoveJoint> goal_handle);

  rclcpp_action::CancelResponse handle_cancel_pose(
      const std::shared_ptr<GoalHandleMovePose> goal_handle);

  rclcpp_action::CancelResponse handle_cancel_cartesian(
      const std::shared_ptr<GoalHandleMoveCartesian> goal_handle);

  rclcpp_action::CancelResponse handle_cancel_tcp(
      const std::shared_ptr<GoalHandleMoveTCP> goal_handle);

  // =================================================================================================================

  void handle_accepted_joint(const std::shared_ptr<GoalHandleMoveJoint> goal_handle);

  void handle_accepted_pose(const std::shared_ptr<GoalHandleMovePose> goal_handle);

  void handle_accepted_cartesian(const std::shared_ptr<GoalHandleMoveCartesian> goal_handle);

  void handle_accepted_tcp(const std::shared_ptr<GoalHandleMoveTCP> goal_handle);

  // =================================================================================================================

  void execute_joint(const std::shared_ptr<GoalHandleMoveJoint> goal_handle);

  void execute_pose(const std::shared_ptr<GoalHandleMovePose> goal_handle);

  void execute_cartesian(const std::shared_ptr<GoalHandleMoveCartesian> goal_handle);

  void execute_tcp(const std::shared_ptr<GoalHandleMoveTCP> goal_handle);
};
