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
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// =====================================================================================================================

template <typename T>
std::string vector2string(const std::vector<T> &v)
{
  std::stringstream ss;
  ss << "[";
  for (size_t i = 0; i < v.size(); ++i)
  {
    if (i != 0)
      ss << ", ";
    ss << v[i];
  }
  ss << "]";
  std::string s = ss.str();
  return s;
}

// =====================================================================================================================

void logCurrentRobotState(moveit::planning_interface::MoveGroupInterface &move_group,
                          const rclcpp::Logger &logger)
{
  std::vector<double> joint_values;
  joint_values = move_group.getCurrentJointValues();

  geometry_msgs::msg::PoseStamped current_pose;
  current_pose = move_group.getCurrentPose();
}

// =====================================================================================================================

bool move2joints(moveit::planning_interface::MoveGroupInterface &move_group,
                 const rclcpp::Logger &logger,
                 const std::vector<double> &joint_values)
{
  RCLCPP_INFO(logger, "Moving to joint position: %s", vector2string(joint_values).c_str());

  if (joint_values.size() != 6)
  {
    RCLCPP_ERROR(logger, "Wrong number of joint values!");
    return false;
  }

  // Set joint target position
  move_group.setJointValueTarget(joint_values);

  // Create a plan to that target pose
  auto const [success_p, plan] = [&move_group]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // If planning failed, stop here
  if (success_p)
  {
    RCLCPP_INFO(logger, "Planing succeeded!");
  }
  else
  {
    RCLCPP_ERROR(logger, "Planing failed!");
    return false;
  }

  // Move along planned path
  auto const success_e = static_cast<bool>(move_group.execute(plan));

  // Print current robot state
  logCurrentRobotState(move_group, logger);

  // If execution failed, stop here
  if (success_e)
  {
    RCLCPP_INFO(logger, "Moving succeeded!");
  }
  else
  {
    RCLCPP_ERROR(logger, "Moving failed!");
    return false;
  }

  return true;
}

// =====================================================================================================================

bool move2pose(moveit::planning_interface::MoveGroupInterface &move_group,
               const rclcpp::Logger &logger,
               const std::vector<double> &pose_values)
{
  if (pose_values.size() != 7)
  {
    RCLCPP_ERROR(logger, "Wrong number of pose values!");
    return false;
  }

  auto const target_pose = [&pose_values]
  {
    geometry_msgs::msg::Pose msg;
    msg.position.x = pose_values[0];
    msg.position.y = pose_values[1];
    msg.position.z = pose_values[2];
    msg.orientation.x = pose_values[3];
    msg.orientation.y = pose_values[4];
    msg.orientation.z = pose_values[5];
    msg.orientation.w = pose_values[6];
    return msg;
  }();

  // Set pose target position
  move_group.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success_p, plan] = [&move_group]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // If planning failed, stop here
  if (success_p)
  {
    RCLCPP_INFO(logger, "Planing succeeded!");
  }
  else
  {
    RCLCPP_ERROR(logger, "Planing failed!");
    return false;
  }

  // Move along planned path
  auto const success_e = static_cast<bool>(move_group.execute(plan));

  // Print current robot state
  logCurrentRobotState(move_group, logger);

  // If execution failed, stop here
  if (success_e)
  {
    RCLCPP_INFO(logger, "Moving succeeded!");
  }
  else
  {
    RCLCPP_ERROR(logger, "Moving failed!");
    return false;
  }

  return true;
}

// =====================================================================================================================

bool moveCartesian(moveit::planning_interface::MoveGroupInterface &move_group,
                   const rclcpp::Logger &logger,
                   const std::vector<double> &pose_values)
{
  if (pose_values.size() != 7)
  {
    RCLCPP_ERROR(logger, "Wrong number of pose values!");
    return false;
  }

  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose target_pose;

  target_pose.position.x = pose_values[0];
  target_pose.position.y = pose_values[1];
  target_pose.position.z = pose_values[2];
  target_pose.orientation.x = pose_values[3];
  target_pose.orientation.y = pose_values[4];
  target_pose.orientation.z = pose_values[5];
  target_pose.orientation.w = pose_values[6];

  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  const double success_p = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  // If planning failed, stop here
  if (success_p == 1.0)
  {
    RCLCPP_INFO(logger, "Planing succeeded!");
  }
  else
  {
    RCLCPP_ERROR(logger, "Planing failed! %.2f%% achieved.", success_p * 100.0);
    return false;
  }

  // Move along planned path
  auto const success_e = static_cast<bool>(move_group.execute(trajectory));

  // Print current robot state
  logCurrentRobotState(move_group, logger);

  // If execution failed, stop here
  if (success_e)
  {
    RCLCPP_INFO(logger, "Moving succeeded!");
  }
  else
  {
    RCLCPP_ERROR(logger, "Moving failed!");
    return false;
  }

  return true;
}

// =====================================================================================================================

class RobotController : public rclcpp::Node
{
  const std::string PLANNING_GROUP = "ur_manipulator";
  const std::string ENDEFFECTOR = "";

public:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using MoveJointAction = modproft_robot_control_msgs::action::MoveJoint;
  using MovePoseAction = modproft_robot_control_msgs::action::MovePose;
  using MoveCartesianAction = modproft_robot_control_msgs::action::MoveCartesian;
  using GoalHandleMoveJoint = rclcpp_action::ServerGoalHandle<MoveJointAction>;
  using GoalHandleMovePose = rclcpp_action::ServerGoalHandle<MovePoseAction>;
  using GoalHandleMoveCartesian = rclcpp_action::ServerGoalHandle<MoveCartesianAction>;

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

  // =================================================================================================================

  void logRobotState()
  {
    logCurrentRobotState(this->move_group, this->get_logger());
  }

  // =================================================================================================================

private:
  std::shared_ptr<rclcpp::Node> pnode;
  moveit::planning_interface::MoveGroupInterface move_group;
  rclcpp_action::Server<MoveJointAction>::SharedPtr action_server_movejoint_;
  rclcpp_action::Server<MovePoseAction>::SharedPtr action_server_movepose_;
  rclcpp_action::Server<MoveCartesianAction>::SharedPtr action_server_movecartesian_;

  // =================================================================================================================

  rclcpp_action::GoalResponse handle_goal_joint(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const MoveJointAction::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request: %s", vector2string(goal->jointvalues).c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::GoalResponse handle_goal_pose(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const MovePoseAction::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request: %s", vector2string(goal->posevalues).c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::GoalResponse handle_goal_cartesian(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const MoveCartesianAction::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request: %s", vector2string(goal->posevalues).c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // =================================================================================================================

  rclcpp_action::CancelResponse handle_cancel_joint(
      const std::shared_ptr<GoalHandleMoveJoint> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  rclcpp_action::CancelResponse handle_cancel_pose(
      const std::shared_ptr<GoalHandleMovePose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  rclcpp_action::CancelResponse handle_cancel_cartesian(
      const std::shared_ptr<GoalHandleMoveCartesian> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // =================================================================================================================

  void handle_accepted_joint(const std::shared_ptr<GoalHandleMoveJoint> goal_handle)
  {
    using namespace std::placeholders;
    RCLCPP_INFO(this->get_logger(), "Executing move request");

    // This needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&RobotController::execute_joint, this, _1), goal_handle}.detach();
  }

  void handle_accepted_pose(const std::shared_ptr<GoalHandleMovePose> goal_handle)
  {
    using namespace std::placeholders;
    RCLCPP_INFO(this->get_logger(), "Executing move request");

    // This needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&RobotController::execute_pose, this, _1), goal_handle}.detach();
  }

  void handle_accepted_cartesian(const std::shared_ptr<GoalHandleMoveCartesian> goal_handle)
  {
    using namespace std::placeholders;
    RCLCPP_INFO(this->get_logger(), "Executing move request");

    // This needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&RobotController::execute_cartesian, this, _1), goal_handle}.detach();
  }

  // =================================================================================================================

  void execute_joint(const std::shared_ptr<GoalHandleMoveJoint> goal_handle)
  {
    auto feedback = std::make_shared<MoveJointAction::Feedback>();
    auto result = std::make_shared<MoveJointAction::Result>();
    const auto goal = goal_handle->get_goal();
    rclcpp::Rate loop_rate(1);

    std::packaged_task<bool(moveit::planning_interface::MoveGroupInterface &,
                            const rclcpp::Logger &,
                            const std::vector<double> &)>
        task(move2joints);
    auto f = task.get_future();

    std::thread move_thread(std::move(task), std::ref(this->move_group), this->get_logger(), std::cref(goal->jointvalues));
    move_thread.detach();

    while (move_thread.joinable())
    {

      // Check if there is a cancel request
      if (goal_handle->is_canceling())
      {
        this->move_group.stop();
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // Publish feedback
      feedback->status = 1;
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok())
    {
      bool res = f.get();
      result->success = res;
      goal_handle->succeed(result);
      if (res)
      {
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Goal failed");
      }
    }
  }

  void execute_pose(const std::shared_ptr<GoalHandleMovePose> goal_handle)
  {
    auto feedback = std::make_shared<MovePoseAction::Feedback>();
    auto result = std::make_shared<MovePoseAction::Result>();
    const auto goal = goal_handle->get_goal();
    rclcpp::Rate loop_rate(1);

    std::packaged_task<bool(moveit::planning_interface::MoveGroupInterface &,
                            const rclcpp::Logger &,
                            const std::vector<double> &)>
        task(move2pose);
    auto f = task.get_future();

    std::thread move_thread(std::move(task), std::ref(this->move_group), this->get_logger(), std::cref(goal->posevalues));
    move_thread.detach();

    while (move_thread.joinable())
    {

      // Check if there is a cancel request
      if (goal_handle->is_canceling())
      {
        this->move_group.stop();
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // Publish feedback
      feedback->status = 1;
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok())
    {
      bool res = f.get();
      result->success = res;
      goal_handle->succeed(result);
      if (res)
      {
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Goal failed");
      }
    }
  }

  void execute_cartesian(const std::shared_ptr<GoalHandleMoveCartesian> goal_handle)
  {
    auto feedback = std::make_shared<MoveCartesianAction::Feedback>();
    auto result = std::make_shared<MoveCartesianAction::Result>();
    const auto goal = goal_handle->get_goal();
    rclcpp::Rate loop_rate(1);

    std::packaged_task<bool(moveit::planning_interface::MoveGroupInterface &,
                            const rclcpp::Logger &,
                            const std::vector<double> &)>
        task(moveCartesian);
    auto f = task.get_future();

    std::thread move_thread(std::move(task), std::ref(this->move_group), this->get_logger(), std::cref(goal->posevalues));
    move_thread.detach();

    while (move_thread.joinable())
    {

      // Check if there is a cancel request
      if (goal_handle->is_canceling())
      {
        this->move_group.stop();
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // Publish feedback
      feedback->status = 1;
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok())
    {
      bool res = f.get();
      result->success = res;
      goal_handle->succeed(result);
      if (res)
      {
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Goal failed");
      }
    }
  }
};

// =====================================================================================================================

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto move_group_node = std::make_shared<RobotController>();

  // Spin up a SingleThreadedExecutor for the current state monitor to get information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread node_thread([&node_thread, &executor]()
                          { 
                                executor.spin();
                                node_thread.detach(); });

  // Print current robot state
  move_group_node->logRobotState();
  RCLCPP_INFO(move_group_node->get_logger(), "<Finished Initialization>");

  // Keep running
  while (node_thread.joinable())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();
  return 0;
}
