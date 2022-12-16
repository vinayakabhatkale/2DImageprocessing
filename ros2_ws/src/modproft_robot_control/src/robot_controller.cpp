#include "../include/modproft_robot_control/robot_controller.hpp"

void RobotController::logRobotState()
{
    logCurrentRobotState(this->move_group, this->get_logger());
}

rclcpp_action::GoalResponse RobotController::handle_goal_joint(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const MoveJointAction::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request: %s", vector2string(goal->jointvalues).c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse RobotController::handle_goal_pose(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const MovePoseAction::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request: %s", vector2string(goal->posevalues).c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse RobotController::handle_goal_cartesian(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const MoveCartesianAction::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request: %s", vector2string(goal->posevalues).c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/*rclcpp_action::GoalResponse RobotController::handle_goal_tcp(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const MoveTCPAction::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request: %s", vector2string(goal->posevalues).c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}*/

// =================================================================================================================

rclcpp_action::CancelResponse RobotController::handle_cancel_joint(
    const std::shared_ptr<GoalHandleMoveJoint> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::CancelResponse RobotController::handle_cancel_pose(
    const std::shared_ptr<GoalHandleMovePose> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::CancelResponse RobotController::handle_cancel_cartesian(
    const std::shared_ptr<GoalHandleMoveCartesian> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

/*rclcpp_action::CancelResponse RobotController::handle_cancel_tcp(
    const std::shared_ptr<GoalHandleMoveTCP> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}*/

// =================================================================================================================

void RobotController::handle_accepted_joint(const std::shared_ptr<GoalHandleMoveJoint> goal_handle)
{
    using namespace std::placeholders;
    RCLCPP_INFO(this->get_logger(), "Executing move request");

    // This needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&RobotController::execute_joint, this, _1), goal_handle}.detach();
}

void RobotController::handle_accepted_pose(const std::shared_ptr<GoalHandleMovePose> goal_handle)
{
    using namespace std::placeholders;
    RCLCPP_INFO(this->get_logger(), "Executing move request");

    // This needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&RobotController::execute_pose, this, _1), goal_handle}.detach();
}

void RobotController::handle_accepted_cartesian(const std::shared_ptr<GoalHandleMoveCartesian> goal_handle)
{
    using namespace std::placeholders;
    RCLCPP_INFO(this->get_logger(), "Executing move request");

    // This needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&RobotController::execute_cartesian, this, _1), goal_handle}.detach();
}

/*void RobotController::handle_accepted_tcp(const std::shared_ptr<GoalHandleMoveTCP> goal_handle)
{
    using namespace std::placeholders;
    RCLCPP_INFO(this->get_logger(), "Executing move request");

    // This needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&RobotController::execute_tcp, this, _1), goal_handle}.detach();
}*/

// =================================================================================================================

void RobotController::execute_joint(const std::shared_ptr<GoalHandleMoveJoint> goal_handle)
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

void RobotController::execute_pose(const std::shared_ptr<GoalHandleMovePose> goal_handle)
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

void RobotController::execute_cartesian(const std::shared_ptr<GoalHandleMoveCartesian> goal_handle)
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

/*void RobotController::execute_tcp(const std::shared_ptr<GoalHandleMoveTCP> goal_handle)
{
    auto feedback = std::make_shared<MoveTCPAction::Feedback>();
    auto result = std::make_shared<MoveTCPAction::Result>();
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
}*/

// =================================================================================================================
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

void logCurrentRobotState(moveit::planning_interface::MoveGroupInterface &move_group,
                          const rclcpp::Logger &logger)
{
    std::vector<double> joint_values;
    joint_values = move_group.getCurrentJointValues();

    geometry_msgs::msg::PoseStamped current_pose;
    current_pose = move_group.getCurrentPose();

    std::string joint_values_str(joint_values.begin(), joint_values.end());
    RCLCPP_INFO(logger, "Joint values: " + vector2string(joint_values));
}

bool move2joints(moveit::planning_interface::MoveGroupInterface &move_group,
                 const rclcpp::Logger &logger,
                 const std::vector<double> &joint_values)
{
    RCLCPP_INFO(logger, "Moving to joint position: %s", ::vector2string(joint_values).c_str());

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

// =================================================================================================================

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