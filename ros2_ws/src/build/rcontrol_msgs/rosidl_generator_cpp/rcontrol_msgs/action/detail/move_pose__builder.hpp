// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rcontrol_msgs:action/MovePose.idl
// generated code does not contain a copyright notice

#ifndef RCONTROL_MSGS__ACTION__DETAIL__MOVE_POSE__BUILDER_HPP_
#define RCONTROL_MSGS__ACTION__DETAIL__MOVE_POSE__BUILDER_HPP_

#include "rcontrol_msgs/action/detail/move_pose__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rcontrol_msgs
{

namespace action
{

namespace builder
{

class Init_MovePose_Goal_posevalues
{
public:
  Init_MovePose_Goal_posevalues()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rcontrol_msgs::action::MovePose_Goal posevalues(::rcontrol_msgs::action::MovePose_Goal::_posevalues_type arg)
  {
    msg_.posevalues = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcontrol_msgs::action::MovePose_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcontrol_msgs::action::MovePose_Goal>()
{
  return rcontrol_msgs::action::builder::Init_MovePose_Goal_posevalues();
}

}  // namespace rcontrol_msgs


namespace rcontrol_msgs
{

namespace action
{

namespace builder
{

class Init_MovePose_Result_success
{
public:
  Init_MovePose_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rcontrol_msgs::action::MovePose_Result success(::rcontrol_msgs::action::MovePose_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcontrol_msgs::action::MovePose_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcontrol_msgs::action::MovePose_Result>()
{
  return rcontrol_msgs::action::builder::Init_MovePose_Result_success();
}

}  // namespace rcontrol_msgs


namespace rcontrol_msgs
{

namespace action
{

namespace builder
{

class Init_MovePose_Feedback_status
{
public:
  Init_MovePose_Feedback_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rcontrol_msgs::action::MovePose_Feedback status(::rcontrol_msgs::action::MovePose_Feedback::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcontrol_msgs::action::MovePose_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcontrol_msgs::action::MovePose_Feedback>()
{
  return rcontrol_msgs::action::builder::Init_MovePose_Feedback_status();
}

}  // namespace rcontrol_msgs


namespace rcontrol_msgs
{

namespace action
{

namespace builder
{

class Init_MovePose_SendGoal_Request_goal
{
public:
  explicit Init_MovePose_SendGoal_Request_goal(::rcontrol_msgs::action::MovePose_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::rcontrol_msgs::action::MovePose_SendGoal_Request goal(::rcontrol_msgs::action::MovePose_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcontrol_msgs::action::MovePose_SendGoal_Request msg_;
};

class Init_MovePose_SendGoal_Request_goal_id
{
public:
  Init_MovePose_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MovePose_SendGoal_Request_goal goal_id(::rcontrol_msgs::action::MovePose_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MovePose_SendGoal_Request_goal(msg_);
  }

private:
  ::rcontrol_msgs::action::MovePose_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcontrol_msgs::action::MovePose_SendGoal_Request>()
{
  return rcontrol_msgs::action::builder::Init_MovePose_SendGoal_Request_goal_id();
}

}  // namespace rcontrol_msgs


namespace rcontrol_msgs
{

namespace action
{

namespace builder
{

class Init_MovePose_SendGoal_Response_stamp
{
public:
  explicit Init_MovePose_SendGoal_Response_stamp(::rcontrol_msgs::action::MovePose_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::rcontrol_msgs::action::MovePose_SendGoal_Response stamp(::rcontrol_msgs::action::MovePose_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcontrol_msgs::action::MovePose_SendGoal_Response msg_;
};

class Init_MovePose_SendGoal_Response_accepted
{
public:
  Init_MovePose_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MovePose_SendGoal_Response_stamp accepted(::rcontrol_msgs::action::MovePose_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_MovePose_SendGoal_Response_stamp(msg_);
  }

private:
  ::rcontrol_msgs::action::MovePose_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcontrol_msgs::action::MovePose_SendGoal_Response>()
{
  return rcontrol_msgs::action::builder::Init_MovePose_SendGoal_Response_accepted();
}

}  // namespace rcontrol_msgs


namespace rcontrol_msgs
{

namespace action
{

namespace builder
{

class Init_MovePose_GetResult_Request_goal_id
{
public:
  Init_MovePose_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rcontrol_msgs::action::MovePose_GetResult_Request goal_id(::rcontrol_msgs::action::MovePose_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcontrol_msgs::action::MovePose_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcontrol_msgs::action::MovePose_GetResult_Request>()
{
  return rcontrol_msgs::action::builder::Init_MovePose_GetResult_Request_goal_id();
}

}  // namespace rcontrol_msgs


namespace rcontrol_msgs
{

namespace action
{

namespace builder
{

class Init_MovePose_GetResult_Response_result
{
public:
  explicit Init_MovePose_GetResult_Response_result(::rcontrol_msgs::action::MovePose_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::rcontrol_msgs::action::MovePose_GetResult_Response result(::rcontrol_msgs::action::MovePose_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcontrol_msgs::action::MovePose_GetResult_Response msg_;
};

class Init_MovePose_GetResult_Response_status
{
public:
  Init_MovePose_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MovePose_GetResult_Response_result status(::rcontrol_msgs::action::MovePose_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_MovePose_GetResult_Response_result(msg_);
  }

private:
  ::rcontrol_msgs::action::MovePose_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcontrol_msgs::action::MovePose_GetResult_Response>()
{
  return rcontrol_msgs::action::builder::Init_MovePose_GetResult_Response_status();
}

}  // namespace rcontrol_msgs


namespace rcontrol_msgs
{

namespace action
{

namespace builder
{

class Init_MovePose_FeedbackMessage_feedback
{
public:
  explicit Init_MovePose_FeedbackMessage_feedback(::rcontrol_msgs::action::MovePose_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::rcontrol_msgs::action::MovePose_FeedbackMessage feedback(::rcontrol_msgs::action::MovePose_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rcontrol_msgs::action::MovePose_FeedbackMessage msg_;
};

class Init_MovePose_FeedbackMessage_goal_id
{
public:
  Init_MovePose_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MovePose_FeedbackMessage_feedback goal_id(::rcontrol_msgs::action::MovePose_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MovePose_FeedbackMessage_feedback(msg_);
  }

private:
  ::rcontrol_msgs::action::MovePose_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::rcontrol_msgs::action::MovePose_FeedbackMessage>()
{
  return rcontrol_msgs::action::builder::Init_MovePose_FeedbackMessage_goal_id();
}

}  // namespace rcontrol_msgs

#endif  // RCONTROL_MSGS__ACTION__DETAIL__MOVE_POSE__BUILDER_HPP_
