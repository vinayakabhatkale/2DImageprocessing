// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rcontrol_msgs:action/MoveCartesian.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rcontrol_msgs/action/detail/move_cartesian__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rcontrol_msgs
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void MoveCartesian_Goal_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rcontrol_msgs::action::MoveCartesian_Goal(_init);
}

void MoveCartesian_Goal_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rcontrol_msgs::action::MoveCartesian_Goal *>(message_memory);
  typed_message->~MoveCartesian_Goal();
}

size_t size_function__MoveCartesian_Goal__posevalues(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MoveCartesian_Goal__posevalues(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__MoveCartesian_Goal__posevalues(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void resize_function__MoveCartesian_Goal__posevalues(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MoveCartesian_Goal_message_member_array[1] = {
  {
    "posevalues",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs::action::MoveCartesian_Goal, posevalues),  // bytes offset in struct
    nullptr,  // default value
    size_function__MoveCartesian_Goal__posevalues,  // size() function pointer
    get_const_function__MoveCartesian_Goal__posevalues,  // get_const(index) function pointer
    get_function__MoveCartesian_Goal__posevalues,  // get(index) function pointer
    resize_function__MoveCartesian_Goal__posevalues  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MoveCartesian_Goal_message_members = {
  "rcontrol_msgs::action",  // message namespace
  "MoveCartesian_Goal",  // message name
  1,  // number of fields
  sizeof(rcontrol_msgs::action::MoveCartesian_Goal),
  MoveCartesian_Goal_message_member_array,  // message members
  MoveCartesian_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveCartesian_Goal_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MoveCartesian_Goal_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveCartesian_Goal_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace rcontrol_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rcontrol_msgs::action::MoveCartesian_Goal>()
{
  return &::rcontrol_msgs::action::rosidl_typesupport_introspection_cpp::MoveCartesian_Goal_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rcontrol_msgs, action, MoveCartesian_Goal)() {
  return &::rcontrol_msgs::action::rosidl_typesupport_introspection_cpp::MoveCartesian_Goal_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_cartesian__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rcontrol_msgs
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void MoveCartesian_Result_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rcontrol_msgs::action::MoveCartesian_Result(_init);
}

void MoveCartesian_Result_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rcontrol_msgs::action::MoveCartesian_Result *>(message_memory);
  typed_message->~MoveCartesian_Result();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MoveCartesian_Result_message_member_array[1] = {
  {
    "success",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs::action::MoveCartesian_Result, success),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MoveCartesian_Result_message_members = {
  "rcontrol_msgs::action",  // message namespace
  "MoveCartesian_Result",  // message name
  1,  // number of fields
  sizeof(rcontrol_msgs::action::MoveCartesian_Result),
  MoveCartesian_Result_message_member_array,  // message members
  MoveCartesian_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveCartesian_Result_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MoveCartesian_Result_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveCartesian_Result_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace rcontrol_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rcontrol_msgs::action::MoveCartesian_Result>()
{
  return &::rcontrol_msgs::action::rosidl_typesupport_introspection_cpp::MoveCartesian_Result_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rcontrol_msgs, action, MoveCartesian_Result)() {
  return &::rcontrol_msgs::action::rosidl_typesupport_introspection_cpp::MoveCartesian_Result_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_cartesian__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rcontrol_msgs
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void MoveCartesian_Feedback_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rcontrol_msgs::action::MoveCartesian_Feedback(_init);
}

void MoveCartesian_Feedback_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rcontrol_msgs::action::MoveCartesian_Feedback *>(message_memory);
  typed_message->~MoveCartesian_Feedback();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MoveCartesian_Feedback_message_member_array[1] = {
  {
    "status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs::action::MoveCartesian_Feedback, status),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MoveCartesian_Feedback_message_members = {
  "rcontrol_msgs::action",  // message namespace
  "MoveCartesian_Feedback",  // message name
  1,  // number of fields
  sizeof(rcontrol_msgs::action::MoveCartesian_Feedback),
  MoveCartesian_Feedback_message_member_array,  // message members
  MoveCartesian_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveCartesian_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MoveCartesian_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveCartesian_Feedback_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace rcontrol_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rcontrol_msgs::action::MoveCartesian_Feedback>()
{
  return &::rcontrol_msgs::action::rosidl_typesupport_introspection_cpp::MoveCartesian_Feedback_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rcontrol_msgs, action, MoveCartesian_Feedback)() {
  return &::rcontrol_msgs::action::rosidl_typesupport_introspection_cpp::MoveCartesian_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_cartesian__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rcontrol_msgs
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void MoveCartesian_SendGoal_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rcontrol_msgs::action::MoveCartesian_SendGoal_Request(_init);
}

void MoveCartesian_SendGoal_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rcontrol_msgs::action::MoveCartesian_SendGoal_Request *>(message_memory);
  typed_message->~MoveCartesian_SendGoal_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MoveCartesian_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<unique_identifier_msgs::msg::UUID>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs::action::MoveCartesian_SendGoal_Request, goal_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "goal",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rcontrol_msgs::action::MoveCartesian_Goal>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs::action::MoveCartesian_SendGoal_Request, goal),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MoveCartesian_SendGoal_Request_message_members = {
  "rcontrol_msgs::action",  // message namespace
  "MoveCartesian_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(rcontrol_msgs::action::MoveCartesian_SendGoal_Request),
  MoveCartesian_SendGoal_Request_message_member_array,  // message members
  MoveCartesian_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveCartesian_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MoveCartesian_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveCartesian_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace rcontrol_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rcontrol_msgs::action::MoveCartesian_SendGoal_Request>()
{
  return &::rcontrol_msgs::action::rosidl_typesupport_introspection_cpp::MoveCartesian_SendGoal_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rcontrol_msgs, action, MoveCartesian_SendGoal_Request)() {
  return &::rcontrol_msgs::action::rosidl_typesupport_introspection_cpp::MoveCartesian_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_cartesian__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rcontrol_msgs
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void MoveCartesian_SendGoal_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rcontrol_msgs::action::MoveCartesian_SendGoal_Response(_init);
}

void MoveCartesian_SendGoal_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rcontrol_msgs::action::MoveCartesian_SendGoal_Response *>(message_memory);
  typed_message->~MoveCartesian_SendGoal_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MoveCartesian_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs::action::MoveCartesian_SendGoal_Response, accepted),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "stamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs::action::MoveCartesian_SendGoal_Response, stamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MoveCartesian_SendGoal_Response_message_members = {
  "rcontrol_msgs::action",  // message namespace
  "MoveCartesian_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(rcontrol_msgs::action::MoveCartesian_SendGoal_Response),
  MoveCartesian_SendGoal_Response_message_member_array,  // message members
  MoveCartesian_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveCartesian_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MoveCartesian_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveCartesian_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace rcontrol_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rcontrol_msgs::action::MoveCartesian_SendGoal_Response>()
{
  return &::rcontrol_msgs::action::rosidl_typesupport_introspection_cpp::MoveCartesian_SendGoal_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rcontrol_msgs, action, MoveCartesian_SendGoal_Response)() {
  return &::rcontrol_msgs::action::rosidl_typesupport_introspection_cpp::MoveCartesian_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_cartesian__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace rcontrol_msgs
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers MoveCartesian_SendGoal_service_members = {
  "rcontrol_msgs::action",  // service namespace
  "MoveCartesian_SendGoal",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<rcontrol_msgs::action::MoveCartesian_SendGoal>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t MoveCartesian_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveCartesian_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace rcontrol_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<rcontrol_msgs::action::MoveCartesian_SendGoal>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::rcontrol_msgs::action::rosidl_typesupport_introspection_cpp::MoveCartesian_SendGoal_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::rcontrol_msgs::action::MoveCartesian_SendGoal_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::rcontrol_msgs::action::MoveCartesian_SendGoal_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rcontrol_msgs, action, MoveCartesian_SendGoal)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<rcontrol_msgs::action::MoveCartesian_SendGoal>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_cartesian__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rcontrol_msgs
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void MoveCartesian_GetResult_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rcontrol_msgs::action::MoveCartesian_GetResult_Request(_init);
}

void MoveCartesian_GetResult_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rcontrol_msgs::action::MoveCartesian_GetResult_Request *>(message_memory);
  typed_message->~MoveCartesian_GetResult_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MoveCartesian_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<unique_identifier_msgs::msg::UUID>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs::action::MoveCartesian_GetResult_Request, goal_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MoveCartesian_GetResult_Request_message_members = {
  "rcontrol_msgs::action",  // message namespace
  "MoveCartesian_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(rcontrol_msgs::action::MoveCartesian_GetResult_Request),
  MoveCartesian_GetResult_Request_message_member_array,  // message members
  MoveCartesian_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveCartesian_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MoveCartesian_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveCartesian_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace rcontrol_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rcontrol_msgs::action::MoveCartesian_GetResult_Request>()
{
  return &::rcontrol_msgs::action::rosidl_typesupport_introspection_cpp::MoveCartesian_GetResult_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rcontrol_msgs, action, MoveCartesian_GetResult_Request)() {
  return &::rcontrol_msgs::action::rosidl_typesupport_introspection_cpp::MoveCartesian_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_cartesian__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rcontrol_msgs
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void MoveCartesian_GetResult_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rcontrol_msgs::action::MoveCartesian_GetResult_Response(_init);
}

void MoveCartesian_GetResult_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rcontrol_msgs::action::MoveCartesian_GetResult_Response *>(message_memory);
  typed_message->~MoveCartesian_GetResult_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MoveCartesian_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs::action::MoveCartesian_GetResult_Response, status),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "result",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rcontrol_msgs::action::MoveCartesian_Result>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs::action::MoveCartesian_GetResult_Response, result),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MoveCartesian_GetResult_Response_message_members = {
  "rcontrol_msgs::action",  // message namespace
  "MoveCartesian_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(rcontrol_msgs::action::MoveCartesian_GetResult_Response),
  MoveCartesian_GetResult_Response_message_member_array,  // message members
  MoveCartesian_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveCartesian_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MoveCartesian_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveCartesian_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace rcontrol_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rcontrol_msgs::action::MoveCartesian_GetResult_Response>()
{
  return &::rcontrol_msgs::action::rosidl_typesupport_introspection_cpp::MoveCartesian_GetResult_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rcontrol_msgs, action, MoveCartesian_GetResult_Response)() {
  return &::rcontrol_msgs::action::rosidl_typesupport_introspection_cpp::MoveCartesian_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_cartesian__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace rcontrol_msgs
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers MoveCartesian_GetResult_service_members = {
  "rcontrol_msgs::action",  // service namespace
  "MoveCartesian_GetResult",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<rcontrol_msgs::action::MoveCartesian_GetResult>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t MoveCartesian_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveCartesian_GetResult_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace rcontrol_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<rcontrol_msgs::action::MoveCartesian_GetResult>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::rcontrol_msgs::action::rosidl_typesupport_introspection_cpp::MoveCartesian_GetResult_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::rcontrol_msgs::action::MoveCartesian_GetResult_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::rcontrol_msgs::action::MoveCartesian_GetResult_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rcontrol_msgs, action, MoveCartesian_GetResult)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<rcontrol_msgs::action::MoveCartesian_GetResult>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_cartesian__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rcontrol_msgs
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void MoveCartesian_FeedbackMessage_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rcontrol_msgs::action::MoveCartesian_FeedbackMessage(_init);
}

void MoveCartesian_FeedbackMessage_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rcontrol_msgs::action::MoveCartesian_FeedbackMessage *>(message_memory);
  typed_message->~MoveCartesian_FeedbackMessage();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MoveCartesian_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<unique_identifier_msgs::msg::UUID>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs::action::MoveCartesian_FeedbackMessage, goal_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "feedback",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rcontrol_msgs::action::MoveCartesian_Feedback>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs::action::MoveCartesian_FeedbackMessage, feedback),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MoveCartesian_FeedbackMessage_message_members = {
  "rcontrol_msgs::action",  // message namespace
  "MoveCartesian_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(rcontrol_msgs::action::MoveCartesian_FeedbackMessage),
  MoveCartesian_FeedbackMessage_message_member_array,  // message members
  MoveCartesian_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveCartesian_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MoveCartesian_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveCartesian_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace rcontrol_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rcontrol_msgs::action::MoveCartesian_FeedbackMessage>()
{
  return &::rcontrol_msgs::action::rosidl_typesupport_introspection_cpp::MoveCartesian_FeedbackMessage_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rcontrol_msgs, action, MoveCartesian_FeedbackMessage)() {
  return &::rcontrol_msgs::action::rosidl_typesupport_introspection_cpp::MoveCartesian_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
