// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rcontrol_msgs:action/MoveJoint.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rcontrol_msgs/action/detail/move_joint__rosidl_typesupport_introspection_c.h"
#include "rcontrol_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rcontrol_msgs/action/detail/move_joint__functions.h"
#include "rcontrol_msgs/action/detail/move_joint__struct.h"


// Include directives for member types
// Member `jointvalues`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MoveJoint_Goal__rosidl_typesupport_introspection_c__MoveJoint_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rcontrol_msgs__action__MoveJoint_Goal__init(message_memory);
}

void MoveJoint_Goal__rosidl_typesupport_introspection_c__MoveJoint_Goal_fini_function(void * message_memory)
{
  rcontrol_msgs__action__MoveJoint_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MoveJoint_Goal__rosidl_typesupport_introspection_c__MoveJoint_Goal_message_member_array[1] = {
  {
    "jointvalues",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs__action__MoveJoint_Goal, jointvalues),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MoveJoint_Goal__rosidl_typesupport_introspection_c__MoveJoint_Goal_message_members = {
  "rcontrol_msgs__action",  // message namespace
  "MoveJoint_Goal",  // message name
  1,  // number of fields
  sizeof(rcontrol_msgs__action__MoveJoint_Goal),
  MoveJoint_Goal__rosidl_typesupport_introspection_c__MoveJoint_Goal_message_member_array,  // message members
  MoveJoint_Goal__rosidl_typesupport_introspection_c__MoveJoint_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveJoint_Goal__rosidl_typesupport_introspection_c__MoveJoint_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MoveJoint_Goal__rosidl_typesupport_introspection_c__MoveJoint_Goal_message_type_support_handle = {
  0,
  &MoveJoint_Goal__rosidl_typesupport_introspection_c__MoveJoint_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rcontrol_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcontrol_msgs, action, MoveJoint_Goal)() {
  if (!MoveJoint_Goal__rosidl_typesupport_introspection_c__MoveJoint_Goal_message_type_support_handle.typesupport_identifier) {
    MoveJoint_Goal__rosidl_typesupport_introspection_c__MoveJoint_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MoveJoint_Goal__rosidl_typesupport_introspection_c__MoveJoint_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rcontrol_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__functions.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void MoveJoint_Result__rosidl_typesupport_introspection_c__MoveJoint_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rcontrol_msgs__action__MoveJoint_Result__init(message_memory);
}

void MoveJoint_Result__rosidl_typesupport_introspection_c__MoveJoint_Result_fini_function(void * message_memory)
{
  rcontrol_msgs__action__MoveJoint_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MoveJoint_Result__rosidl_typesupport_introspection_c__MoveJoint_Result_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs__action__MoveJoint_Result, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MoveJoint_Result__rosidl_typesupport_introspection_c__MoveJoint_Result_message_members = {
  "rcontrol_msgs__action",  // message namespace
  "MoveJoint_Result",  // message name
  1,  // number of fields
  sizeof(rcontrol_msgs__action__MoveJoint_Result),
  MoveJoint_Result__rosidl_typesupport_introspection_c__MoveJoint_Result_message_member_array,  // message members
  MoveJoint_Result__rosidl_typesupport_introspection_c__MoveJoint_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveJoint_Result__rosidl_typesupport_introspection_c__MoveJoint_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MoveJoint_Result__rosidl_typesupport_introspection_c__MoveJoint_Result_message_type_support_handle = {
  0,
  &MoveJoint_Result__rosidl_typesupport_introspection_c__MoveJoint_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rcontrol_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcontrol_msgs, action, MoveJoint_Result)() {
  if (!MoveJoint_Result__rosidl_typesupport_introspection_c__MoveJoint_Result_message_type_support_handle.typesupport_identifier) {
    MoveJoint_Result__rosidl_typesupport_introspection_c__MoveJoint_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MoveJoint_Result__rosidl_typesupport_introspection_c__MoveJoint_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rcontrol_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__functions.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void MoveJoint_Feedback__rosidl_typesupport_introspection_c__MoveJoint_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rcontrol_msgs__action__MoveJoint_Feedback__init(message_memory);
}

void MoveJoint_Feedback__rosidl_typesupport_introspection_c__MoveJoint_Feedback_fini_function(void * message_memory)
{
  rcontrol_msgs__action__MoveJoint_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MoveJoint_Feedback__rosidl_typesupport_introspection_c__MoveJoint_Feedback_message_member_array[1] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs__action__MoveJoint_Feedback, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MoveJoint_Feedback__rosidl_typesupport_introspection_c__MoveJoint_Feedback_message_members = {
  "rcontrol_msgs__action",  // message namespace
  "MoveJoint_Feedback",  // message name
  1,  // number of fields
  sizeof(rcontrol_msgs__action__MoveJoint_Feedback),
  MoveJoint_Feedback__rosidl_typesupport_introspection_c__MoveJoint_Feedback_message_member_array,  // message members
  MoveJoint_Feedback__rosidl_typesupport_introspection_c__MoveJoint_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveJoint_Feedback__rosidl_typesupport_introspection_c__MoveJoint_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MoveJoint_Feedback__rosidl_typesupport_introspection_c__MoveJoint_Feedback_message_type_support_handle = {
  0,
  &MoveJoint_Feedback__rosidl_typesupport_introspection_c__MoveJoint_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rcontrol_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcontrol_msgs, action, MoveJoint_Feedback)() {
  if (!MoveJoint_Feedback__rosidl_typesupport_introspection_c__MoveJoint_Feedback_message_type_support_handle.typesupport_identifier) {
    MoveJoint_Feedback__rosidl_typesupport_introspection_c__MoveJoint_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MoveJoint_Feedback__rosidl_typesupport_introspection_c__MoveJoint_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rcontrol_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__functions.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "rcontrol_msgs/action/move_joint.h"
// Member `goal`
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MoveJoint_SendGoal_Request__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rcontrol_msgs__action__MoveJoint_SendGoal_Request__init(message_memory);
}

void MoveJoint_SendGoal_Request__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Request_fini_function(void * message_memory)
{
  rcontrol_msgs__action__MoveJoint_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MoveJoint_SendGoal_Request__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs__action__MoveJoint_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs__action__MoveJoint_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MoveJoint_SendGoal_Request__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Request_message_members = {
  "rcontrol_msgs__action",  // message namespace
  "MoveJoint_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(rcontrol_msgs__action__MoveJoint_SendGoal_Request),
  MoveJoint_SendGoal_Request__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Request_message_member_array,  // message members
  MoveJoint_SendGoal_Request__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveJoint_SendGoal_Request__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MoveJoint_SendGoal_Request__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Request_message_type_support_handle = {
  0,
  &MoveJoint_SendGoal_Request__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rcontrol_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcontrol_msgs, action, MoveJoint_SendGoal_Request)() {
  MoveJoint_SendGoal_Request__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  MoveJoint_SendGoal_Request__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcontrol_msgs, action, MoveJoint_Goal)();
  if (!MoveJoint_SendGoal_Request__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    MoveJoint_SendGoal_Request__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MoveJoint_SendGoal_Request__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rcontrol_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__functions.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MoveJoint_SendGoal_Response__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rcontrol_msgs__action__MoveJoint_SendGoal_Response__init(message_memory);
}

void MoveJoint_SendGoal_Response__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Response_fini_function(void * message_memory)
{
  rcontrol_msgs__action__MoveJoint_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MoveJoint_SendGoal_Response__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs__action__MoveJoint_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs__action__MoveJoint_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MoveJoint_SendGoal_Response__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Response_message_members = {
  "rcontrol_msgs__action",  // message namespace
  "MoveJoint_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(rcontrol_msgs__action__MoveJoint_SendGoal_Response),
  MoveJoint_SendGoal_Response__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Response_message_member_array,  // message members
  MoveJoint_SendGoal_Response__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveJoint_SendGoal_Response__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MoveJoint_SendGoal_Response__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Response_message_type_support_handle = {
  0,
  &MoveJoint_SendGoal_Response__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rcontrol_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcontrol_msgs, action, MoveJoint_SendGoal_Response)() {
  MoveJoint_SendGoal_Response__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!MoveJoint_SendGoal_Response__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    MoveJoint_SendGoal_Response__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MoveJoint_SendGoal_Response__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rcontrol_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers rcontrol_msgs__action__detail__move_joint__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_service_members = {
  "rcontrol_msgs__action",  // service namespace
  "MoveJoint_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // rcontrol_msgs__action__detail__move_joint__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // rcontrol_msgs__action__detail__move_joint__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t rcontrol_msgs__action__detail__move_joint__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_service_type_support_handle = {
  0,
  &rcontrol_msgs__action__detail__move_joint__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcontrol_msgs, action, MoveJoint_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcontrol_msgs, action, MoveJoint_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rcontrol_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcontrol_msgs, action, MoveJoint_SendGoal)() {
  if (!rcontrol_msgs__action__detail__move_joint__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_service_type_support_handle.typesupport_identifier) {
    rcontrol_msgs__action__detail__move_joint__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)rcontrol_msgs__action__detail__move_joint__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcontrol_msgs, action, MoveJoint_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcontrol_msgs, action, MoveJoint_SendGoal_Response)()->data;
  }

  return &rcontrol_msgs__action__detail__move_joint__rosidl_typesupport_introspection_c__MoveJoint_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rcontrol_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__functions.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MoveJoint_GetResult_Request__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rcontrol_msgs__action__MoveJoint_GetResult_Request__init(message_memory);
}

void MoveJoint_GetResult_Request__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Request_fini_function(void * message_memory)
{
  rcontrol_msgs__action__MoveJoint_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MoveJoint_GetResult_Request__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs__action__MoveJoint_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MoveJoint_GetResult_Request__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Request_message_members = {
  "rcontrol_msgs__action",  // message namespace
  "MoveJoint_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(rcontrol_msgs__action__MoveJoint_GetResult_Request),
  MoveJoint_GetResult_Request__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Request_message_member_array,  // message members
  MoveJoint_GetResult_Request__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveJoint_GetResult_Request__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MoveJoint_GetResult_Request__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Request_message_type_support_handle = {
  0,
  &MoveJoint_GetResult_Request__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rcontrol_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcontrol_msgs, action, MoveJoint_GetResult_Request)() {
  MoveJoint_GetResult_Request__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!MoveJoint_GetResult_Request__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    MoveJoint_GetResult_Request__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MoveJoint_GetResult_Request__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rcontrol_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__functions.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "rcontrol_msgs/action/move_joint.h"
// Member `result`
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MoveJoint_GetResult_Response__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rcontrol_msgs__action__MoveJoint_GetResult_Response__init(message_memory);
}

void MoveJoint_GetResult_Response__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Response_fini_function(void * message_memory)
{
  rcontrol_msgs__action__MoveJoint_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MoveJoint_GetResult_Response__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs__action__MoveJoint_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs__action__MoveJoint_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MoveJoint_GetResult_Response__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Response_message_members = {
  "rcontrol_msgs__action",  // message namespace
  "MoveJoint_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(rcontrol_msgs__action__MoveJoint_GetResult_Response),
  MoveJoint_GetResult_Response__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Response_message_member_array,  // message members
  MoveJoint_GetResult_Response__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveJoint_GetResult_Response__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MoveJoint_GetResult_Response__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Response_message_type_support_handle = {
  0,
  &MoveJoint_GetResult_Response__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rcontrol_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcontrol_msgs, action, MoveJoint_GetResult_Response)() {
  MoveJoint_GetResult_Response__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcontrol_msgs, action, MoveJoint_Result)();
  if (!MoveJoint_GetResult_Response__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    MoveJoint_GetResult_Response__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MoveJoint_GetResult_Response__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rcontrol_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers rcontrol_msgs__action__detail__move_joint__rosidl_typesupport_introspection_c__MoveJoint_GetResult_service_members = {
  "rcontrol_msgs__action",  // service namespace
  "MoveJoint_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // rcontrol_msgs__action__detail__move_joint__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // rcontrol_msgs__action__detail__move_joint__rosidl_typesupport_introspection_c__MoveJoint_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t rcontrol_msgs__action__detail__move_joint__rosidl_typesupport_introspection_c__MoveJoint_GetResult_service_type_support_handle = {
  0,
  &rcontrol_msgs__action__detail__move_joint__rosidl_typesupport_introspection_c__MoveJoint_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcontrol_msgs, action, MoveJoint_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcontrol_msgs, action, MoveJoint_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rcontrol_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcontrol_msgs, action, MoveJoint_GetResult)() {
  if (!rcontrol_msgs__action__detail__move_joint__rosidl_typesupport_introspection_c__MoveJoint_GetResult_service_type_support_handle.typesupport_identifier) {
    rcontrol_msgs__action__detail__move_joint__rosidl_typesupport_introspection_c__MoveJoint_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)rcontrol_msgs__action__detail__move_joint__rosidl_typesupport_introspection_c__MoveJoint_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcontrol_msgs, action, MoveJoint_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcontrol_msgs, action, MoveJoint_GetResult_Response)()->data;
  }

  return &rcontrol_msgs__action__detail__move_joint__rosidl_typesupport_introspection_c__MoveJoint_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rcontrol_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__functions.h"
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "rcontrol_msgs/action/move_joint.h"
// Member `feedback`
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MoveJoint_FeedbackMessage__rosidl_typesupport_introspection_c__MoveJoint_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rcontrol_msgs__action__MoveJoint_FeedbackMessage__init(message_memory);
}

void MoveJoint_FeedbackMessage__rosidl_typesupport_introspection_c__MoveJoint_FeedbackMessage_fini_function(void * message_memory)
{
  rcontrol_msgs__action__MoveJoint_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MoveJoint_FeedbackMessage__rosidl_typesupport_introspection_c__MoveJoint_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs__action__MoveJoint_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rcontrol_msgs__action__MoveJoint_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MoveJoint_FeedbackMessage__rosidl_typesupport_introspection_c__MoveJoint_FeedbackMessage_message_members = {
  "rcontrol_msgs__action",  // message namespace
  "MoveJoint_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(rcontrol_msgs__action__MoveJoint_FeedbackMessage),
  MoveJoint_FeedbackMessage__rosidl_typesupport_introspection_c__MoveJoint_FeedbackMessage_message_member_array,  // message members
  MoveJoint_FeedbackMessage__rosidl_typesupport_introspection_c__MoveJoint_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveJoint_FeedbackMessage__rosidl_typesupport_introspection_c__MoveJoint_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MoveJoint_FeedbackMessage__rosidl_typesupport_introspection_c__MoveJoint_FeedbackMessage_message_type_support_handle = {
  0,
  &MoveJoint_FeedbackMessage__rosidl_typesupport_introspection_c__MoveJoint_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rcontrol_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcontrol_msgs, action, MoveJoint_FeedbackMessage)() {
  MoveJoint_FeedbackMessage__rosidl_typesupport_introspection_c__MoveJoint_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  MoveJoint_FeedbackMessage__rosidl_typesupport_introspection_c__MoveJoint_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rcontrol_msgs, action, MoveJoint_Feedback)();
  if (!MoveJoint_FeedbackMessage__rosidl_typesupport_introspection_c__MoveJoint_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    MoveJoint_FeedbackMessage__rosidl_typesupport_introspection_c__MoveJoint_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MoveJoint_FeedbackMessage__rosidl_typesupport_introspection_c__MoveJoint_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
