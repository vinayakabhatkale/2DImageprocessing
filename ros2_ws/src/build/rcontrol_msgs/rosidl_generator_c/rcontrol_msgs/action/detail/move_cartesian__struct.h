// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rcontrol_msgs:action/MoveCartesian.idl
// generated code does not contain a copyright notice

#ifndef RCONTROL_MSGS__ACTION__DETAIL__MOVE_CARTESIAN__STRUCT_H_
#define RCONTROL_MSGS__ACTION__DETAIL__MOVE_CARTESIAN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'posevalues'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in action/MoveCartesian in the package rcontrol_msgs.
typedef struct rcontrol_msgs__action__MoveCartesian_Goal
{
  rosidl_runtime_c__double__Sequence posevalues;
} rcontrol_msgs__action__MoveCartesian_Goal;

// Struct for a sequence of rcontrol_msgs__action__MoveCartesian_Goal.
typedef struct rcontrol_msgs__action__MoveCartesian_Goal__Sequence
{
  rcontrol_msgs__action__MoveCartesian_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcontrol_msgs__action__MoveCartesian_Goal__Sequence;


// Constants defined in the message

// Struct defined in action/MoveCartesian in the package rcontrol_msgs.
typedef struct rcontrol_msgs__action__MoveCartesian_Result
{
  bool success;
} rcontrol_msgs__action__MoveCartesian_Result;

// Struct for a sequence of rcontrol_msgs__action__MoveCartesian_Result.
typedef struct rcontrol_msgs__action__MoveCartesian_Result__Sequence
{
  rcontrol_msgs__action__MoveCartesian_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcontrol_msgs__action__MoveCartesian_Result__Sequence;


// Constants defined in the message

// Struct defined in action/MoveCartesian in the package rcontrol_msgs.
typedef struct rcontrol_msgs__action__MoveCartesian_Feedback
{
  int8_t status;
} rcontrol_msgs__action__MoveCartesian_Feedback;

// Struct for a sequence of rcontrol_msgs__action__MoveCartesian_Feedback.
typedef struct rcontrol_msgs__action__MoveCartesian_Feedback__Sequence
{
  rcontrol_msgs__action__MoveCartesian_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcontrol_msgs__action__MoveCartesian_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "rcontrol_msgs/action/detail/move_cartesian__struct.h"

// Struct defined in action/MoveCartesian in the package rcontrol_msgs.
typedef struct rcontrol_msgs__action__MoveCartesian_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  rcontrol_msgs__action__MoveCartesian_Goal goal;
} rcontrol_msgs__action__MoveCartesian_SendGoal_Request;

// Struct for a sequence of rcontrol_msgs__action__MoveCartesian_SendGoal_Request.
typedef struct rcontrol_msgs__action__MoveCartesian_SendGoal_Request__Sequence
{
  rcontrol_msgs__action__MoveCartesian_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcontrol_msgs__action__MoveCartesian_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

// Struct defined in action/MoveCartesian in the package rcontrol_msgs.
typedef struct rcontrol_msgs__action__MoveCartesian_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} rcontrol_msgs__action__MoveCartesian_SendGoal_Response;

// Struct for a sequence of rcontrol_msgs__action__MoveCartesian_SendGoal_Response.
typedef struct rcontrol_msgs__action__MoveCartesian_SendGoal_Response__Sequence
{
  rcontrol_msgs__action__MoveCartesian_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcontrol_msgs__action__MoveCartesian_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

// Struct defined in action/MoveCartesian in the package rcontrol_msgs.
typedef struct rcontrol_msgs__action__MoveCartesian_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} rcontrol_msgs__action__MoveCartesian_GetResult_Request;

// Struct for a sequence of rcontrol_msgs__action__MoveCartesian_GetResult_Request.
typedef struct rcontrol_msgs__action__MoveCartesian_GetResult_Request__Sequence
{
  rcontrol_msgs__action__MoveCartesian_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcontrol_msgs__action__MoveCartesian_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "rcontrol_msgs/action/detail/move_cartesian__struct.h"

// Struct defined in action/MoveCartesian in the package rcontrol_msgs.
typedef struct rcontrol_msgs__action__MoveCartesian_GetResult_Response
{
  int8_t status;
  rcontrol_msgs__action__MoveCartesian_Result result;
} rcontrol_msgs__action__MoveCartesian_GetResult_Response;

// Struct for a sequence of rcontrol_msgs__action__MoveCartesian_GetResult_Response.
typedef struct rcontrol_msgs__action__MoveCartesian_GetResult_Response__Sequence
{
  rcontrol_msgs__action__MoveCartesian_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcontrol_msgs__action__MoveCartesian_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "rcontrol_msgs/action/detail/move_cartesian__struct.h"

// Struct defined in action/MoveCartesian in the package rcontrol_msgs.
typedef struct rcontrol_msgs__action__MoveCartesian_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  rcontrol_msgs__action__MoveCartesian_Feedback feedback;
} rcontrol_msgs__action__MoveCartesian_FeedbackMessage;

// Struct for a sequence of rcontrol_msgs__action__MoveCartesian_FeedbackMessage.
typedef struct rcontrol_msgs__action__MoveCartesian_FeedbackMessage__Sequence
{
  rcontrol_msgs__action__MoveCartesian_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rcontrol_msgs__action__MoveCartesian_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RCONTROL_MSGS__ACTION__DETAIL__MOVE_CARTESIAN__STRUCT_H_
