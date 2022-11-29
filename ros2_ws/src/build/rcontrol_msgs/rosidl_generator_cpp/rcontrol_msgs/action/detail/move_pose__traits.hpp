// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rcontrol_msgs:action/MovePose.idl
// generated code does not contain a copyright notice

#ifndef RCONTROL_MSGS__ACTION__DETAIL__MOVE_POSE__TRAITS_HPP_
#define RCONTROL_MSGS__ACTION__DETAIL__MOVE_POSE__TRAITS_HPP_

#include "rcontrol_msgs/action/detail/move_pose__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rcontrol_msgs::action::MovePose_Goal>()
{
  return "rcontrol_msgs::action::MovePose_Goal";
}

template<>
inline const char * name<rcontrol_msgs::action::MovePose_Goal>()
{
  return "rcontrol_msgs/action/MovePose_Goal";
}

template<>
struct has_fixed_size<rcontrol_msgs::action::MovePose_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rcontrol_msgs::action::MovePose_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rcontrol_msgs::action::MovePose_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rcontrol_msgs::action::MovePose_Result>()
{
  return "rcontrol_msgs::action::MovePose_Result";
}

template<>
inline const char * name<rcontrol_msgs::action::MovePose_Result>()
{
  return "rcontrol_msgs/action/MovePose_Result";
}

template<>
struct has_fixed_size<rcontrol_msgs::action::MovePose_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rcontrol_msgs::action::MovePose_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rcontrol_msgs::action::MovePose_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rcontrol_msgs::action::MovePose_Feedback>()
{
  return "rcontrol_msgs::action::MovePose_Feedback";
}

template<>
inline const char * name<rcontrol_msgs::action::MovePose_Feedback>()
{
  return "rcontrol_msgs/action/MovePose_Feedback";
}

template<>
struct has_fixed_size<rcontrol_msgs::action::MovePose_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rcontrol_msgs::action::MovePose_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rcontrol_msgs::action::MovePose_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "rcontrol_msgs/action/detail/move_pose__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rcontrol_msgs::action::MovePose_SendGoal_Request>()
{
  return "rcontrol_msgs::action::MovePose_SendGoal_Request";
}

template<>
inline const char * name<rcontrol_msgs::action::MovePose_SendGoal_Request>()
{
  return "rcontrol_msgs/action/MovePose_SendGoal_Request";
}

template<>
struct has_fixed_size<rcontrol_msgs::action::MovePose_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<rcontrol_msgs::action::MovePose_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<rcontrol_msgs::action::MovePose_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<rcontrol_msgs::action::MovePose_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<rcontrol_msgs::action::MovePose_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rcontrol_msgs::action::MovePose_SendGoal_Response>()
{
  return "rcontrol_msgs::action::MovePose_SendGoal_Response";
}

template<>
inline const char * name<rcontrol_msgs::action::MovePose_SendGoal_Response>()
{
  return "rcontrol_msgs/action/MovePose_SendGoal_Response";
}

template<>
struct has_fixed_size<rcontrol_msgs::action::MovePose_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<rcontrol_msgs::action::MovePose_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<rcontrol_msgs::action::MovePose_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rcontrol_msgs::action::MovePose_SendGoal>()
{
  return "rcontrol_msgs::action::MovePose_SendGoal";
}

template<>
inline const char * name<rcontrol_msgs::action::MovePose_SendGoal>()
{
  return "rcontrol_msgs/action/MovePose_SendGoal";
}

template<>
struct has_fixed_size<rcontrol_msgs::action::MovePose_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<rcontrol_msgs::action::MovePose_SendGoal_Request>::value &&
    has_fixed_size<rcontrol_msgs::action::MovePose_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<rcontrol_msgs::action::MovePose_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<rcontrol_msgs::action::MovePose_SendGoal_Request>::value &&
    has_bounded_size<rcontrol_msgs::action::MovePose_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<rcontrol_msgs::action::MovePose_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<rcontrol_msgs::action::MovePose_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rcontrol_msgs::action::MovePose_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rcontrol_msgs::action::MovePose_GetResult_Request>()
{
  return "rcontrol_msgs::action::MovePose_GetResult_Request";
}

template<>
inline const char * name<rcontrol_msgs::action::MovePose_GetResult_Request>()
{
  return "rcontrol_msgs/action/MovePose_GetResult_Request";
}

template<>
struct has_fixed_size<rcontrol_msgs::action::MovePose_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<rcontrol_msgs::action::MovePose_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<rcontrol_msgs::action::MovePose_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "rcontrol_msgs/action/detail/move_pose__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rcontrol_msgs::action::MovePose_GetResult_Response>()
{
  return "rcontrol_msgs::action::MovePose_GetResult_Response";
}

template<>
inline const char * name<rcontrol_msgs::action::MovePose_GetResult_Response>()
{
  return "rcontrol_msgs/action/MovePose_GetResult_Response";
}

template<>
struct has_fixed_size<rcontrol_msgs::action::MovePose_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<rcontrol_msgs::action::MovePose_Result>::value> {};

template<>
struct has_bounded_size<rcontrol_msgs::action::MovePose_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<rcontrol_msgs::action::MovePose_Result>::value> {};

template<>
struct is_message<rcontrol_msgs::action::MovePose_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rcontrol_msgs::action::MovePose_GetResult>()
{
  return "rcontrol_msgs::action::MovePose_GetResult";
}

template<>
inline const char * name<rcontrol_msgs::action::MovePose_GetResult>()
{
  return "rcontrol_msgs/action/MovePose_GetResult";
}

template<>
struct has_fixed_size<rcontrol_msgs::action::MovePose_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<rcontrol_msgs::action::MovePose_GetResult_Request>::value &&
    has_fixed_size<rcontrol_msgs::action::MovePose_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<rcontrol_msgs::action::MovePose_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<rcontrol_msgs::action::MovePose_GetResult_Request>::value &&
    has_bounded_size<rcontrol_msgs::action::MovePose_GetResult_Response>::value
  >
{
};

template<>
struct is_service<rcontrol_msgs::action::MovePose_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<rcontrol_msgs::action::MovePose_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rcontrol_msgs::action::MovePose_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "rcontrol_msgs/action/detail/move_pose__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rcontrol_msgs::action::MovePose_FeedbackMessage>()
{
  return "rcontrol_msgs::action::MovePose_FeedbackMessage";
}

template<>
inline const char * name<rcontrol_msgs::action::MovePose_FeedbackMessage>()
{
  return "rcontrol_msgs/action/MovePose_FeedbackMessage";
}

template<>
struct has_fixed_size<rcontrol_msgs::action::MovePose_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<rcontrol_msgs::action::MovePose_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<rcontrol_msgs::action::MovePose_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<rcontrol_msgs::action::MovePose_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<rcontrol_msgs::action::MovePose_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<rcontrol_msgs::action::MovePose>
  : std::true_type
{
};

template<>
struct is_action_goal<rcontrol_msgs::action::MovePose_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<rcontrol_msgs::action::MovePose_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<rcontrol_msgs::action::MovePose_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // RCONTROL_MSGS__ACTION__DETAIL__MOVE_POSE__TRAITS_HPP_
