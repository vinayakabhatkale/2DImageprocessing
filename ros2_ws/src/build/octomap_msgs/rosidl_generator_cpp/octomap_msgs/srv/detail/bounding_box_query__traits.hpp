// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from octomap_msgs:srv/BoundingBoxQuery.idl
// generated code does not contain a copyright notice

#ifndef OCTOMAP_MSGS__SRV__DETAIL__BOUNDING_BOX_QUERY__TRAITS_HPP_
#define OCTOMAP_MSGS__SRV__DETAIL__BOUNDING_BOX_QUERY__TRAITS_HPP_

#include "octomap_msgs/srv/detail/bounding_box_query__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'min'
// Member 'max'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<octomap_msgs::srv::BoundingBoxQuery_Request>()
{
  return "octomap_msgs::srv::BoundingBoxQuery_Request";
}

template<>
inline const char * name<octomap_msgs::srv::BoundingBoxQuery_Request>()
{
  return "octomap_msgs/srv/BoundingBoxQuery_Request";
}

template<>
struct has_fixed_size<octomap_msgs::srv::BoundingBoxQuery_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value> {};

template<>
struct has_bounded_size<octomap_msgs::srv::BoundingBoxQuery_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value> {};

template<>
struct is_message<octomap_msgs::srv::BoundingBoxQuery_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<octomap_msgs::srv::BoundingBoxQuery_Response>()
{
  return "octomap_msgs::srv::BoundingBoxQuery_Response";
}

template<>
inline const char * name<octomap_msgs::srv::BoundingBoxQuery_Response>()
{
  return "octomap_msgs/srv/BoundingBoxQuery_Response";
}

template<>
struct has_fixed_size<octomap_msgs::srv::BoundingBoxQuery_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<octomap_msgs::srv::BoundingBoxQuery_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<octomap_msgs::srv::BoundingBoxQuery_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<octomap_msgs::srv::BoundingBoxQuery>()
{
  return "octomap_msgs::srv::BoundingBoxQuery";
}

template<>
inline const char * name<octomap_msgs::srv::BoundingBoxQuery>()
{
  return "octomap_msgs/srv/BoundingBoxQuery";
}

template<>
struct has_fixed_size<octomap_msgs::srv::BoundingBoxQuery>
  : std::integral_constant<
    bool,
    has_fixed_size<octomap_msgs::srv::BoundingBoxQuery_Request>::value &&
    has_fixed_size<octomap_msgs::srv::BoundingBoxQuery_Response>::value
  >
{
};

template<>
struct has_bounded_size<octomap_msgs::srv::BoundingBoxQuery>
  : std::integral_constant<
    bool,
    has_bounded_size<octomap_msgs::srv::BoundingBoxQuery_Request>::value &&
    has_bounded_size<octomap_msgs::srv::BoundingBoxQuery_Response>::value
  >
{
};

template<>
struct is_service<octomap_msgs::srv::BoundingBoxQuery>
  : std::true_type
{
};

template<>
struct is_service_request<octomap_msgs::srv::BoundingBoxQuery_Request>
  : std::true_type
{
};

template<>
struct is_service_response<octomap_msgs::srv::BoundingBoxQuery_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // OCTOMAP_MSGS__SRV__DETAIL__BOUNDING_BOX_QUERY__TRAITS_HPP_
