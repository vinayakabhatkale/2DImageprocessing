// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from octomap_msgs:msg/Octomap.idl
// generated code does not contain a copyright notice

#ifndef OCTOMAP_MSGS__MSG__DETAIL__OCTOMAP__TRAITS_HPP_
#define OCTOMAP_MSGS__MSG__DETAIL__OCTOMAP__TRAITS_HPP_

#include "octomap_msgs/msg/detail/octomap__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<octomap_msgs::msg::Octomap>()
{
  return "octomap_msgs::msg::Octomap";
}

template<>
inline const char * name<octomap_msgs::msg::Octomap>()
{
  return "octomap_msgs/msg/Octomap";
}

template<>
struct has_fixed_size<octomap_msgs::msg::Octomap>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<octomap_msgs::msg::Octomap>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<octomap_msgs::msg::Octomap>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // OCTOMAP_MSGS__MSG__DETAIL__OCTOMAP__TRAITS_HPP_
