// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from octomap_msgs:srv/BoundingBoxQuery.idl
// generated code does not contain a copyright notice

#ifndef OCTOMAP_MSGS__SRV__DETAIL__BOUNDING_BOX_QUERY__STRUCT_HPP_
#define OCTOMAP_MSGS__SRV__DETAIL__BOUNDING_BOX_QUERY__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'min'
// Member 'max'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__octomap_msgs__srv__BoundingBoxQuery_Request __attribute__((deprecated))
#else
# define DEPRECATED__octomap_msgs__srv__BoundingBoxQuery_Request __declspec(deprecated)
#endif

namespace octomap_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct BoundingBoxQuery_Request_
{
  using Type = BoundingBoxQuery_Request_<ContainerAllocator>;

  explicit BoundingBoxQuery_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : min(_init),
    max(_init)
  {
    (void)_init;
  }

  explicit BoundingBoxQuery_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : min(_alloc, _init),
    max(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _min_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _min_type min;
  using _max_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _max_type max;

  // setters for named parameter idiom
  Type & set__min(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->min = _arg;
    return *this;
  }
  Type & set__max(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->max = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    octomap_msgs::srv::BoundingBoxQuery_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const octomap_msgs::srv::BoundingBoxQuery_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<octomap_msgs::srv::BoundingBoxQuery_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<octomap_msgs::srv::BoundingBoxQuery_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      octomap_msgs::srv::BoundingBoxQuery_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<octomap_msgs::srv::BoundingBoxQuery_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      octomap_msgs::srv::BoundingBoxQuery_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<octomap_msgs::srv::BoundingBoxQuery_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<octomap_msgs::srv::BoundingBoxQuery_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<octomap_msgs::srv::BoundingBoxQuery_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__octomap_msgs__srv__BoundingBoxQuery_Request
    std::shared_ptr<octomap_msgs::srv::BoundingBoxQuery_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__octomap_msgs__srv__BoundingBoxQuery_Request
    std::shared_ptr<octomap_msgs::srv::BoundingBoxQuery_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BoundingBoxQuery_Request_ & other) const
  {
    if (this->min != other.min) {
      return false;
    }
    if (this->max != other.max) {
      return false;
    }
    return true;
  }
  bool operator!=(const BoundingBoxQuery_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BoundingBoxQuery_Request_

// alias to use template instance with default allocator
using BoundingBoxQuery_Request =
  octomap_msgs::srv::BoundingBoxQuery_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace octomap_msgs


#ifndef _WIN32
# define DEPRECATED__octomap_msgs__srv__BoundingBoxQuery_Response __attribute__((deprecated))
#else
# define DEPRECATED__octomap_msgs__srv__BoundingBoxQuery_Response __declspec(deprecated)
#endif

namespace octomap_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct BoundingBoxQuery_Response_
{
  using Type = BoundingBoxQuery_Response_<ContainerAllocator>;

  explicit BoundingBoxQuery_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit BoundingBoxQuery_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    octomap_msgs::srv::BoundingBoxQuery_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const octomap_msgs::srv::BoundingBoxQuery_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<octomap_msgs::srv::BoundingBoxQuery_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<octomap_msgs::srv::BoundingBoxQuery_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      octomap_msgs::srv::BoundingBoxQuery_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<octomap_msgs::srv::BoundingBoxQuery_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      octomap_msgs::srv::BoundingBoxQuery_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<octomap_msgs::srv::BoundingBoxQuery_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<octomap_msgs::srv::BoundingBoxQuery_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<octomap_msgs::srv::BoundingBoxQuery_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__octomap_msgs__srv__BoundingBoxQuery_Response
    std::shared_ptr<octomap_msgs::srv::BoundingBoxQuery_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__octomap_msgs__srv__BoundingBoxQuery_Response
    std::shared_ptr<octomap_msgs::srv::BoundingBoxQuery_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BoundingBoxQuery_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const BoundingBoxQuery_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BoundingBoxQuery_Response_

// alias to use template instance with default allocator
using BoundingBoxQuery_Response =
  octomap_msgs::srv::BoundingBoxQuery_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace octomap_msgs

namespace octomap_msgs
{

namespace srv
{

struct BoundingBoxQuery
{
  using Request = octomap_msgs::srv::BoundingBoxQuery_Request;
  using Response = octomap_msgs::srv::BoundingBoxQuery_Response;
};

}  // namespace srv

}  // namespace octomap_msgs

#endif  // OCTOMAP_MSGS__SRV__DETAIL__BOUNDING_BOX_QUERY__STRUCT_HPP_
