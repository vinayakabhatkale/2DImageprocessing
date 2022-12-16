// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from octomap_msgs:msg/Octomap.idl
// generated code does not contain a copyright notice

#ifndef OCTOMAP_MSGS__MSG__DETAIL__OCTOMAP__FUNCTIONS_H_
#define OCTOMAP_MSGS__MSG__DETAIL__OCTOMAP__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "octomap_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "octomap_msgs/msg/detail/octomap__struct.h"

/// Initialize msg/Octomap message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * octomap_msgs__msg__Octomap
 * )) before or use
 * octomap_msgs__msg__Octomap__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_octomap_msgs
bool
octomap_msgs__msg__Octomap__init(octomap_msgs__msg__Octomap * msg);

/// Finalize msg/Octomap message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_octomap_msgs
void
octomap_msgs__msg__Octomap__fini(octomap_msgs__msg__Octomap * msg);

/// Create msg/Octomap message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * octomap_msgs__msg__Octomap__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_octomap_msgs
octomap_msgs__msg__Octomap *
octomap_msgs__msg__Octomap__create();

/// Destroy msg/Octomap message.
/**
 * It calls
 * octomap_msgs__msg__Octomap__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_octomap_msgs
void
octomap_msgs__msg__Octomap__destroy(octomap_msgs__msg__Octomap * msg);


/// Initialize array of msg/Octomap messages.
/**
 * It allocates the memory for the number of elements and calls
 * octomap_msgs__msg__Octomap__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_octomap_msgs
bool
octomap_msgs__msg__Octomap__Sequence__init(octomap_msgs__msg__Octomap__Sequence * array, size_t size);

/// Finalize array of msg/Octomap messages.
/**
 * It calls
 * octomap_msgs__msg__Octomap__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_octomap_msgs
void
octomap_msgs__msg__Octomap__Sequence__fini(octomap_msgs__msg__Octomap__Sequence * array);

/// Create array of msg/Octomap messages.
/**
 * It allocates the memory for the array and calls
 * octomap_msgs__msg__Octomap__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_octomap_msgs
octomap_msgs__msg__Octomap__Sequence *
octomap_msgs__msg__Octomap__Sequence__create(size_t size);

/// Destroy array of msg/Octomap messages.
/**
 * It calls
 * octomap_msgs__msg__Octomap__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_octomap_msgs
void
octomap_msgs__msg__Octomap__Sequence__destroy(octomap_msgs__msg__Octomap__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // OCTOMAP_MSGS__MSG__DETAIL__OCTOMAP__FUNCTIONS_H_
