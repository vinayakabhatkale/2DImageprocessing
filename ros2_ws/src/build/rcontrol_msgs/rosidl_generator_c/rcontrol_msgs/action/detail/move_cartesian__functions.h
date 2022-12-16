// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rcontrol_msgs:action/MoveCartesian.idl
// generated code does not contain a copyright notice

#ifndef RCONTROL_MSGS__ACTION__DETAIL__MOVE_CARTESIAN__FUNCTIONS_H_
#define RCONTROL_MSGS__ACTION__DETAIL__MOVE_CARTESIAN__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rcontrol_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "rcontrol_msgs/action/detail/move_cartesian__struct.h"

/// Initialize action/MoveCartesian message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rcontrol_msgs__action__MoveCartesian_Goal
 * )) before or use
 * rcontrol_msgs__action__MoveCartesian_Goal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
bool
rcontrol_msgs__action__MoveCartesian_Goal__init(rcontrol_msgs__action__MoveCartesian_Goal * msg);

/// Finalize action/MoveCartesian message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_Goal__fini(rcontrol_msgs__action__MoveCartesian_Goal * msg);

/// Create action/MoveCartesian message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rcontrol_msgs__action__MoveCartesian_Goal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
rcontrol_msgs__action__MoveCartesian_Goal *
rcontrol_msgs__action__MoveCartesian_Goal__create();

/// Destroy action/MoveCartesian message.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_Goal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_Goal__destroy(rcontrol_msgs__action__MoveCartesian_Goal * msg);


/// Initialize array of action/MoveCartesian messages.
/**
 * It allocates the memory for the number of elements and calls
 * rcontrol_msgs__action__MoveCartesian_Goal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
bool
rcontrol_msgs__action__MoveCartesian_Goal__Sequence__init(rcontrol_msgs__action__MoveCartesian_Goal__Sequence * array, size_t size);

/// Finalize array of action/MoveCartesian messages.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_Goal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_Goal__Sequence__fini(rcontrol_msgs__action__MoveCartesian_Goal__Sequence * array);

/// Create array of action/MoveCartesian messages.
/**
 * It allocates the memory for the array and calls
 * rcontrol_msgs__action__MoveCartesian_Goal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
rcontrol_msgs__action__MoveCartesian_Goal__Sequence *
rcontrol_msgs__action__MoveCartesian_Goal__Sequence__create(size_t size);

/// Destroy array of action/MoveCartesian messages.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_Goal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_Goal__Sequence__destroy(rcontrol_msgs__action__MoveCartesian_Goal__Sequence * array);

/// Initialize action/MoveCartesian message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rcontrol_msgs__action__MoveCartesian_Result
 * )) before or use
 * rcontrol_msgs__action__MoveCartesian_Result__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
bool
rcontrol_msgs__action__MoveCartesian_Result__init(rcontrol_msgs__action__MoveCartesian_Result * msg);

/// Finalize action/MoveCartesian message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_Result__fini(rcontrol_msgs__action__MoveCartesian_Result * msg);

/// Create action/MoveCartesian message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rcontrol_msgs__action__MoveCartesian_Result__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
rcontrol_msgs__action__MoveCartesian_Result *
rcontrol_msgs__action__MoveCartesian_Result__create();

/// Destroy action/MoveCartesian message.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_Result__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_Result__destroy(rcontrol_msgs__action__MoveCartesian_Result * msg);


/// Initialize array of action/MoveCartesian messages.
/**
 * It allocates the memory for the number of elements and calls
 * rcontrol_msgs__action__MoveCartesian_Result__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
bool
rcontrol_msgs__action__MoveCartesian_Result__Sequence__init(rcontrol_msgs__action__MoveCartesian_Result__Sequence * array, size_t size);

/// Finalize array of action/MoveCartesian messages.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_Result__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_Result__Sequence__fini(rcontrol_msgs__action__MoveCartesian_Result__Sequence * array);

/// Create array of action/MoveCartesian messages.
/**
 * It allocates the memory for the array and calls
 * rcontrol_msgs__action__MoveCartesian_Result__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
rcontrol_msgs__action__MoveCartesian_Result__Sequence *
rcontrol_msgs__action__MoveCartesian_Result__Sequence__create(size_t size);

/// Destroy array of action/MoveCartesian messages.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_Result__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_Result__Sequence__destroy(rcontrol_msgs__action__MoveCartesian_Result__Sequence * array);

/// Initialize action/MoveCartesian message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rcontrol_msgs__action__MoveCartesian_Feedback
 * )) before or use
 * rcontrol_msgs__action__MoveCartesian_Feedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
bool
rcontrol_msgs__action__MoveCartesian_Feedback__init(rcontrol_msgs__action__MoveCartesian_Feedback * msg);

/// Finalize action/MoveCartesian message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_Feedback__fini(rcontrol_msgs__action__MoveCartesian_Feedback * msg);

/// Create action/MoveCartesian message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rcontrol_msgs__action__MoveCartesian_Feedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
rcontrol_msgs__action__MoveCartesian_Feedback *
rcontrol_msgs__action__MoveCartesian_Feedback__create();

/// Destroy action/MoveCartesian message.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_Feedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_Feedback__destroy(rcontrol_msgs__action__MoveCartesian_Feedback * msg);


/// Initialize array of action/MoveCartesian messages.
/**
 * It allocates the memory for the number of elements and calls
 * rcontrol_msgs__action__MoveCartesian_Feedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
bool
rcontrol_msgs__action__MoveCartesian_Feedback__Sequence__init(rcontrol_msgs__action__MoveCartesian_Feedback__Sequence * array, size_t size);

/// Finalize array of action/MoveCartesian messages.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_Feedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_Feedback__Sequence__fini(rcontrol_msgs__action__MoveCartesian_Feedback__Sequence * array);

/// Create array of action/MoveCartesian messages.
/**
 * It allocates the memory for the array and calls
 * rcontrol_msgs__action__MoveCartesian_Feedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
rcontrol_msgs__action__MoveCartesian_Feedback__Sequence *
rcontrol_msgs__action__MoveCartesian_Feedback__Sequence__create(size_t size);

/// Destroy array of action/MoveCartesian messages.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_Feedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_Feedback__Sequence__destroy(rcontrol_msgs__action__MoveCartesian_Feedback__Sequence * array);

/// Initialize action/MoveCartesian message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rcontrol_msgs__action__MoveCartesian_SendGoal_Request
 * )) before or use
 * rcontrol_msgs__action__MoveCartesian_SendGoal_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
bool
rcontrol_msgs__action__MoveCartesian_SendGoal_Request__init(rcontrol_msgs__action__MoveCartesian_SendGoal_Request * msg);

/// Finalize action/MoveCartesian message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_SendGoal_Request__fini(rcontrol_msgs__action__MoveCartesian_SendGoal_Request * msg);

/// Create action/MoveCartesian message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rcontrol_msgs__action__MoveCartesian_SendGoal_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
rcontrol_msgs__action__MoveCartesian_SendGoal_Request *
rcontrol_msgs__action__MoveCartesian_SendGoal_Request__create();

/// Destroy action/MoveCartesian message.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_SendGoal_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_SendGoal_Request__destroy(rcontrol_msgs__action__MoveCartesian_SendGoal_Request * msg);


/// Initialize array of action/MoveCartesian messages.
/**
 * It allocates the memory for the number of elements and calls
 * rcontrol_msgs__action__MoveCartesian_SendGoal_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
bool
rcontrol_msgs__action__MoveCartesian_SendGoal_Request__Sequence__init(rcontrol_msgs__action__MoveCartesian_SendGoal_Request__Sequence * array, size_t size);

/// Finalize array of action/MoveCartesian messages.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_SendGoal_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_SendGoal_Request__Sequence__fini(rcontrol_msgs__action__MoveCartesian_SendGoal_Request__Sequence * array);

/// Create array of action/MoveCartesian messages.
/**
 * It allocates the memory for the array and calls
 * rcontrol_msgs__action__MoveCartesian_SendGoal_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
rcontrol_msgs__action__MoveCartesian_SendGoal_Request__Sequence *
rcontrol_msgs__action__MoveCartesian_SendGoal_Request__Sequence__create(size_t size);

/// Destroy array of action/MoveCartesian messages.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_SendGoal_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_SendGoal_Request__Sequence__destroy(rcontrol_msgs__action__MoveCartesian_SendGoal_Request__Sequence * array);

/// Initialize action/MoveCartesian message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rcontrol_msgs__action__MoveCartesian_SendGoal_Response
 * )) before or use
 * rcontrol_msgs__action__MoveCartesian_SendGoal_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
bool
rcontrol_msgs__action__MoveCartesian_SendGoal_Response__init(rcontrol_msgs__action__MoveCartesian_SendGoal_Response * msg);

/// Finalize action/MoveCartesian message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_SendGoal_Response__fini(rcontrol_msgs__action__MoveCartesian_SendGoal_Response * msg);

/// Create action/MoveCartesian message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rcontrol_msgs__action__MoveCartesian_SendGoal_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
rcontrol_msgs__action__MoveCartesian_SendGoal_Response *
rcontrol_msgs__action__MoveCartesian_SendGoal_Response__create();

/// Destroy action/MoveCartesian message.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_SendGoal_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_SendGoal_Response__destroy(rcontrol_msgs__action__MoveCartesian_SendGoal_Response * msg);


/// Initialize array of action/MoveCartesian messages.
/**
 * It allocates the memory for the number of elements and calls
 * rcontrol_msgs__action__MoveCartesian_SendGoal_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
bool
rcontrol_msgs__action__MoveCartesian_SendGoal_Response__Sequence__init(rcontrol_msgs__action__MoveCartesian_SendGoal_Response__Sequence * array, size_t size);

/// Finalize array of action/MoveCartesian messages.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_SendGoal_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_SendGoal_Response__Sequence__fini(rcontrol_msgs__action__MoveCartesian_SendGoal_Response__Sequence * array);

/// Create array of action/MoveCartesian messages.
/**
 * It allocates the memory for the array and calls
 * rcontrol_msgs__action__MoveCartesian_SendGoal_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
rcontrol_msgs__action__MoveCartesian_SendGoal_Response__Sequence *
rcontrol_msgs__action__MoveCartesian_SendGoal_Response__Sequence__create(size_t size);

/// Destroy array of action/MoveCartesian messages.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_SendGoal_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_SendGoal_Response__Sequence__destroy(rcontrol_msgs__action__MoveCartesian_SendGoal_Response__Sequence * array);

/// Initialize action/MoveCartesian message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rcontrol_msgs__action__MoveCartesian_GetResult_Request
 * )) before or use
 * rcontrol_msgs__action__MoveCartesian_GetResult_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
bool
rcontrol_msgs__action__MoveCartesian_GetResult_Request__init(rcontrol_msgs__action__MoveCartesian_GetResult_Request * msg);

/// Finalize action/MoveCartesian message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_GetResult_Request__fini(rcontrol_msgs__action__MoveCartesian_GetResult_Request * msg);

/// Create action/MoveCartesian message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rcontrol_msgs__action__MoveCartesian_GetResult_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
rcontrol_msgs__action__MoveCartesian_GetResult_Request *
rcontrol_msgs__action__MoveCartesian_GetResult_Request__create();

/// Destroy action/MoveCartesian message.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_GetResult_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_GetResult_Request__destroy(rcontrol_msgs__action__MoveCartesian_GetResult_Request * msg);


/// Initialize array of action/MoveCartesian messages.
/**
 * It allocates the memory for the number of elements and calls
 * rcontrol_msgs__action__MoveCartesian_GetResult_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
bool
rcontrol_msgs__action__MoveCartesian_GetResult_Request__Sequence__init(rcontrol_msgs__action__MoveCartesian_GetResult_Request__Sequence * array, size_t size);

/// Finalize array of action/MoveCartesian messages.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_GetResult_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_GetResult_Request__Sequence__fini(rcontrol_msgs__action__MoveCartesian_GetResult_Request__Sequence * array);

/// Create array of action/MoveCartesian messages.
/**
 * It allocates the memory for the array and calls
 * rcontrol_msgs__action__MoveCartesian_GetResult_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
rcontrol_msgs__action__MoveCartesian_GetResult_Request__Sequence *
rcontrol_msgs__action__MoveCartesian_GetResult_Request__Sequence__create(size_t size);

/// Destroy array of action/MoveCartesian messages.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_GetResult_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_GetResult_Request__Sequence__destroy(rcontrol_msgs__action__MoveCartesian_GetResult_Request__Sequence * array);

/// Initialize action/MoveCartesian message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rcontrol_msgs__action__MoveCartesian_GetResult_Response
 * )) before or use
 * rcontrol_msgs__action__MoveCartesian_GetResult_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
bool
rcontrol_msgs__action__MoveCartesian_GetResult_Response__init(rcontrol_msgs__action__MoveCartesian_GetResult_Response * msg);

/// Finalize action/MoveCartesian message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_GetResult_Response__fini(rcontrol_msgs__action__MoveCartesian_GetResult_Response * msg);

/// Create action/MoveCartesian message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rcontrol_msgs__action__MoveCartesian_GetResult_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
rcontrol_msgs__action__MoveCartesian_GetResult_Response *
rcontrol_msgs__action__MoveCartesian_GetResult_Response__create();

/// Destroy action/MoveCartesian message.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_GetResult_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_GetResult_Response__destroy(rcontrol_msgs__action__MoveCartesian_GetResult_Response * msg);


/// Initialize array of action/MoveCartesian messages.
/**
 * It allocates the memory for the number of elements and calls
 * rcontrol_msgs__action__MoveCartesian_GetResult_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
bool
rcontrol_msgs__action__MoveCartesian_GetResult_Response__Sequence__init(rcontrol_msgs__action__MoveCartesian_GetResult_Response__Sequence * array, size_t size);

/// Finalize array of action/MoveCartesian messages.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_GetResult_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_GetResult_Response__Sequence__fini(rcontrol_msgs__action__MoveCartesian_GetResult_Response__Sequence * array);

/// Create array of action/MoveCartesian messages.
/**
 * It allocates the memory for the array and calls
 * rcontrol_msgs__action__MoveCartesian_GetResult_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
rcontrol_msgs__action__MoveCartesian_GetResult_Response__Sequence *
rcontrol_msgs__action__MoveCartesian_GetResult_Response__Sequence__create(size_t size);

/// Destroy array of action/MoveCartesian messages.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_GetResult_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_GetResult_Response__Sequence__destroy(rcontrol_msgs__action__MoveCartesian_GetResult_Response__Sequence * array);

/// Initialize action/MoveCartesian message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rcontrol_msgs__action__MoveCartesian_FeedbackMessage
 * )) before or use
 * rcontrol_msgs__action__MoveCartesian_FeedbackMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
bool
rcontrol_msgs__action__MoveCartesian_FeedbackMessage__init(rcontrol_msgs__action__MoveCartesian_FeedbackMessage * msg);

/// Finalize action/MoveCartesian message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_FeedbackMessage__fini(rcontrol_msgs__action__MoveCartesian_FeedbackMessage * msg);

/// Create action/MoveCartesian message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rcontrol_msgs__action__MoveCartesian_FeedbackMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
rcontrol_msgs__action__MoveCartesian_FeedbackMessage *
rcontrol_msgs__action__MoveCartesian_FeedbackMessage__create();

/// Destroy action/MoveCartesian message.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_FeedbackMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_FeedbackMessage__destroy(rcontrol_msgs__action__MoveCartesian_FeedbackMessage * msg);


/// Initialize array of action/MoveCartesian messages.
/**
 * It allocates the memory for the number of elements and calls
 * rcontrol_msgs__action__MoveCartesian_FeedbackMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
bool
rcontrol_msgs__action__MoveCartesian_FeedbackMessage__Sequence__init(rcontrol_msgs__action__MoveCartesian_FeedbackMessage__Sequence * array, size_t size);

/// Finalize array of action/MoveCartesian messages.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_FeedbackMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_FeedbackMessage__Sequence__fini(rcontrol_msgs__action__MoveCartesian_FeedbackMessage__Sequence * array);

/// Create array of action/MoveCartesian messages.
/**
 * It allocates the memory for the array and calls
 * rcontrol_msgs__action__MoveCartesian_FeedbackMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
rcontrol_msgs__action__MoveCartesian_FeedbackMessage__Sequence *
rcontrol_msgs__action__MoveCartesian_FeedbackMessage__Sequence__create(size_t size);

/// Destroy array of action/MoveCartesian messages.
/**
 * It calls
 * rcontrol_msgs__action__MoveCartesian_FeedbackMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rcontrol_msgs
void
rcontrol_msgs__action__MoveCartesian_FeedbackMessage__Sequence__destroy(rcontrol_msgs__action__MoveCartesian_FeedbackMessage__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // RCONTROL_MSGS__ACTION__DETAIL__MOVE_CARTESIAN__FUNCTIONS_H_
