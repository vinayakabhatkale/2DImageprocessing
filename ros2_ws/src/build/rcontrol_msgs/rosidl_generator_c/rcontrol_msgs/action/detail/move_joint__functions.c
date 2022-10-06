// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rcontrol_msgs:action/MoveJoint.idl
// generated code does not contain a copyright notice
#include "rcontrol_msgs/action/detail/move_joint__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `jointvalues`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
rcontrol_msgs__action__MoveJoint_Goal__init(rcontrol_msgs__action__MoveJoint_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // jointvalues
  if (!rosidl_runtime_c__double__Sequence__init(&msg->jointvalues, 0)) {
    rcontrol_msgs__action__MoveJoint_Goal__fini(msg);
    return false;
  }
  return true;
}

void
rcontrol_msgs__action__MoveJoint_Goal__fini(rcontrol_msgs__action__MoveJoint_Goal * msg)
{
  if (!msg) {
    return;
  }
  // jointvalues
  rosidl_runtime_c__double__Sequence__fini(&msg->jointvalues);
}

rcontrol_msgs__action__MoveJoint_Goal *
rcontrol_msgs__action__MoveJoint_Goal__create()
{
  rcontrol_msgs__action__MoveJoint_Goal * msg = (rcontrol_msgs__action__MoveJoint_Goal *)malloc(sizeof(rcontrol_msgs__action__MoveJoint_Goal));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rcontrol_msgs__action__MoveJoint_Goal));
  bool success = rcontrol_msgs__action__MoveJoint_Goal__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rcontrol_msgs__action__MoveJoint_Goal__destroy(rcontrol_msgs__action__MoveJoint_Goal * msg)
{
  if (msg) {
    rcontrol_msgs__action__MoveJoint_Goal__fini(msg);
  }
  free(msg);
}


bool
rcontrol_msgs__action__MoveJoint_Goal__Sequence__init(rcontrol_msgs__action__MoveJoint_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcontrol_msgs__action__MoveJoint_Goal * data = NULL;
  if (size) {
    data = (rcontrol_msgs__action__MoveJoint_Goal *)calloc(size, sizeof(rcontrol_msgs__action__MoveJoint_Goal));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rcontrol_msgs__action__MoveJoint_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rcontrol_msgs__action__MoveJoint_Goal__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rcontrol_msgs__action__MoveJoint_Goal__Sequence__fini(rcontrol_msgs__action__MoveJoint_Goal__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rcontrol_msgs__action__MoveJoint_Goal__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rcontrol_msgs__action__MoveJoint_Goal__Sequence *
rcontrol_msgs__action__MoveJoint_Goal__Sequence__create(size_t size)
{
  rcontrol_msgs__action__MoveJoint_Goal__Sequence * array = (rcontrol_msgs__action__MoveJoint_Goal__Sequence *)malloc(sizeof(rcontrol_msgs__action__MoveJoint_Goal__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rcontrol_msgs__action__MoveJoint_Goal__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rcontrol_msgs__action__MoveJoint_Goal__Sequence__destroy(rcontrol_msgs__action__MoveJoint_Goal__Sequence * array)
{
  if (array) {
    rcontrol_msgs__action__MoveJoint_Goal__Sequence__fini(array);
  }
  free(array);
}


bool
rcontrol_msgs__action__MoveJoint_Result__init(rcontrol_msgs__action__MoveJoint_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
rcontrol_msgs__action__MoveJoint_Result__fini(rcontrol_msgs__action__MoveJoint_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
}

rcontrol_msgs__action__MoveJoint_Result *
rcontrol_msgs__action__MoveJoint_Result__create()
{
  rcontrol_msgs__action__MoveJoint_Result * msg = (rcontrol_msgs__action__MoveJoint_Result *)malloc(sizeof(rcontrol_msgs__action__MoveJoint_Result));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rcontrol_msgs__action__MoveJoint_Result));
  bool success = rcontrol_msgs__action__MoveJoint_Result__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rcontrol_msgs__action__MoveJoint_Result__destroy(rcontrol_msgs__action__MoveJoint_Result * msg)
{
  if (msg) {
    rcontrol_msgs__action__MoveJoint_Result__fini(msg);
  }
  free(msg);
}


bool
rcontrol_msgs__action__MoveJoint_Result__Sequence__init(rcontrol_msgs__action__MoveJoint_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcontrol_msgs__action__MoveJoint_Result * data = NULL;
  if (size) {
    data = (rcontrol_msgs__action__MoveJoint_Result *)calloc(size, sizeof(rcontrol_msgs__action__MoveJoint_Result));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rcontrol_msgs__action__MoveJoint_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rcontrol_msgs__action__MoveJoint_Result__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rcontrol_msgs__action__MoveJoint_Result__Sequence__fini(rcontrol_msgs__action__MoveJoint_Result__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rcontrol_msgs__action__MoveJoint_Result__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rcontrol_msgs__action__MoveJoint_Result__Sequence *
rcontrol_msgs__action__MoveJoint_Result__Sequence__create(size_t size)
{
  rcontrol_msgs__action__MoveJoint_Result__Sequence * array = (rcontrol_msgs__action__MoveJoint_Result__Sequence *)malloc(sizeof(rcontrol_msgs__action__MoveJoint_Result__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rcontrol_msgs__action__MoveJoint_Result__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rcontrol_msgs__action__MoveJoint_Result__Sequence__destroy(rcontrol_msgs__action__MoveJoint_Result__Sequence * array)
{
  if (array) {
    rcontrol_msgs__action__MoveJoint_Result__Sequence__fini(array);
  }
  free(array);
}


bool
rcontrol_msgs__action__MoveJoint_Feedback__init(rcontrol_msgs__action__MoveJoint_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // status
  return true;
}

void
rcontrol_msgs__action__MoveJoint_Feedback__fini(rcontrol_msgs__action__MoveJoint_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // status
}

rcontrol_msgs__action__MoveJoint_Feedback *
rcontrol_msgs__action__MoveJoint_Feedback__create()
{
  rcontrol_msgs__action__MoveJoint_Feedback * msg = (rcontrol_msgs__action__MoveJoint_Feedback *)malloc(sizeof(rcontrol_msgs__action__MoveJoint_Feedback));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rcontrol_msgs__action__MoveJoint_Feedback));
  bool success = rcontrol_msgs__action__MoveJoint_Feedback__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rcontrol_msgs__action__MoveJoint_Feedback__destroy(rcontrol_msgs__action__MoveJoint_Feedback * msg)
{
  if (msg) {
    rcontrol_msgs__action__MoveJoint_Feedback__fini(msg);
  }
  free(msg);
}


bool
rcontrol_msgs__action__MoveJoint_Feedback__Sequence__init(rcontrol_msgs__action__MoveJoint_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcontrol_msgs__action__MoveJoint_Feedback * data = NULL;
  if (size) {
    data = (rcontrol_msgs__action__MoveJoint_Feedback *)calloc(size, sizeof(rcontrol_msgs__action__MoveJoint_Feedback));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rcontrol_msgs__action__MoveJoint_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rcontrol_msgs__action__MoveJoint_Feedback__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rcontrol_msgs__action__MoveJoint_Feedback__Sequence__fini(rcontrol_msgs__action__MoveJoint_Feedback__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rcontrol_msgs__action__MoveJoint_Feedback__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rcontrol_msgs__action__MoveJoint_Feedback__Sequence *
rcontrol_msgs__action__MoveJoint_Feedback__Sequence__create(size_t size)
{
  rcontrol_msgs__action__MoveJoint_Feedback__Sequence * array = (rcontrol_msgs__action__MoveJoint_Feedback__Sequence *)malloc(sizeof(rcontrol_msgs__action__MoveJoint_Feedback__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rcontrol_msgs__action__MoveJoint_Feedback__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rcontrol_msgs__action__MoveJoint_Feedback__Sequence__destroy(rcontrol_msgs__action__MoveJoint_Feedback__Sequence * array)
{
  if (array) {
    rcontrol_msgs__action__MoveJoint_Feedback__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__functions.h"

bool
rcontrol_msgs__action__MoveJoint_SendGoal_Request__init(rcontrol_msgs__action__MoveJoint_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    rcontrol_msgs__action__MoveJoint_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!rcontrol_msgs__action__MoveJoint_Goal__init(&msg->goal)) {
    rcontrol_msgs__action__MoveJoint_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
rcontrol_msgs__action__MoveJoint_SendGoal_Request__fini(rcontrol_msgs__action__MoveJoint_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  rcontrol_msgs__action__MoveJoint_Goal__fini(&msg->goal);
}

rcontrol_msgs__action__MoveJoint_SendGoal_Request *
rcontrol_msgs__action__MoveJoint_SendGoal_Request__create()
{
  rcontrol_msgs__action__MoveJoint_SendGoal_Request * msg = (rcontrol_msgs__action__MoveJoint_SendGoal_Request *)malloc(sizeof(rcontrol_msgs__action__MoveJoint_SendGoal_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rcontrol_msgs__action__MoveJoint_SendGoal_Request));
  bool success = rcontrol_msgs__action__MoveJoint_SendGoal_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rcontrol_msgs__action__MoveJoint_SendGoal_Request__destroy(rcontrol_msgs__action__MoveJoint_SendGoal_Request * msg)
{
  if (msg) {
    rcontrol_msgs__action__MoveJoint_SendGoal_Request__fini(msg);
  }
  free(msg);
}


bool
rcontrol_msgs__action__MoveJoint_SendGoal_Request__Sequence__init(rcontrol_msgs__action__MoveJoint_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcontrol_msgs__action__MoveJoint_SendGoal_Request * data = NULL;
  if (size) {
    data = (rcontrol_msgs__action__MoveJoint_SendGoal_Request *)calloc(size, sizeof(rcontrol_msgs__action__MoveJoint_SendGoal_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rcontrol_msgs__action__MoveJoint_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rcontrol_msgs__action__MoveJoint_SendGoal_Request__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rcontrol_msgs__action__MoveJoint_SendGoal_Request__Sequence__fini(rcontrol_msgs__action__MoveJoint_SendGoal_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rcontrol_msgs__action__MoveJoint_SendGoal_Request__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rcontrol_msgs__action__MoveJoint_SendGoal_Request__Sequence *
rcontrol_msgs__action__MoveJoint_SendGoal_Request__Sequence__create(size_t size)
{
  rcontrol_msgs__action__MoveJoint_SendGoal_Request__Sequence * array = (rcontrol_msgs__action__MoveJoint_SendGoal_Request__Sequence *)malloc(sizeof(rcontrol_msgs__action__MoveJoint_SendGoal_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rcontrol_msgs__action__MoveJoint_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rcontrol_msgs__action__MoveJoint_SendGoal_Request__Sequence__destroy(rcontrol_msgs__action__MoveJoint_SendGoal_Request__Sequence * array)
{
  if (array) {
    rcontrol_msgs__action__MoveJoint_SendGoal_Request__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
rcontrol_msgs__action__MoveJoint_SendGoal_Response__init(rcontrol_msgs__action__MoveJoint_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    rcontrol_msgs__action__MoveJoint_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
rcontrol_msgs__action__MoveJoint_SendGoal_Response__fini(rcontrol_msgs__action__MoveJoint_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

rcontrol_msgs__action__MoveJoint_SendGoal_Response *
rcontrol_msgs__action__MoveJoint_SendGoal_Response__create()
{
  rcontrol_msgs__action__MoveJoint_SendGoal_Response * msg = (rcontrol_msgs__action__MoveJoint_SendGoal_Response *)malloc(sizeof(rcontrol_msgs__action__MoveJoint_SendGoal_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rcontrol_msgs__action__MoveJoint_SendGoal_Response));
  bool success = rcontrol_msgs__action__MoveJoint_SendGoal_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rcontrol_msgs__action__MoveJoint_SendGoal_Response__destroy(rcontrol_msgs__action__MoveJoint_SendGoal_Response * msg)
{
  if (msg) {
    rcontrol_msgs__action__MoveJoint_SendGoal_Response__fini(msg);
  }
  free(msg);
}


bool
rcontrol_msgs__action__MoveJoint_SendGoal_Response__Sequence__init(rcontrol_msgs__action__MoveJoint_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcontrol_msgs__action__MoveJoint_SendGoal_Response * data = NULL;
  if (size) {
    data = (rcontrol_msgs__action__MoveJoint_SendGoal_Response *)calloc(size, sizeof(rcontrol_msgs__action__MoveJoint_SendGoal_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rcontrol_msgs__action__MoveJoint_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rcontrol_msgs__action__MoveJoint_SendGoal_Response__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rcontrol_msgs__action__MoveJoint_SendGoal_Response__Sequence__fini(rcontrol_msgs__action__MoveJoint_SendGoal_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rcontrol_msgs__action__MoveJoint_SendGoal_Response__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rcontrol_msgs__action__MoveJoint_SendGoal_Response__Sequence *
rcontrol_msgs__action__MoveJoint_SendGoal_Response__Sequence__create(size_t size)
{
  rcontrol_msgs__action__MoveJoint_SendGoal_Response__Sequence * array = (rcontrol_msgs__action__MoveJoint_SendGoal_Response__Sequence *)malloc(sizeof(rcontrol_msgs__action__MoveJoint_SendGoal_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rcontrol_msgs__action__MoveJoint_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rcontrol_msgs__action__MoveJoint_SendGoal_Response__Sequence__destroy(rcontrol_msgs__action__MoveJoint_SendGoal_Response__Sequence * array)
{
  if (array) {
    rcontrol_msgs__action__MoveJoint_SendGoal_Response__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
rcontrol_msgs__action__MoveJoint_GetResult_Request__init(rcontrol_msgs__action__MoveJoint_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    rcontrol_msgs__action__MoveJoint_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
rcontrol_msgs__action__MoveJoint_GetResult_Request__fini(rcontrol_msgs__action__MoveJoint_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

rcontrol_msgs__action__MoveJoint_GetResult_Request *
rcontrol_msgs__action__MoveJoint_GetResult_Request__create()
{
  rcontrol_msgs__action__MoveJoint_GetResult_Request * msg = (rcontrol_msgs__action__MoveJoint_GetResult_Request *)malloc(sizeof(rcontrol_msgs__action__MoveJoint_GetResult_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rcontrol_msgs__action__MoveJoint_GetResult_Request));
  bool success = rcontrol_msgs__action__MoveJoint_GetResult_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rcontrol_msgs__action__MoveJoint_GetResult_Request__destroy(rcontrol_msgs__action__MoveJoint_GetResult_Request * msg)
{
  if (msg) {
    rcontrol_msgs__action__MoveJoint_GetResult_Request__fini(msg);
  }
  free(msg);
}


bool
rcontrol_msgs__action__MoveJoint_GetResult_Request__Sequence__init(rcontrol_msgs__action__MoveJoint_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcontrol_msgs__action__MoveJoint_GetResult_Request * data = NULL;
  if (size) {
    data = (rcontrol_msgs__action__MoveJoint_GetResult_Request *)calloc(size, sizeof(rcontrol_msgs__action__MoveJoint_GetResult_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rcontrol_msgs__action__MoveJoint_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rcontrol_msgs__action__MoveJoint_GetResult_Request__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rcontrol_msgs__action__MoveJoint_GetResult_Request__Sequence__fini(rcontrol_msgs__action__MoveJoint_GetResult_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rcontrol_msgs__action__MoveJoint_GetResult_Request__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rcontrol_msgs__action__MoveJoint_GetResult_Request__Sequence *
rcontrol_msgs__action__MoveJoint_GetResult_Request__Sequence__create(size_t size)
{
  rcontrol_msgs__action__MoveJoint_GetResult_Request__Sequence * array = (rcontrol_msgs__action__MoveJoint_GetResult_Request__Sequence *)malloc(sizeof(rcontrol_msgs__action__MoveJoint_GetResult_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rcontrol_msgs__action__MoveJoint_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rcontrol_msgs__action__MoveJoint_GetResult_Request__Sequence__destroy(rcontrol_msgs__action__MoveJoint_GetResult_Request__Sequence * array)
{
  if (array) {
    rcontrol_msgs__action__MoveJoint_GetResult_Request__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `result`
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__functions.h"

bool
rcontrol_msgs__action__MoveJoint_GetResult_Response__init(rcontrol_msgs__action__MoveJoint_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!rcontrol_msgs__action__MoveJoint_Result__init(&msg->result)) {
    rcontrol_msgs__action__MoveJoint_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
rcontrol_msgs__action__MoveJoint_GetResult_Response__fini(rcontrol_msgs__action__MoveJoint_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  rcontrol_msgs__action__MoveJoint_Result__fini(&msg->result);
}

rcontrol_msgs__action__MoveJoint_GetResult_Response *
rcontrol_msgs__action__MoveJoint_GetResult_Response__create()
{
  rcontrol_msgs__action__MoveJoint_GetResult_Response * msg = (rcontrol_msgs__action__MoveJoint_GetResult_Response *)malloc(sizeof(rcontrol_msgs__action__MoveJoint_GetResult_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rcontrol_msgs__action__MoveJoint_GetResult_Response));
  bool success = rcontrol_msgs__action__MoveJoint_GetResult_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rcontrol_msgs__action__MoveJoint_GetResult_Response__destroy(rcontrol_msgs__action__MoveJoint_GetResult_Response * msg)
{
  if (msg) {
    rcontrol_msgs__action__MoveJoint_GetResult_Response__fini(msg);
  }
  free(msg);
}


bool
rcontrol_msgs__action__MoveJoint_GetResult_Response__Sequence__init(rcontrol_msgs__action__MoveJoint_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcontrol_msgs__action__MoveJoint_GetResult_Response * data = NULL;
  if (size) {
    data = (rcontrol_msgs__action__MoveJoint_GetResult_Response *)calloc(size, sizeof(rcontrol_msgs__action__MoveJoint_GetResult_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rcontrol_msgs__action__MoveJoint_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rcontrol_msgs__action__MoveJoint_GetResult_Response__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rcontrol_msgs__action__MoveJoint_GetResult_Response__Sequence__fini(rcontrol_msgs__action__MoveJoint_GetResult_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rcontrol_msgs__action__MoveJoint_GetResult_Response__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rcontrol_msgs__action__MoveJoint_GetResult_Response__Sequence *
rcontrol_msgs__action__MoveJoint_GetResult_Response__Sequence__create(size_t size)
{
  rcontrol_msgs__action__MoveJoint_GetResult_Response__Sequence * array = (rcontrol_msgs__action__MoveJoint_GetResult_Response__Sequence *)malloc(sizeof(rcontrol_msgs__action__MoveJoint_GetResult_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rcontrol_msgs__action__MoveJoint_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rcontrol_msgs__action__MoveJoint_GetResult_Response__Sequence__destroy(rcontrol_msgs__action__MoveJoint_GetResult_Response__Sequence * array)
{
  if (array) {
    rcontrol_msgs__action__MoveJoint_GetResult_Response__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "rcontrol_msgs/action/detail/move_joint__functions.h"

bool
rcontrol_msgs__action__MoveJoint_FeedbackMessage__init(rcontrol_msgs__action__MoveJoint_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    rcontrol_msgs__action__MoveJoint_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!rcontrol_msgs__action__MoveJoint_Feedback__init(&msg->feedback)) {
    rcontrol_msgs__action__MoveJoint_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
rcontrol_msgs__action__MoveJoint_FeedbackMessage__fini(rcontrol_msgs__action__MoveJoint_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  rcontrol_msgs__action__MoveJoint_Feedback__fini(&msg->feedback);
}

rcontrol_msgs__action__MoveJoint_FeedbackMessage *
rcontrol_msgs__action__MoveJoint_FeedbackMessage__create()
{
  rcontrol_msgs__action__MoveJoint_FeedbackMessage * msg = (rcontrol_msgs__action__MoveJoint_FeedbackMessage *)malloc(sizeof(rcontrol_msgs__action__MoveJoint_FeedbackMessage));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rcontrol_msgs__action__MoveJoint_FeedbackMessage));
  bool success = rcontrol_msgs__action__MoveJoint_FeedbackMessage__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rcontrol_msgs__action__MoveJoint_FeedbackMessage__destroy(rcontrol_msgs__action__MoveJoint_FeedbackMessage * msg)
{
  if (msg) {
    rcontrol_msgs__action__MoveJoint_FeedbackMessage__fini(msg);
  }
  free(msg);
}


bool
rcontrol_msgs__action__MoveJoint_FeedbackMessage__Sequence__init(rcontrol_msgs__action__MoveJoint_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcontrol_msgs__action__MoveJoint_FeedbackMessage * data = NULL;
  if (size) {
    data = (rcontrol_msgs__action__MoveJoint_FeedbackMessage *)calloc(size, sizeof(rcontrol_msgs__action__MoveJoint_FeedbackMessage));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rcontrol_msgs__action__MoveJoint_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rcontrol_msgs__action__MoveJoint_FeedbackMessage__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rcontrol_msgs__action__MoveJoint_FeedbackMessage__Sequence__fini(rcontrol_msgs__action__MoveJoint_FeedbackMessage__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rcontrol_msgs__action__MoveJoint_FeedbackMessage__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rcontrol_msgs__action__MoveJoint_FeedbackMessage__Sequence *
rcontrol_msgs__action__MoveJoint_FeedbackMessage__Sequence__create(size_t size)
{
  rcontrol_msgs__action__MoveJoint_FeedbackMessage__Sequence * array = (rcontrol_msgs__action__MoveJoint_FeedbackMessage__Sequence *)malloc(sizeof(rcontrol_msgs__action__MoveJoint_FeedbackMessage__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rcontrol_msgs__action__MoveJoint_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rcontrol_msgs__action__MoveJoint_FeedbackMessage__Sequence__destroy(rcontrol_msgs__action__MoveJoint_FeedbackMessage__Sequence * array)
{
  if (array) {
    rcontrol_msgs__action__MoveJoint_FeedbackMessage__Sequence__fini(array);
  }
  free(array);
}
