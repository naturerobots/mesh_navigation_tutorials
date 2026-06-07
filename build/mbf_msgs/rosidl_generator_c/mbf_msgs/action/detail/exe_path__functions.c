// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mbf_msgs:action/ExePath.idl
// generated code does not contain a copyright notice
#include "mbf_msgs/action/detail/exe_path__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `path`
#include "nav_msgs/msg/detail/path__functions.h"
// Member `controller`
#include "rosidl_runtime_c/string_functions.h"

bool
mbf_msgs__action__ExePath_Goal__init(mbf_msgs__action__ExePath_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // path
  if (!nav_msgs__msg__Path__init(&msg->path)) {
    mbf_msgs__action__ExePath_Goal__fini(msg);
    return false;
  }
  // controller
  if (!rosidl_runtime_c__String__init(&msg->controller)) {
    mbf_msgs__action__ExePath_Goal__fini(msg);
    return false;
  }
  // concurrency_slot
  // tolerance_from_action
  // dist_tolerance
  // angle_tolerance
  return true;
}

void
mbf_msgs__action__ExePath_Goal__fini(mbf_msgs__action__ExePath_Goal * msg)
{
  if (!msg) {
    return;
  }
  // path
  nav_msgs__msg__Path__fini(&msg->path);
  // controller
  rosidl_runtime_c__String__fini(&msg->controller);
  // concurrency_slot
  // tolerance_from_action
  // dist_tolerance
  // angle_tolerance
}

bool
mbf_msgs__action__ExePath_Goal__are_equal(const mbf_msgs__action__ExePath_Goal * lhs, const mbf_msgs__action__ExePath_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // path
  if (!nav_msgs__msg__Path__are_equal(
      &(lhs->path), &(rhs->path)))
  {
    return false;
  }
  // controller
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->controller), &(rhs->controller)))
  {
    return false;
  }
  // concurrency_slot
  if (lhs->concurrency_slot != rhs->concurrency_slot) {
    return false;
  }
  // tolerance_from_action
  if (lhs->tolerance_from_action != rhs->tolerance_from_action) {
    return false;
  }
  // dist_tolerance
  if (lhs->dist_tolerance != rhs->dist_tolerance) {
    return false;
  }
  // angle_tolerance
  if (lhs->angle_tolerance != rhs->angle_tolerance) {
    return false;
  }
  return true;
}

bool
mbf_msgs__action__ExePath_Goal__copy(
  const mbf_msgs__action__ExePath_Goal * input,
  mbf_msgs__action__ExePath_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // path
  if (!nav_msgs__msg__Path__copy(
      &(input->path), &(output->path)))
  {
    return false;
  }
  // controller
  if (!rosidl_runtime_c__String__copy(
      &(input->controller), &(output->controller)))
  {
    return false;
  }
  // concurrency_slot
  output->concurrency_slot = input->concurrency_slot;
  // tolerance_from_action
  output->tolerance_from_action = input->tolerance_from_action;
  // dist_tolerance
  output->dist_tolerance = input->dist_tolerance;
  // angle_tolerance
  output->angle_tolerance = input->angle_tolerance;
  return true;
}

mbf_msgs__action__ExePath_Goal *
mbf_msgs__action__ExePath_Goal__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_Goal * msg = (mbf_msgs__action__ExePath_Goal *)allocator.allocate(sizeof(mbf_msgs__action__ExePath_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbf_msgs__action__ExePath_Goal));
  bool success = mbf_msgs__action__ExePath_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbf_msgs__action__ExePath_Goal__destroy(mbf_msgs__action__ExePath_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbf_msgs__action__ExePath_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbf_msgs__action__ExePath_Goal__Sequence__init(mbf_msgs__action__ExePath_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_Goal * data = NULL;

  if (size) {
    data = (mbf_msgs__action__ExePath_Goal *)allocator.zero_allocate(size, sizeof(mbf_msgs__action__ExePath_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbf_msgs__action__ExePath_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbf_msgs__action__ExePath_Goal__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mbf_msgs__action__ExePath_Goal__Sequence__fini(mbf_msgs__action__ExePath_Goal__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mbf_msgs__action__ExePath_Goal__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mbf_msgs__action__ExePath_Goal__Sequence *
mbf_msgs__action__ExePath_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_Goal__Sequence * array = (mbf_msgs__action__ExePath_Goal__Sequence *)allocator.allocate(sizeof(mbf_msgs__action__ExePath_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbf_msgs__action__ExePath_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbf_msgs__action__ExePath_Goal__Sequence__destroy(mbf_msgs__action__ExePath_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbf_msgs__action__ExePath_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbf_msgs__action__ExePath_Goal__Sequence__are_equal(const mbf_msgs__action__ExePath_Goal__Sequence * lhs, const mbf_msgs__action__ExePath_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbf_msgs__action__ExePath_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbf_msgs__action__ExePath_Goal__Sequence__copy(
  const mbf_msgs__action__ExePath_Goal__Sequence * input,
  mbf_msgs__action__ExePath_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbf_msgs__action__ExePath_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mbf_msgs__action__ExePath_Goal * data =
      (mbf_msgs__action__ExePath_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbf_msgs__action__ExePath_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mbf_msgs__action__ExePath_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbf_msgs__action__ExePath_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `final_pose`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
mbf_msgs__action__ExePath_Result__init(mbf_msgs__action__ExePath_Result * msg)
{
  if (!msg) {
    return false;
  }
  // outcome
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    mbf_msgs__action__ExePath_Result__fini(msg);
    return false;
  }
  // final_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->final_pose)) {
    mbf_msgs__action__ExePath_Result__fini(msg);
    return false;
  }
  // dist_to_goal
  // angle_to_goal
  return true;
}

void
mbf_msgs__action__ExePath_Result__fini(mbf_msgs__action__ExePath_Result * msg)
{
  if (!msg) {
    return;
  }
  // outcome
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // final_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->final_pose);
  // dist_to_goal
  // angle_to_goal
}

bool
mbf_msgs__action__ExePath_Result__are_equal(const mbf_msgs__action__ExePath_Result * lhs, const mbf_msgs__action__ExePath_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // outcome
  if (lhs->outcome != rhs->outcome) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  // final_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->final_pose), &(rhs->final_pose)))
  {
    return false;
  }
  // dist_to_goal
  if (lhs->dist_to_goal != rhs->dist_to_goal) {
    return false;
  }
  // angle_to_goal
  if (lhs->angle_to_goal != rhs->angle_to_goal) {
    return false;
  }
  return true;
}

bool
mbf_msgs__action__ExePath_Result__copy(
  const mbf_msgs__action__ExePath_Result * input,
  mbf_msgs__action__ExePath_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // outcome
  output->outcome = input->outcome;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  // final_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->final_pose), &(output->final_pose)))
  {
    return false;
  }
  // dist_to_goal
  output->dist_to_goal = input->dist_to_goal;
  // angle_to_goal
  output->angle_to_goal = input->angle_to_goal;
  return true;
}

mbf_msgs__action__ExePath_Result *
mbf_msgs__action__ExePath_Result__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_Result * msg = (mbf_msgs__action__ExePath_Result *)allocator.allocate(sizeof(mbf_msgs__action__ExePath_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbf_msgs__action__ExePath_Result));
  bool success = mbf_msgs__action__ExePath_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbf_msgs__action__ExePath_Result__destroy(mbf_msgs__action__ExePath_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbf_msgs__action__ExePath_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbf_msgs__action__ExePath_Result__Sequence__init(mbf_msgs__action__ExePath_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_Result * data = NULL;

  if (size) {
    data = (mbf_msgs__action__ExePath_Result *)allocator.zero_allocate(size, sizeof(mbf_msgs__action__ExePath_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbf_msgs__action__ExePath_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbf_msgs__action__ExePath_Result__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mbf_msgs__action__ExePath_Result__Sequence__fini(mbf_msgs__action__ExePath_Result__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mbf_msgs__action__ExePath_Result__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mbf_msgs__action__ExePath_Result__Sequence *
mbf_msgs__action__ExePath_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_Result__Sequence * array = (mbf_msgs__action__ExePath_Result__Sequence *)allocator.allocate(sizeof(mbf_msgs__action__ExePath_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbf_msgs__action__ExePath_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbf_msgs__action__ExePath_Result__Sequence__destroy(mbf_msgs__action__ExePath_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbf_msgs__action__ExePath_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbf_msgs__action__ExePath_Result__Sequence__are_equal(const mbf_msgs__action__ExePath_Result__Sequence * lhs, const mbf_msgs__action__ExePath_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbf_msgs__action__ExePath_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbf_msgs__action__ExePath_Result__Sequence__copy(
  const mbf_msgs__action__ExePath_Result__Sequence * input,
  mbf_msgs__action__ExePath_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbf_msgs__action__ExePath_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mbf_msgs__action__ExePath_Result * data =
      (mbf_msgs__action__ExePath_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbf_msgs__action__ExePath_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mbf_msgs__action__ExePath_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbf_msgs__action__ExePath_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `current_pose`
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__functions.h"
// Member `last_cmd_vel`
#include "geometry_msgs/msg/detail/twist_stamped__functions.h"

bool
mbf_msgs__action__ExePath_Feedback__init(mbf_msgs__action__ExePath_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // outcome
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    mbf_msgs__action__ExePath_Feedback__fini(msg);
    return false;
  }
  // dist_to_goal
  // angle_to_goal
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->current_pose)) {
    mbf_msgs__action__ExePath_Feedback__fini(msg);
    return false;
  }
  // last_cmd_vel
  if (!geometry_msgs__msg__TwistStamped__init(&msg->last_cmd_vel)) {
    mbf_msgs__action__ExePath_Feedback__fini(msg);
    return false;
  }
  return true;
}

void
mbf_msgs__action__ExePath_Feedback__fini(mbf_msgs__action__ExePath_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // outcome
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // dist_to_goal
  // angle_to_goal
  // current_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->current_pose);
  // last_cmd_vel
  geometry_msgs__msg__TwistStamped__fini(&msg->last_cmd_vel);
}

bool
mbf_msgs__action__ExePath_Feedback__are_equal(const mbf_msgs__action__ExePath_Feedback * lhs, const mbf_msgs__action__ExePath_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // outcome
  if (lhs->outcome != rhs->outcome) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  // dist_to_goal
  if (lhs->dist_to_goal != rhs->dist_to_goal) {
    return false;
  }
  // angle_to_goal
  if (lhs->angle_to_goal != rhs->angle_to_goal) {
    return false;
  }
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->current_pose), &(rhs->current_pose)))
  {
    return false;
  }
  // last_cmd_vel
  if (!geometry_msgs__msg__TwistStamped__are_equal(
      &(lhs->last_cmd_vel), &(rhs->last_cmd_vel)))
  {
    return false;
  }
  return true;
}

bool
mbf_msgs__action__ExePath_Feedback__copy(
  const mbf_msgs__action__ExePath_Feedback * input,
  mbf_msgs__action__ExePath_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // outcome
  output->outcome = input->outcome;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  // dist_to_goal
  output->dist_to_goal = input->dist_to_goal;
  // angle_to_goal
  output->angle_to_goal = input->angle_to_goal;
  // current_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->current_pose), &(output->current_pose)))
  {
    return false;
  }
  // last_cmd_vel
  if (!geometry_msgs__msg__TwistStamped__copy(
      &(input->last_cmd_vel), &(output->last_cmd_vel)))
  {
    return false;
  }
  return true;
}

mbf_msgs__action__ExePath_Feedback *
mbf_msgs__action__ExePath_Feedback__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_Feedback * msg = (mbf_msgs__action__ExePath_Feedback *)allocator.allocate(sizeof(mbf_msgs__action__ExePath_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbf_msgs__action__ExePath_Feedback));
  bool success = mbf_msgs__action__ExePath_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbf_msgs__action__ExePath_Feedback__destroy(mbf_msgs__action__ExePath_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbf_msgs__action__ExePath_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbf_msgs__action__ExePath_Feedback__Sequence__init(mbf_msgs__action__ExePath_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_Feedback * data = NULL;

  if (size) {
    data = (mbf_msgs__action__ExePath_Feedback *)allocator.zero_allocate(size, sizeof(mbf_msgs__action__ExePath_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbf_msgs__action__ExePath_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbf_msgs__action__ExePath_Feedback__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mbf_msgs__action__ExePath_Feedback__Sequence__fini(mbf_msgs__action__ExePath_Feedback__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mbf_msgs__action__ExePath_Feedback__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mbf_msgs__action__ExePath_Feedback__Sequence *
mbf_msgs__action__ExePath_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_Feedback__Sequence * array = (mbf_msgs__action__ExePath_Feedback__Sequence *)allocator.allocate(sizeof(mbf_msgs__action__ExePath_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbf_msgs__action__ExePath_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbf_msgs__action__ExePath_Feedback__Sequence__destroy(mbf_msgs__action__ExePath_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbf_msgs__action__ExePath_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbf_msgs__action__ExePath_Feedback__Sequence__are_equal(const mbf_msgs__action__ExePath_Feedback__Sequence * lhs, const mbf_msgs__action__ExePath_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbf_msgs__action__ExePath_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbf_msgs__action__ExePath_Feedback__Sequence__copy(
  const mbf_msgs__action__ExePath_Feedback__Sequence * input,
  mbf_msgs__action__ExePath_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbf_msgs__action__ExePath_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mbf_msgs__action__ExePath_Feedback * data =
      (mbf_msgs__action__ExePath_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbf_msgs__action__ExePath_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mbf_msgs__action__ExePath_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbf_msgs__action__ExePath_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "mbf_msgs/action/detail/exe_path__functions.h"

bool
mbf_msgs__action__ExePath_SendGoal_Request__init(mbf_msgs__action__ExePath_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    mbf_msgs__action__ExePath_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!mbf_msgs__action__ExePath_Goal__init(&msg->goal)) {
    mbf_msgs__action__ExePath_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
mbf_msgs__action__ExePath_SendGoal_Request__fini(mbf_msgs__action__ExePath_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  mbf_msgs__action__ExePath_Goal__fini(&msg->goal);
}

bool
mbf_msgs__action__ExePath_SendGoal_Request__are_equal(const mbf_msgs__action__ExePath_SendGoal_Request * lhs, const mbf_msgs__action__ExePath_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!mbf_msgs__action__ExePath_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
mbf_msgs__action__ExePath_SendGoal_Request__copy(
  const mbf_msgs__action__ExePath_SendGoal_Request * input,
  mbf_msgs__action__ExePath_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!mbf_msgs__action__ExePath_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

mbf_msgs__action__ExePath_SendGoal_Request *
mbf_msgs__action__ExePath_SendGoal_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_SendGoal_Request * msg = (mbf_msgs__action__ExePath_SendGoal_Request *)allocator.allocate(sizeof(mbf_msgs__action__ExePath_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbf_msgs__action__ExePath_SendGoal_Request));
  bool success = mbf_msgs__action__ExePath_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbf_msgs__action__ExePath_SendGoal_Request__destroy(mbf_msgs__action__ExePath_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbf_msgs__action__ExePath_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbf_msgs__action__ExePath_SendGoal_Request__Sequence__init(mbf_msgs__action__ExePath_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_SendGoal_Request * data = NULL;

  if (size) {
    data = (mbf_msgs__action__ExePath_SendGoal_Request *)allocator.zero_allocate(size, sizeof(mbf_msgs__action__ExePath_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbf_msgs__action__ExePath_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbf_msgs__action__ExePath_SendGoal_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mbf_msgs__action__ExePath_SendGoal_Request__Sequence__fini(mbf_msgs__action__ExePath_SendGoal_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mbf_msgs__action__ExePath_SendGoal_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mbf_msgs__action__ExePath_SendGoal_Request__Sequence *
mbf_msgs__action__ExePath_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_SendGoal_Request__Sequence * array = (mbf_msgs__action__ExePath_SendGoal_Request__Sequence *)allocator.allocate(sizeof(mbf_msgs__action__ExePath_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbf_msgs__action__ExePath_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbf_msgs__action__ExePath_SendGoal_Request__Sequence__destroy(mbf_msgs__action__ExePath_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbf_msgs__action__ExePath_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbf_msgs__action__ExePath_SendGoal_Request__Sequence__are_equal(const mbf_msgs__action__ExePath_SendGoal_Request__Sequence * lhs, const mbf_msgs__action__ExePath_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbf_msgs__action__ExePath_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbf_msgs__action__ExePath_SendGoal_Request__Sequence__copy(
  const mbf_msgs__action__ExePath_SendGoal_Request__Sequence * input,
  mbf_msgs__action__ExePath_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbf_msgs__action__ExePath_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mbf_msgs__action__ExePath_SendGoal_Request * data =
      (mbf_msgs__action__ExePath_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbf_msgs__action__ExePath_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mbf_msgs__action__ExePath_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbf_msgs__action__ExePath_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
mbf_msgs__action__ExePath_SendGoal_Response__init(mbf_msgs__action__ExePath_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    mbf_msgs__action__ExePath_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
mbf_msgs__action__ExePath_SendGoal_Response__fini(mbf_msgs__action__ExePath_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
mbf_msgs__action__ExePath_SendGoal_Response__are_equal(const mbf_msgs__action__ExePath_SendGoal_Response * lhs, const mbf_msgs__action__ExePath_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
mbf_msgs__action__ExePath_SendGoal_Response__copy(
  const mbf_msgs__action__ExePath_SendGoal_Response * input,
  mbf_msgs__action__ExePath_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

mbf_msgs__action__ExePath_SendGoal_Response *
mbf_msgs__action__ExePath_SendGoal_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_SendGoal_Response * msg = (mbf_msgs__action__ExePath_SendGoal_Response *)allocator.allocate(sizeof(mbf_msgs__action__ExePath_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbf_msgs__action__ExePath_SendGoal_Response));
  bool success = mbf_msgs__action__ExePath_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbf_msgs__action__ExePath_SendGoal_Response__destroy(mbf_msgs__action__ExePath_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbf_msgs__action__ExePath_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbf_msgs__action__ExePath_SendGoal_Response__Sequence__init(mbf_msgs__action__ExePath_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_SendGoal_Response * data = NULL;

  if (size) {
    data = (mbf_msgs__action__ExePath_SendGoal_Response *)allocator.zero_allocate(size, sizeof(mbf_msgs__action__ExePath_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbf_msgs__action__ExePath_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbf_msgs__action__ExePath_SendGoal_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mbf_msgs__action__ExePath_SendGoal_Response__Sequence__fini(mbf_msgs__action__ExePath_SendGoal_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mbf_msgs__action__ExePath_SendGoal_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mbf_msgs__action__ExePath_SendGoal_Response__Sequence *
mbf_msgs__action__ExePath_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_SendGoal_Response__Sequence * array = (mbf_msgs__action__ExePath_SendGoal_Response__Sequence *)allocator.allocate(sizeof(mbf_msgs__action__ExePath_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbf_msgs__action__ExePath_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbf_msgs__action__ExePath_SendGoal_Response__Sequence__destroy(mbf_msgs__action__ExePath_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbf_msgs__action__ExePath_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbf_msgs__action__ExePath_SendGoal_Response__Sequence__are_equal(const mbf_msgs__action__ExePath_SendGoal_Response__Sequence * lhs, const mbf_msgs__action__ExePath_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbf_msgs__action__ExePath_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbf_msgs__action__ExePath_SendGoal_Response__Sequence__copy(
  const mbf_msgs__action__ExePath_SendGoal_Response__Sequence * input,
  mbf_msgs__action__ExePath_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbf_msgs__action__ExePath_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mbf_msgs__action__ExePath_SendGoal_Response * data =
      (mbf_msgs__action__ExePath_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbf_msgs__action__ExePath_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mbf_msgs__action__ExePath_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbf_msgs__action__ExePath_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "mbf_msgs/action/detail/exe_path__functions.h"

bool
mbf_msgs__action__ExePath_SendGoal_Event__init(mbf_msgs__action__ExePath_SendGoal_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    mbf_msgs__action__ExePath_SendGoal_Event__fini(msg);
    return false;
  }
  // request
  if (!mbf_msgs__action__ExePath_SendGoal_Request__Sequence__init(&msg->request, 0)) {
    mbf_msgs__action__ExePath_SendGoal_Event__fini(msg);
    return false;
  }
  // response
  if (!mbf_msgs__action__ExePath_SendGoal_Response__Sequence__init(&msg->response, 0)) {
    mbf_msgs__action__ExePath_SendGoal_Event__fini(msg);
    return false;
  }
  return true;
}

void
mbf_msgs__action__ExePath_SendGoal_Event__fini(mbf_msgs__action__ExePath_SendGoal_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  mbf_msgs__action__ExePath_SendGoal_Request__Sequence__fini(&msg->request);
  // response
  mbf_msgs__action__ExePath_SendGoal_Response__Sequence__fini(&msg->response);
}

bool
mbf_msgs__action__ExePath_SendGoal_Event__are_equal(const mbf_msgs__action__ExePath_SendGoal_Event * lhs, const mbf_msgs__action__ExePath_SendGoal_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!mbf_msgs__action__ExePath_SendGoal_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!mbf_msgs__action__ExePath_SendGoal_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
mbf_msgs__action__ExePath_SendGoal_Event__copy(
  const mbf_msgs__action__ExePath_SendGoal_Event * input,
  mbf_msgs__action__ExePath_SendGoal_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!mbf_msgs__action__ExePath_SendGoal_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!mbf_msgs__action__ExePath_SendGoal_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

mbf_msgs__action__ExePath_SendGoal_Event *
mbf_msgs__action__ExePath_SendGoal_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_SendGoal_Event * msg = (mbf_msgs__action__ExePath_SendGoal_Event *)allocator.allocate(sizeof(mbf_msgs__action__ExePath_SendGoal_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbf_msgs__action__ExePath_SendGoal_Event));
  bool success = mbf_msgs__action__ExePath_SendGoal_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbf_msgs__action__ExePath_SendGoal_Event__destroy(mbf_msgs__action__ExePath_SendGoal_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbf_msgs__action__ExePath_SendGoal_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbf_msgs__action__ExePath_SendGoal_Event__Sequence__init(mbf_msgs__action__ExePath_SendGoal_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_SendGoal_Event * data = NULL;

  if (size) {
    data = (mbf_msgs__action__ExePath_SendGoal_Event *)allocator.zero_allocate(size, sizeof(mbf_msgs__action__ExePath_SendGoal_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbf_msgs__action__ExePath_SendGoal_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbf_msgs__action__ExePath_SendGoal_Event__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mbf_msgs__action__ExePath_SendGoal_Event__Sequence__fini(mbf_msgs__action__ExePath_SendGoal_Event__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mbf_msgs__action__ExePath_SendGoal_Event__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mbf_msgs__action__ExePath_SendGoal_Event__Sequence *
mbf_msgs__action__ExePath_SendGoal_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_SendGoal_Event__Sequence * array = (mbf_msgs__action__ExePath_SendGoal_Event__Sequence *)allocator.allocate(sizeof(mbf_msgs__action__ExePath_SendGoal_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbf_msgs__action__ExePath_SendGoal_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbf_msgs__action__ExePath_SendGoal_Event__Sequence__destroy(mbf_msgs__action__ExePath_SendGoal_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbf_msgs__action__ExePath_SendGoal_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbf_msgs__action__ExePath_SendGoal_Event__Sequence__are_equal(const mbf_msgs__action__ExePath_SendGoal_Event__Sequence * lhs, const mbf_msgs__action__ExePath_SendGoal_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbf_msgs__action__ExePath_SendGoal_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbf_msgs__action__ExePath_SendGoal_Event__Sequence__copy(
  const mbf_msgs__action__ExePath_SendGoal_Event__Sequence * input,
  mbf_msgs__action__ExePath_SendGoal_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbf_msgs__action__ExePath_SendGoal_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mbf_msgs__action__ExePath_SendGoal_Event * data =
      (mbf_msgs__action__ExePath_SendGoal_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbf_msgs__action__ExePath_SendGoal_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mbf_msgs__action__ExePath_SendGoal_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbf_msgs__action__ExePath_SendGoal_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
mbf_msgs__action__ExePath_GetResult_Request__init(mbf_msgs__action__ExePath_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    mbf_msgs__action__ExePath_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
mbf_msgs__action__ExePath_GetResult_Request__fini(mbf_msgs__action__ExePath_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
mbf_msgs__action__ExePath_GetResult_Request__are_equal(const mbf_msgs__action__ExePath_GetResult_Request * lhs, const mbf_msgs__action__ExePath_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
mbf_msgs__action__ExePath_GetResult_Request__copy(
  const mbf_msgs__action__ExePath_GetResult_Request * input,
  mbf_msgs__action__ExePath_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

mbf_msgs__action__ExePath_GetResult_Request *
mbf_msgs__action__ExePath_GetResult_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_GetResult_Request * msg = (mbf_msgs__action__ExePath_GetResult_Request *)allocator.allocate(sizeof(mbf_msgs__action__ExePath_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbf_msgs__action__ExePath_GetResult_Request));
  bool success = mbf_msgs__action__ExePath_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbf_msgs__action__ExePath_GetResult_Request__destroy(mbf_msgs__action__ExePath_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbf_msgs__action__ExePath_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbf_msgs__action__ExePath_GetResult_Request__Sequence__init(mbf_msgs__action__ExePath_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_GetResult_Request * data = NULL;

  if (size) {
    data = (mbf_msgs__action__ExePath_GetResult_Request *)allocator.zero_allocate(size, sizeof(mbf_msgs__action__ExePath_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbf_msgs__action__ExePath_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbf_msgs__action__ExePath_GetResult_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mbf_msgs__action__ExePath_GetResult_Request__Sequence__fini(mbf_msgs__action__ExePath_GetResult_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mbf_msgs__action__ExePath_GetResult_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mbf_msgs__action__ExePath_GetResult_Request__Sequence *
mbf_msgs__action__ExePath_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_GetResult_Request__Sequence * array = (mbf_msgs__action__ExePath_GetResult_Request__Sequence *)allocator.allocate(sizeof(mbf_msgs__action__ExePath_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbf_msgs__action__ExePath_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbf_msgs__action__ExePath_GetResult_Request__Sequence__destroy(mbf_msgs__action__ExePath_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbf_msgs__action__ExePath_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbf_msgs__action__ExePath_GetResult_Request__Sequence__are_equal(const mbf_msgs__action__ExePath_GetResult_Request__Sequence * lhs, const mbf_msgs__action__ExePath_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbf_msgs__action__ExePath_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbf_msgs__action__ExePath_GetResult_Request__Sequence__copy(
  const mbf_msgs__action__ExePath_GetResult_Request__Sequence * input,
  mbf_msgs__action__ExePath_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbf_msgs__action__ExePath_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mbf_msgs__action__ExePath_GetResult_Request * data =
      (mbf_msgs__action__ExePath_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbf_msgs__action__ExePath_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mbf_msgs__action__ExePath_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbf_msgs__action__ExePath_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "mbf_msgs/action/detail/exe_path__functions.h"

bool
mbf_msgs__action__ExePath_GetResult_Response__init(mbf_msgs__action__ExePath_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!mbf_msgs__action__ExePath_Result__init(&msg->result)) {
    mbf_msgs__action__ExePath_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
mbf_msgs__action__ExePath_GetResult_Response__fini(mbf_msgs__action__ExePath_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  mbf_msgs__action__ExePath_Result__fini(&msg->result);
}

bool
mbf_msgs__action__ExePath_GetResult_Response__are_equal(const mbf_msgs__action__ExePath_GetResult_Response * lhs, const mbf_msgs__action__ExePath_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!mbf_msgs__action__ExePath_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
mbf_msgs__action__ExePath_GetResult_Response__copy(
  const mbf_msgs__action__ExePath_GetResult_Response * input,
  mbf_msgs__action__ExePath_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!mbf_msgs__action__ExePath_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

mbf_msgs__action__ExePath_GetResult_Response *
mbf_msgs__action__ExePath_GetResult_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_GetResult_Response * msg = (mbf_msgs__action__ExePath_GetResult_Response *)allocator.allocate(sizeof(mbf_msgs__action__ExePath_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbf_msgs__action__ExePath_GetResult_Response));
  bool success = mbf_msgs__action__ExePath_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbf_msgs__action__ExePath_GetResult_Response__destroy(mbf_msgs__action__ExePath_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbf_msgs__action__ExePath_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbf_msgs__action__ExePath_GetResult_Response__Sequence__init(mbf_msgs__action__ExePath_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_GetResult_Response * data = NULL;

  if (size) {
    data = (mbf_msgs__action__ExePath_GetResult_Response *)allocator.zero_allocate(size, sizeof(mbf_msgs__action__ExePath_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbf_msgs__action__ExePath_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbf_msgs__action__ExePath_GetResult_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mbf_msgs__action__ExePath_GetResult_Response__Sequence__fini(mbf_msgs__action__ExePath_GetResult_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mbf_msgs__action__ExePath_GetResult_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mbf_msgs__action__ExePath_GetResult_Response__Sequence *
mbf_msgs__action__ExePath_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_GetResult_Response__Sequence * array = (mbf_msgs__action__ExePath_GetResult_Response__Sequence *)allocator.allocate(sizeof(mbf_msgs__action__ExePath_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbf_msgs__action__ExePath_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbf_msgs__action__ExePath_GetResult_Response__Sequence__destroy(mbf_msgs__action__ExePath_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbf_msgs__action__ExePath_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbf_msgs__action__ExePath_GetResult_Response__Sequence__are_equal(const mbf_msgs__action__ExePath_GetResult_Response__Sequence * lhs, const mbf_msgs__action__ExePath_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbf_msgs__action__ExePath_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbf_msgs__action__ExePath_GetResult_Response__Sequence__copy(
  const mbf_msgs__action__ExePath_GetResult_Response__Sequence * input,
  mbf_msgs__action__ExePath_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbf_msgs__action__ExePath_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mbf_msgs__action__ExePath_GetResult_Response * data =
      (mbf_msgs__action__ExePath_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbf_msgs__action__ExePath_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mbf_msgs__action__ExePath_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbf_msgs__action__ExePath_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
// already included above
// #include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "mbf_msgs/action/detail/exe_path__functions.h"

bool
mbf_msgs__action__ExePath_GetResult_Event__init(mbf_msgs__action__ExePath_GetResult_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    mbf_msgs__action__ExePath_GetResult_Event__fini(msg);
    return false;
  }
  // request
  if (!mbf_msgs__action__ExePath_GetResult_Request__Sequence__init(&msg->request, 0)) {
    mbf_msgs__action__ExePath_GetResult_Event__fini(msg);
    return false;
  }
  // response
  if (!mbf_msgs__action__ExePath_GetResult_Response__Sequence__init(&msg->response, 0)) {
    mbf_msgs__action__ExePath_GetResult_Event__fini(msg);
    return false;
  }
  return true;
}

void
mbf_msgs__action__ExePath_GetResult_Event__fini(mbf_msgs__action__ExePath_GetResult_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  mbf_msgs__action__ExePath_GetResult_Request__Sequence__fini(&msg->request);
  // response
  mbf_msgs__action__ExePath_GetResult_Response__Sequence__fini(&msg->response);
}

bool
mbf_msgs__action__ExePath_GetResult_Event__are_equal(const mbf_msgs__action__ExePath_GetResult_Event * lhs, const mbf_msgs__action__ExePath_GetResult_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!mbf_msgs__action__ExePath_GetResult_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!mbf_msgs__action__ExePath_GetResult_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
mbf_msgs__action__ExePath_GetResult_Event__copy(
  const mbf_msgs__action__ExePath_GetResult_Event * input,
  mbf_msgs__action__ExePath_GetResult_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!mbf_msgs__action__ExePath_GetResult_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!mbf_msgs__action__ExePath_GetResult_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

mbf_msgs__action__ExePath_GetResult_Event *
mbf_msgs__action__ExePath_GetResult_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_GetResult_Event * msg = (mbf_msgs__action__ExePath_GetResult_Event *)allocator.allocate(sizeof(mbf_msgs__action__ExePath_GetResult_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbf_msgs__action__ExePath_GetResult_Event));
  bool success = mbf_msgs__action__ExePath_GetResult_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbf_msgs__action__ExePath_GetResult_Event__destroy(mbf_msgs__action__ExePath_GetResult_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbf_msgs__action__ExePath_GetResult_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbf_msgs__action__ExePath_GetResult_Event__Sequence__init(mbf_msgs__action__ExePath_GetResult_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_GetResult_Event * data = NULL;

  if (size) {
    data = (mbf_msgs__action__ExePath_GetResult_Event *)allocator.zero_allocate(size, sizeof(mbf_msgs__action__ExePath_GetResult_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbf_msgs__action__ExePath_GetResult_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbf_msgs__action__ExePath_GetResult_Event__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mbf_msgs__action__ExePath_GetResult_Event__Sequence__fini(mbf_msgs__action__ExePath_GetResult_Event__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mbf_msgs__action__ExePath_GetResult_Event__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mbf_msgs__action__ExePath_GetResult_Event__Sequence *
mbf_msgs__action__ExePath_GetResult_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_GetResult_Event__Sequence * array = (mbf_msgs__action__ExePath_GetResult_Event__Sequence *)allocator.allocate(sizeof(mbf_msgs__action__ExePath_GetResult_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbf_msgs__action__ExePath_GetResult_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbf_msgs__action__ExePath_GetResult_Event__Sequence__destroy(mbf_msgs__action__ExePath_GetResult_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbf_msgs__action__ExePath_GetResult_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbf_msgs__action__ExePath_GetResult_Event__Sequence__are_equal(const mbf_msgs__action__ExePath_GetResult_Event__Sequence * lhs, const mbf_msgs__action__ExePath_GetResult_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbf_msgs__action__ExePath_GetResult_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbf_msgs__action__ExePath_GetResult_Event__Sequence__copy(
  const mbf_msgs__action__ExePath_GetResult_Event__Sequence * input,
  mbf_msgs__action__ExePath_GetResult_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbf_msgs__action__ExePath_GetResult_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mbf_msgs__action__ExePath_GetResult_Event * data =
      (mbf_msgs__action__ExePath_GetResult_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbf_msgs__action__ExePath_GetResult_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mbf_msgs__action__ExePath_GetResult_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbf_msgs__action__ExePath_GetResult_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "mbf_msgs/action/detail/exe_path__functions.h"

bool
mbf_msgs__action__ExePath_FeedbackMessage__init(mbf_msgs__action__ExePath_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    mbf_msgs__action__ExePath_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!mbf_msgs__action__ExePath_Feedback__init(&msg->feedback)) {
    mbf_msgs__action__ExePath_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
mbf_msgs__action__ExePath_FeedbackMessage__fini(mbf_msgs__action__ExePath_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  mbf_msgs__action__ExePath_Feedback__fini(&msg->feedback);
}

bool
mbf_msgs__action__ExePath_FeedbackMessage__are_equal(const mbf_msgs__action__ExePath_FeedbackMessage * lhs, const mbf_msgs__action__ExePath_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!mbf_msgs__action__ExePath_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
mbf_msgs__action__ExePath_FeedbackMessage__copy(
  const mbf_msgs__action__ExePath_FeedbackMessage * input,
  mbf_msgs__action__ExePath_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!mbf_msgs__action__ExePath_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

mbf_msgs__action__ExePath_FeedbackMessage *
mbf_msgs__action__ExePath_FeedbackMessage__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_FeedbackMessage * msg = (mbf_msgs__action__ExePath_FeedbackMessage *)allocator.allocate(sizeof(mbf_msgs__action__ExePath_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbf_msgs__action__ExePath_FeedbackMessage));
  bool success = mbf_msgs__action__ExePath_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbf_msgs__action__ExePath_FeedbackMessage__destroy(mbf_msgs__action__ExePath_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbf_msgs__action__ExePath_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbf_msgs__action__ExePath_FeedbackMessage__Sequence__init(mbf_msgs__action__ExePath_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_FeedbackMessage * data = NULL;

  if (size) {
    data = (mbf_msgs__action__ExePath_FeedbackMessage *)allocator.zero_allocate(size, sizeof(mbf_msgs__action__ExePath_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbf_msgs__action__ExePath_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbf_msgs__action__ExePath_FeedbackMessage__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mbf_msgs__action__ExePath_FeedbackMessage__Sequence__fini(mbf_msgs__action__ExePath_FeedbackMessage__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mbf_msgs__action__ExePath_FeedbackMessage__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mbf_msgs__action__ExePath_FeedbackMessage__Sequence *
mbf_msgs__action__ExePath_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__action__ExePath_FeedbackMessage__Sequence * array = (mbf_msgs__action__ExePath_FeedbackMessage__Sequence *)allocator.allocate(sizeof(mbf_msgs__action__ExePath_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbf_msgs__action__ExePath_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbf_msgs__action__ExePath_FeedbackMessage__Sequence__destroy(mbf_msgs__action__ExePath_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbf_msgs__action__ExePath_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbf_msgs__action__ExePath_FeedbackMessage__Sequence__are_equal(const mbf_msgs__action__ExePath_FeedbackMessage__Sequence * lhs, const mbf_msgs__action__ExePath_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbf_msgs__action__ExePath_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbf_msgs__action__ExePath_FeedbackMessage__Sequence__copy(
  const mbf_msgs__action__ExePath_FeedbackMessage__Sequence * input,
  mbf_msgs__action__ExePath_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbf_msgs__action__ExePath_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mbf_msgs__action__ExePath_FeedbackMessage * data =
      (mbf_msgs__action__ExePath_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbf_msgs__action__ExePath_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mbf_msgs__action__ExePath_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbf_msgs__action__ExePath_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
