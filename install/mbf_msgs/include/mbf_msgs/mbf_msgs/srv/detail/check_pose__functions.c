// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mbf_msgs:srv/CheckPose.idl
// generated code does not contain a copyright notice
#include "mbf_msgs/srv/detail/check_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
mbf_msgs__srv__CheckPose_Request__init(mbf_msgs__srv__CheckPose_Request * msg)
{
  if (!msg) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->pose)) {
    mbf_msgs__srv__CheckPose_Request__fini(msg);
    return false;
  }
  // safety_dist
  // lethal_cost_mult
  // inscrib_cost_mult
  // unknown_cost_mult
  // costmap
  // current_pose
  // use_padded_fp
  return true;
}

void
mbf_msgs__srv__CheckPose_Request__fini(mbf_msgs__srv__CheckPose_Request * msg)
{
  if (!msg) {
    return;
  }
  // pose
  geometry_msgs__msg__PoseStamped__fini(&msg->pose);
  // safety_dist
  // lethal_cost_mult
  // inscrib_cost_mult
  // unknown_cost_mult
  // costmap
  // current_pose
  // use_padded_fp
}

bool
mbf_msgs__srv__CheckPose_Request__are_equal(const mbf_msgs__srv__CheckPose_Request * lhs, const mbf_msgs__srv__CheckPose_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // safety_dist
  if (lhs->safety_dist != rhs->safety_dist) {
    return false;
  }
  // lethal_cost_mult
  if (lhs->lethal_cost_mult != rhs->lethal_cost_mult) {
    return false;
  }
  // inscrib_cost_mult
  if (lhs->inscrib_cost_mult != rhs->inscrib_cost_mult) {
    return false;
  }
  // unknown_cost_mult
  if (lhs->unknown_cost_mult != rhs->unknown_cost_mult) {
    return false;
  }
  // costmap
  if (lhs->costmap != rhs->costmap) {
    return false;
  }
  // current_pose
  if (lhs->current_pose != rhs->current_pose) {
    return false;
  }
  // use_padded_fp
  if (lhs->use_padded_fp != rhs->use_padded_fp) {
    return false;
  }
  return true;
}

bool
mbf_msgs__srv__CheckPose_Request__copy(
  const mbf_msgs__srv__CheckPose_Request * input,
  mbf_msgs__srv__CheckPose_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // safety_dist
  output->safety_dist = input->safety_dist;
  // lethal_cost_mult
  output->lethal_cost_mult = input->lethal_cost_mult;
  // inscrib_cost_mult
  output->inscrib_cost_mult = input->inscrib_cost_mult;
  // unknown_cost_mult
  output->unknown_cost_mult = input->unknown_cost_mult;
  // costmap
  output->costmap = input->costmap;
  // current_pose
  output->current_pose = input->current_pose;
  // use_padded_fp
  output->use_padded_fp = input->use_padded_fp;
  return true;
}

mbf_msgs__srv__CheckPose_Request *
mbf_msgs__srv__CheckPose_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__srv__CheckPose_Request * msg = (mbf_msgs__srv__CheckPose_Request *)allocator.allocate(sizeof(mbf_msgs__srv__CheckPose_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbf_msgs__srv__CheckPose_Request));
  bool success = mbf_msgs__srv__CheckPose_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbf_msgs__srv__CheckPose_Request__destroy(mbf_msgs__srv__CheckPose_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbf_msgs__srv__CheckPose_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbf_msgs__srv__CheckPose_Request__Sequence__init(mbf_msgs__srv__CheckPose_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__srv__CheckPose_Request * data = NULL;

  if (size) {
    data = (mbf_msgs__srv__CheckPose_Request *)allocator.zero_allocate(size, sizeof(mbf_msgs__srv__CheckPose_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbf_msgs__srv__CheckPose_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbf_msgs__srv__CheckPose_Request__fini(&data[i - 1]);
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
mbf_msgs__srv__CheckPose_Request__Sequence__fini(mbf_msgs__srv__CheckPose_Request__Sequence * array)
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
      mbf_msgs__srv__CheckPose_Request__fini(&array->data[i]);
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

mbf_msgs__srv__CheckPose_Request__Sequence *
mbf_msgs__srv__CheckPose_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__srv__CheckPose_Request__Sequence * array = (mbf_msgs__srv__CheckPose_Request__Sequence *)allocator.allocate(sizeof(mbf_msgs__srv__CheckPose_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbf_msgs__srv__CheckPose_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbf_msgs__srv__CheckPose_Request__Sequence__destroy(mbf_msgs__srv__CheckPose_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbf_msgs__srv__CheckPose_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbf_msgs__srv__CheckPose_Request__Sequence__are_equal(const mbf_msgs__srv__CheckPose_Request__Sequence * lhs, const mbf_msgs__srv__CheckPose_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbf_msgs__srv__CheckPose_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbf_msgs__srv__CheckPose_Request__Sequence__copy(
  const mbf_msgs__srv__CheckPose_Request__Sequence * input,
  mbf_msgs__srv__CheckPose_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbf_msgs__srv__CheckPose_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mbf_msgs__srv__CheckPose_Request * data =
      (mbf_msgs__srv__CheckPose_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbf_msgs__srv__CheckPose_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mbf_msgs__srv__CheckPose_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbf_msgs__srv__CheckPose_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
mbf_msgs__srv__CheckPose_Response__init(mbf_msgs__srv__CheckPose_Response * msg)
{
  if (!msg) {
    return false;
  }
  // state
  // cost
  return true;
}

void
mbf_msgs__srv__CheckPose_Response__fini(mbf_msgs__srv__CheckPose_Response * msg)
{
  if (!msg) {
    return;
  }
  // state
  // cost
}

bool
mbf_msgs__srv__CheckPose_Response__are_equal(const mbf_msgs__srv__CheckPose_Response * lhs, const mbf_msgs__srv__CheckPose_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  // cost
  if (lhs->cost != rhs->cost) {
    return false;
  }
  return true;
}

bool
mbf_msgs__srv__CheckPose_Response__copy(
  const mbf_msgs__srv__CheckPose_Response * input,
  mbf_msgs__srv__CheckPose_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // state
  output->state = input->state;
  // cost
  output->cost = input->cost;
  return true;
}

mbf_msgs__srv__CheckPose_Response *
mbf_msgs__srv__CheckPose_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__srv__CheckPose_Response * msg = (mbf_msgs__srv__CheckPose_Response *)allocator.allocate(sizeof(mbf_msgs__srv__CheckPose_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbf_msgs__srv__CheckPose_Response));
  bool success = mbf_msgs__srv__CheckPose_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbf_msgs__srv__CheckPose_Response__destroy(mbf_msgs__srv__CheckPose_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbf_msgs__srv__CheckPose_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbf_msgs__srv__CheckPose_Response__Sequence__init(mbf_msgs__srv__CheckPose_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__srv__CheckPose_Response * data = NULL;

  if (size) {
    data = (mbf_msgs__srv__CheckPose_Response *)allocator.zero_allocate(size, sizeof(mbf_msgs__srv__CheckPose_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbf_msgs__srv__CheckPose_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbf_msgs__srv__CheckPose_Response__fini(&data[i - 1]);
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
mbf_msgs__srv__CheckPose_Response__Sequence__fini(mbf_msgs__srv__CheckPose_Response__Sequence * array)
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
      mbf_msgs__srv__CheckPose_Response__fini(&array->data[i]);
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

mbf_msgs__srv__CheckPose_Response__Sequence *
mbf_msgs__srv__CheckPose_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__srv__CheckPose_Response__Sequence * array = (mbf_msgs__srv__CheckPose_Response__Sequence *)allocator.allocate(sizeof(mbf_msgs__srv__CheckPose_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbf_msgs__srv__CheckPose_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbf_msgs__srv__CheckPose_Response__Sequence__destroy(mbf_msgs__srv__CheckPose_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbf_msgs__srv__CheckPose_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbf_msgs__srv__CheckPose_Response__Sequence__are_equal(const mbf_msgs__srv__CheckPose_Response__Sequence * lhs, const mbf_msgs__srv__CheckPose_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbf_msgs__srv__CheckPose_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbf_msgs__srv__CheckPose_Response__Sequence__copy(
  const mbf_msgs__srv__CheckPose_Response__Sequence * input,
  mbf_msgs__srv__CheckPose_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbf_msgs__srv__CheckPose_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mbf_msgs__srv__CheckPose_Response * data =
      (mbf_msgs__srv__CheckPose_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbf_msgs__srv__CheckPose_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mbf_msgs__srv__CheckPose_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbf_msgs__srv__CheckPose_Response__copy(
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
// #include "mbf_msgs/srv/detail/check_pose__functions.h"

bool
mbf_msgs__srv__CheckPose_Event__init(mbf_msgs__srv__CheckPose_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    mbf_msgs__srv__CheckPose_Event__fini(msg);
    return false;
  }
  // request
  if (!mbf_msgs__srv__CheckPose_Request__Sequence__init(&msg->request, 0)) {
    mbf_msgs__srv__CheckPose_Event__fini(msg);
    return false;
  }
  // response
  if (!mbf_msgs__srv__CheckPose_Response__Sequence__init(&msg->response, 0)) {
    mbf_msgs__srv__CheckPose_Event__fini(msg);
    return false;
  }
  return true;
}

void
mbf_msgs__srv__CheckPose_Event__fini(mbf_msgs__srv__CheckPose_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  mbf_msgs__srv__CheckPose_Request__Sequence__fini(&msg->request);
  // response
  mbf_msgs__srv__CheckPose_Response__Sequence__fini(&msg->response);
}

bool
mbf_msgs__srv__CheckPose_Event__are_equal(const mbf_msgs__srv__CheckPose_Event * lhs, const mbf_msgs__srv__CheckPose_Event * rhs)
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
  if (!mbf_msgs__srv__CheckPose_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!mbf_msgs__srv__CheckPose_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
mbf_msgs__srv__CheckPose_Event__copy(
  const mbf_msgs__srv__CheckPose_Event * input,
  mbf_msgs__srv__CheckPose_Event * output)
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
  if (!mbf_msgs__srv__CheckPose_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!mbf_msgs__srv__CheckPose_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

mbf_msgs__srv__CheckPose_Event *
mbf_msgs__srv__CheckPose_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__srv__CheckPose_Event * msg = (mbf_msgs__srv__CheckPose_Event *)allocator.allocate(sizeof(mbf_msgs__srv__CheckPose_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbf_msgs__srv__CheckPose_Event));
  bool success = mbf_msgs__srv__CheckPose_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbf_msgs__srv__CheckPose_Event__destroy(mbf_msgs__srv__CheckPose_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbf_msgs__srv__CheckPose_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbf_msgs__srv__CheckPose_Event__Sequence__init(mbf_msgs__srv__CheckPose_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__srv__CheckPose_Event * data = NULL;

  if (size) {
    data = (mbf_msgs__srv__CheckPose_Event *)allocator.zero_allocate(size, sizeof(mbf_msgs__srv__CheckPose_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbf_msgs__srv__CheckPose_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbf_msgs__srv__CheckPose_Event__fini(&data[i - 1]);
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
mbf_msgs__srv__CheckPose_Event__Sequence__fini(mbf_msgs__srv__CheckPose_Event__Sequence * array)
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
      mbf_msgs__srv__CheckPose_Event__fini(&array->data[i]);
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

mbf_msgs__srv__CheckPose_Event__Sequence *
mbf_msgs__srv__CheckPose_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__srv__CheckPose_Event__Sequence * array = (mbf_msgs__srv__CheckPose_Event__Sequence *)allocator.allocate(sizeof(mbf_msgs__srv__CheckPose_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbf_msgs__srv__CheckPose_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbf_msgs__srv__CheckPose_Event__Sequence__destroy(mbf_msgs__srv__CheckPose_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbf_msgs__srv__CheckPose_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbf_msgs__srv__CheckPose_Event__Sequence__are_equal(const mbf_msgs__srv__CheckPose_Event__Sequence * lhs, const mbf_msgs__srv__CheckPose_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbf_msgs__srv__CheckPose_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbf_msgs__srv__CheckPose_Event__Sequence__copy(
  const mbf_msgs__srv__CheckPose_Event__Sequence * input,
  mbf_msgs__srv__CheckPose_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbf_msgs__srv__CheckPose_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mbf_msgs__srv__CheckPose_Event * data =
      (mbf_msgs__srv__CheckPose_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbf_msgs__srv__CheckPose_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mbf_msgs__srv__CheckPose_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbf_msgs__srv__CheckPose_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
