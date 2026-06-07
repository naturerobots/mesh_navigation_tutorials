// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mbf_msgs:srv/SetTestRobotState.idl
// generated code does not contain a copyright notice
#include "mbf_msgs/srv/detail/set_test_robot_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `transform`
#include "geometry_msgs/msg/detail/transform_stamped__functions.h"
// Member `velocity_robot`
#include "geometry_msgs/msg/detail/twist_stamped__functions.h"

bool
mbf_msgs__srv__SetTestRobotState_Request__init(mbf_msgs__srv__SetTestRobotState_Request * msg)
{
  if (!msg) {
    return false;
  }
  // transform
  if (!geometry_msgs__msg__TransformStamped__init(&msg->transform)) {
    mbf_msgs__srv__SetTestRobotState_Request__fini(msg);
    return false;
  }
  // set_velocity
  // velocity_robot
  if (!geometry_msgs__msg__TwistStamped__init(&msg->velocity_robot)) {
    mbf_msgs__srv__SetTestRobotState_Request__fini(msg);
    return false;
  }
  return true;
}

void
mbf_msgs__srv__SetTestRobotState_Request__fini(mbf_msgs__srv__SetTestRobotState_Request * msg)
{
  if (!msg) {
    return;
  }
  // transform
  geometry_msgs__msg__TransformStamped__fini(&msg->transform);
  // set_velocity
  // velocity_robot
  geometry_msgs__msg__TwistStamped__fini(&msg->velocity_robot);
}

bool
mbf_msgs__srv__SetTestRobotState_Request__are_equal(const mbf_msgs__srv__SetTestRobotState_Request * lhs, const mbf_msgs__srv__SetTestRobotState_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // transform
  if (!geometry_msgs__msg__TransformStamped__are_equal(
      &(lhs->transform), &(rhs->transform)))
  {
    return false;
  }
  // set_velocity
  if (lhs->set_velocity != rhs->set_velocity) {
    return false;
  }
  // velocity_robot
  if (!geometry_msgs__msg__TwistStamped__are_equal(
      &(lhs->velocity_robot), &(rhs->velocity_robot)))
  {
    return false;
  }
  return true;
}

bool
mbf_msgs__srv__SetTestRobotState_Request__copy(
  const mbf_msgs__srv__SetTestRobotState_Request * input,
  mbf_msgs__srv__SetTestRobotState_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // transform
  if (!geometry_msgs__msg__TransformStamped__copy(
      &(input->transform), &(output->transform)))
  {
    return false;
  }
  // set_velocity
  output->set_velocity = input->set_velocity;
  // velocity_robot
  if (!geometry_msgs__msg__TwistStamped__copy(
      &(input->velocity_robot), &(output->velocity_robot)))
  {
    return false;
  }
  return true;
}

mbf_msgs__srv__SetTestRobotState_Request *
mbf_msgs__srv__SetTestRobotState_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__srv__SetTestRobotState_Request * msg = (mbf_msgs__srv__SetTestRobotState_Request *)allocator.allocate(sizeof(mbf_msgs__srv__SetTestRobotState_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbf_msgs__srv__SetTestRobotState_Request));
  bool success = mbf_msgs__srv__SetTestRobotState_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbf_msgs__srv__SetTestRobotState_Request__destroy(mbf_msgs__srv__SetTestRobotState_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbf_msgs__srv__SetTestRobotState_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbf_msgs__srv__SetTestRobotState_Request__Sequence__init(mbf_msgs__srv__SetTestRobotState_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__srv__SetTestRobotState_Request * data = NULL;

  if (size) {
    data = (mbf_msgs__srv__SetTestRobotState_Request *)allocator.zero_allocate(size, sizeof(mbf_msgs__srv__SetTestRobotState_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbf_msgs__srv__SetTestRobotState_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbf_msgs__srv__SetTestRobotState_Request__fini(&data[i - 1]);
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
mbf_msgs__srv__SetTestRobotState_Request__Sequence__fini(mbf_msgs__srv__SetTestRobotState_Request__Sequence * array)
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
      mbf_msgs__srv__SetTestRobotState_Request__fini(&array->data[i]);
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

mbf_msgs__srv__SetTestRobotState_Request__Sequence *
mbf_msgs__srv__SetTestRobotState_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__srv__SetTestRobotState_Request__Sequence * array = (mbf_msgs__srv__SetTestRobotState_Request__Sequence *)allocator.allocate(sizeof(mbf_msgs__srv__SetTestRobotState_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbf_msgs__srv__SetTestRobotState_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbf_msgs__srv__SetTestRobotState_Request__Sequence__destroy(mbf_msgs__srv__SetTestRobotState_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbf_msgs__srv__SetTestRobotState_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbf_msgs__srv__SetTestRobotState_Request__Sequence__are_equal(const mbf_msgs__srv__SetTestRobotState_Request__Sequence * lhs, const mbf_msgs__srv__SetTestRobotState_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbf_msgs__srv__SetTestRobotState_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbf_msgs__srv__SetTestRobotState_Request__Sequence__copy(
  const mbf_msgs__srv__SetTestRobotState_Request__Sequence * input,
  mbf_msgs__srv__SetTestRobotState_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbf_msgs__srv__SetTestRobotState_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mbf_msgs__srv__SetTestRobotState_Request * data =
      (mbf_msgs__srv__SetTestRobotState_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbf_msgs__srv__SetTestRobotState_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mbf_msgs__srv__SetTestRobotState_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbf_msgs__srv__SetTestRobotState_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
mbf_msgs__srv__SetTestRobotState_Response__init(mbf_msgs__srv__SetTestRobotState_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
mbf_msgs__srv__SetTestRobotState_Response__fini(mbf_msgs__srv__SetTestRobotState_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
mbf_msgs__srv__SetTestRobotState_Response__are_equal(const mbf_msgs__srv__SetTestRobotState_Response * lhs, const mbf_msgs__srv__SetTestRobotState_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
mbf_msgs__srv__SetTestRobotState_Response__copy(
  const mbf_msgs__srv__SetTestRobotState_Response * input,
  mbf_msgs__srv__SetTestRobotState_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

mbf_msgs__srv__SetTestRobotState_Response *
mbf_msgs__srv__SetTestRobotState_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__srv__SetTestRobotState_Response * msg = (mbf_msgs__srv__SetTestRobotState_Response *)allocator.allocate(sizeof(mbf_msgs__srv__SetTestRobotState_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbf_msgs__srv__SetTestRobotState_Response));
  bool success = mbf_msgs__srv__SetTestRobotState_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbf_msgs__srv__SetTestRobotState_Response__destroy(mbf_msgs__srv__SetTestRobotState_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbf_msgs__srv__SetTestRobotState_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbf_msgs__srv__SetTestRobotState_Response__Sequence__init(mbf_msgs__srv__SetTestRobotState_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__srv__SetTestRobotState_Response * data = NULL;

  if (size) {
    data = (mbf_msgs__srv__SetTestRobotState_Response *)allocator.zero_allocate(size, sizeof(mbf_msgs__srv__SetTestRobotState_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbf_msgs__srv__SetTestRobotState_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbf_msgs__srv__SetTestRobotState_Response__fini(&data[i - 1]);
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
mbf_msgs__srv__SetTestRobotState_Response__Sequence__fini(mbf_msgs__srv__SetTestRobotState_Response__Sequence * array)
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
      mbf_msgs__srv__SetTestRobotState_Response__fini(&array->data[i]);
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

mbf_msgs__srv__SetTestRobotState_Response__Sequence *
mbf_msgs__srv__SetTestRobotState_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__srv__SetTestRobotState_Response__Sequence * array = (mbf_msgs__srv__SetTestRobotState_Response__Sequence *)allocator.allocate(sizeof(mbf_msgs__srv__SetTestRobotState_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbf_msgs__srv__SetTestRobotState_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbf_msgs__srv__SetTestRobotState_Response__Sequence__destroy(mbf_msgs__srv__SetTestRobotState_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbf_msgs__srv__SetTestRobotState_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbf_msgs__srv__SetTestRobotState_Response__Sequence__are_equal(const mbf_msgs__srv__SetTestRobotState_Response__Sequence * lhs, const mbf_msgs__srv__SetTestRobotState_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbf_msgs__srv__SetTestRobotState_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbf_msgs__srv__SetTestRobotState_Response__Sequence__copy(
  const mbf_msgs__srv__SetTestRobotState_Response__Sequence * input,
  mbf_msgs__srv__SetTestRobotState_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbf_msgs__srv__SetTestRobotState_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mbf_msgs__srv__SetTestRobotState_Response * data =
      (mbf_msgs__srv__SetTestRobotState_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbf_msgs__srv__SetTestRobotState_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mbf_msgs__srv__SetTestRobotState_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbf_msgs__srv__SetTestRobotState_Response__copy(
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
// #include "mbf_msgs/srv/detail/set_test_robot_state__functions.h"

bool
mbf_msgs__srv__SetTestRobotState_Event__init(mbf_msgs__srv__SetTestRobotState_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    mbf_msgs__srv__SetTestRobotState_Event__fini(msg);
    return false;
  }
  // request
  if (!mbf_msgs__srv__SetTestRobotState_Request__Sequence__init(&msg->request, 0)) {
    mbf_msgs__srv__SetTestRobotState_Event__fini(msg);
    return false;
  }
  // response
  if (!mbf_msgs__srv__SetTestRobotState_Response__Sequence__init(&msg->response, 0)) {
    mbf_msgs__srv__SetTestRobotState_Event__fini(msg);
    return false;
  }
  return true;
}

void
mbf_msgs__srv__SetTestRobotState_Event__fini(mbf_msgs__srv__SetTestRobotState_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  mbf_msgs__srv__SetTestRobotState_Request__Sequence__fini(&msg->request);
  // response
  mbf_msgs__srv__SetTestRobotState_Response__Sequence__fini(&msg->response);
}

bool
mbf_msgs__srv__SetTestRobotState_Event__are_equal(const mbf_msgs__srv__SetTestRobotState_Event * lhs, const mbf_msgs__srv__SetTestRobotState_Event * rhs)
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
  if (!mbf_msgs__srv__SetTestRobotState_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!mbf_msgs__srv__SetTestRobotState_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
mbf_msgs__srv__SetTestRobotState_Event__copy(
  const mbf_msgs__srv__SetTestRobotState_Event * input,
  mbf_msgs__srv__SetTestRobotState_Event * output)
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
  if (!mbf_msgs__srv__SetTestRobotState_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!mbf_msgs__srv__SetTestRobotState_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

mbf_msgs__srv__SetTestRobotState_Event *
mbf_msgs__srv__SetTestRobotState_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__srv__SetTestRobotState_Event * msg = (mbf_msgs__srv__SetTestRobotState_Event *)allocator.allocate(sizeof(mbf_msgs__srv__SetTestRobotState_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbf_msgs__srv__SetTestRobotState_Event));
  bool success = mbf_msgs__srv__SetTestRobotState_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbf_msgs__srv__SetTestRobotState_Event__destroy(mbf_msgs__srv__SetTestRobotState_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbf_msgs__srv__SetTestRobotState_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbf_msgs__srv__SetTestRobotState_Event__Sequence__init(mbf_msgs__srv__SetTestRobotState_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__srv__SetTestRobotState_Event * data = NULL;

  if (size) {
    data = (mbf_msgs__srv__SetTestRobotState_Event *)allocator.zero_allocate(size, sizeof(mbf_msgs__srv__SetTestRobotState_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbf_msgs__srv__SetTestRobotState_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbf_msgs__srv__SetTestRobotState_Event__fini(&data[i - 1]);
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
mbf_msgs__srv__SetTestRobotState_Event__Sequence__fini(mbf_msgs__srv__SetTestRobotState_Event__Sequence * array)
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
      mbf_msgs__srv__SetTestRobotState_Event__fini(&array->data[i]);
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

mbf_msgs__srv__SetTestRobotState_Event__Sequence *
mbf_msgs__srv__SetTestRobotState_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbf_msgs__srv__SetTestRobotState_Event__Sequence * array = (mbf_msgs__srv__SetTestRobotState_Event__Sequence *)allocator.allocate(sizeof(mbf_msgs__srv__SetTestRobotState_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbf_msgs__srv__SetTestRobotState_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbf_msgs__srv__SetTestRobotState_Event__Sequence__destroy(mbf_msgs__srv__SetTestRobotState_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbf_msgs__srv__SetTestRobotState_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbf_msgs__srv__SetTestRobotState_Event__Sequence__are_equal(const mbf_msgs__srv__SetTestRobotState_Event__Sequence * lhs, const mbf_msgs__srv__SetTestRobotState_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbf_msgs__srv__SetTestRobotState_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbf_msgs__srv__SetTestRobotState_Event__Sequence__copy(
  const mbf_msgs__srv__SetTestRobotState_Event__Sequence * input,
  mbf_msgs__srv__SetTestRobotState_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbf_msgs__srv__SetTestRobotState_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mbf_msgs__srv__SetTestRobotState_Event * data =
      (mbf_msgs__srv__SetTestRobotState_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbf_msgs__srv__SetTestRobotState_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mbf_msgs__srv__SetTestRobotState_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbf_msgs__srv__SetTestRobotState_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
