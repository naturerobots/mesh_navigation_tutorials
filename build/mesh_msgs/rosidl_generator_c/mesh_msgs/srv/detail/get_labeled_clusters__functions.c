// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mesh_msgs:srv/GetLabeledClusters.idl
// generated code does not contain a copyright notice
#include "mesh_msgs/srv/detail/get_labeled_clusters__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `uuid`
#include "rosidl_runtime_c/string_functions.h"

bool
mesh_msgs__srv__GetLabeledClusters_Request__init(mesh_msgs__srv__GetLabeledClusters_Request * msg)
{
  if (!msg) {
    return false;
  }
  // uuid
  if (!rosidl_runtime_c__String__init(&msg->uuid)) {
    mesh_msgs__srv__GetLabeledClusters_Request__fini(msg);
    return false;
  }
  return true;
}

void
mesh_msgs__srv__GetLabeledClusters_Request__fini(mesh_msgs__srv__GetLabeledClusters_Request * msg)
{
  if (!msg) {
    return;
  }
  // uuid
  rosidl_runtime_c__String__fini(&msg->uuid);
}

bool
mesh_msgs__srv__GetLabeledClusters_Request__are_equal(const mesh_msgs__srv__GetLabeledClusters_Request * lhs, const mesh_msgs__srv__GetLabeledClusters_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // uuid
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->uuid), &(rhs->uuid)))
  {
    return false;
  }
  return true;
}

bool
mesh_msgs__srv__GetLabeledClusters_Request__copy(
  const mesh_msgs__srv__GetLabeledClusters_Request * input,
  mesh_msgs__srv__GetLabeledClusters_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // uuid
  if (!rosidl_runtime_c__String__copy(
      &(input->uuid), &(output->uuid)))
  {
    return false;
  }
  return true;
}

mesh_msgs__srv__GetLabeledClusters_Request *
mesh_msgs__srv__GetLabeledClusters_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__srv__GetLabeledClusters_Request * msg = (mesh_msgs__srv__GetLabeledClusters_Request *)allocator.allocate(sizeof(mesh_msgs__srv__GetLabeledClusters_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mesh_msgs__srv__GetLabeledClusters_Request));
  bool success = mesh_msgs__srv__GetLabeledClusters_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mesh_msgs__srv__GetLabeledClusters_Request__destroy(mesh_msgs__srv__GetLabeledClusters_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mesh_msgs__srv__GetLabeledClusters_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mesh_msgs__srv__GetLabeledClusters_Request__Sequence__init(mesh_msgs__srv__GetLabeledClusters_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__srv__GetLabeledClusters_Request * data = NULL;

  if (size) {
    data = (mesh_msgs__srv__GetLabeledClusters_Request *)allocator.zero_allocate(size, sizeof(mesh_msgs__srv__GetLabeledClusters_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mesh_msgs__srv__GetLabeledClusters_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mesh_msgs__srv__GetLabeledClusters_Request__fini(&data[i - 1]);
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
mesh_msgs__srv__GetLabeledClusters_Request__Sequence__fini(mesh_msgs__srv__GetLabeledClusters_Request__Sequence * array)
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
      mesh_msgs__srv__GetLabeledClusters_Request__fini(&array->data[i]);
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

mesh_msgs__srv__GetLabeledClusters_Request__Sequence *
mesh_msgs__srv__GetLabeledClusters_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__srv__GetLabeledClusters_Request__Sequence * array = (mesh_msgs__srv__GetLabeledClusters_Request__Sequence *)allocator.allocate(sizeof(mesh_msgs__srv__GetLabeledClusters_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mesh_msgs__srv__GetLabeledClusters_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mesh_msgs__srv__GetLabeledClusters_Request__Sequence__destroy(mesh_msgs__srv__GetLabeledClusters_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mesh_msgs__srv__GetLabeledClusters_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mesh_msgs__srv__GetLabeledClusters_Request__Sequence__are_equal(const mesh_msgs__srv__GetLabeledClusters_Request__Sequence * lhs, const mesh_msgs__srv__GetLabeledClusters_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mesh_msgs__srv__GetLabeledClusters_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mesh_msgs__srv__GetLabeledClusters_Request__Sequence__copy(
  const mesh_msgs__srv__GetLabeledClusters_Request__Sequence * input,
  mesh_msgs__srv__GetLabeledClusters_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mesh_msgs__srv__GetLabeledClusters_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mesh_msgs__srv__GetLabeledClusters_Request * data =
      (mesh_msgs__srv__GetLabeledClusters_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mesh_msgs__srv__GetLabeledClusters_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mesh_msgs__srv__GetLabeledClusters_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mesh_msgs__srv__GetLabeledClusters_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `clusters`
#include "mesh_msgs/msg/detail/mesh_face_cluster__functions.h"

bool
mesh_msgs__srv__GetLabeledClusters_Response__init(mesh_msgs__srv__GetLabeledClusters_Response * msg)
{
  if (!msg) {
    return false;
  }
  // clusters
  if (!mesh_msgs__msg__MeshFaceCluster__Sequence__init(&msg->clusters, 0)) {
    mesh_msgs__srv__GetLabeledClusters_Response__fini(msg);
    return false;
  }
  return true;
}

void
mesh_msgs__srv__GetLabeledClusters_Response__fini(mesh_msgs__srv__GetLabeledClusters_Response * msg)
{
  if (!msg) {
    return;
  }
  // clusters
  mesh_msgs__msg__MeshFaceCluster__Sequence__fini(&msg->clusters);
}

bool
mesh_msgs__srv__GetLabeledClusters_Response__are_equal(const mesh_msgs__srv__GetLabeledClusters_Response * lhs, const mesh_msgs__srv__GetLabeledClusters_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // clusters
  if (!mesh_msgs__msg__MeshFaceCluster__Sequence__are_equal(
      &(lhs->clusters), &(rhs->clusters)))
  {
    return false;
  }
  return true;
}

bool
mesh_msgs__srv__GetLabeledClusters_Response__copy(
  const mesh_msgs__srv__GetLabeledClusters_Response * input,
  mesh_msgs__srv__GetLabeledClusters_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // clusters
  if (!mesh_msgs__msg__MeshFaceCluster__Sequence__copy(
      &(input->clusters), &(output->clusters)))
  {
    return false;
  }
  return true;
}

mesh_msgs__srv__GetLabeledClusters_Response *
mesh_msgs__srv__GetLabeledClusters_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__srv__GetLabeledClusters_Response * msg = (mesh_msgs__srv__GetLabeledClusters_Response *)allocator.allocate(sizeof(mesh_msgs__srv__GetLabeledClusters_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mesh_msgs__srv__GetLabeledClusters_Response));
  bool success = mesh_msgs__srv__GetLabeledClusters_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mesh_msgs__srv__GetLabeledClusters_Response__destroy(mesh_msgs__srv__GetLabeledClusters_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mesh_msgs__srv__GetLabeledClusters_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mesh_msgs__srv__GetLabeledClusters_Response__Sequence__init(mesh_msgs__srv__GetLabeledClusters_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__srv__GetLabeledClusters_Response * data = NULL;

  if (size) {
    data = (mesh_msgs__srv__GetLabeledClusters_Response *)allocator.zero_allocate(size, sizeof(mesh_msgs__srv__GetLabeledClusters_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mesh_msgs__srv__GetLabeledClusters_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mesh_msgs__srv__GetLabeledClusters_Response__fini(&data[i - 1]);
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
mesh_msgs__srv__GetLabeledClusters_Response__Sequence__fini(mesh_msgs__srv__GetLabeledClusters_Response__Sequence * array)
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
      mesh_msgs__srv__GetLabeledClusters_Response__fini(&array->data[i]);
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

mesh_msgs__srv__GetLabeledClusters_Response__Sequence *
mesh_msgs__srv__GetLabeledClusters_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__srv__GetLabeledClusters_Response__Sequence * array = (mesh_msgs__srv__GetLabeledClusters_Response__Sequence *)allocator.allocate(sizeof(mesh_msgs__srv__GetLabeledClusters_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mesh_msgs__srv__GetLabeledClusters_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mesh_msgs__srv__GetLabeledClusters_Response__Sequence__destroy(mesh_msgs__srv__GetLabeledClusters_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mesh_msgs__srv__GetLabeledClusters_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mesh_msgs__srv__GetLabeledClusters_Response__Sequence__are_equal(const mesh_msgs__srv__GetLabeledClusters_Response__Sequence * lhs, const mesh_msgs__srv__GetLabeledClusters_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mesh_msgs__srv__GetLabeledClusters_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mesh_msgs__srv__GetLabeledClusters_Response__Sequence__copy(
  const mesh_msgs__srv__GetLabeledClusters_Response__Sequence * input,
  mesh_msgs__srv__GetLabeledClusters_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mesh_msgs__srv__GetLabeledClusters_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mesh_msgs__srv__GetLabeledClusters_Response * data =
      (mesh_msgs__srv__GetLabeledClusters_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mesh_msgs__srv__GetLabeledClusters_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mesh_msgs__srv__GetLabeledClusters_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mesh_msgs__srv__GetLabeledClusters_Response__copy(
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
// #include "mesh_msgs/srv/detail/get_labeled_clusters__functions.h"

bool
mesh_msgs__srv__GetLabeledClusters_Event__init(mesh_msgs__srv__GetLabeledClusters_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    mesh_msgs__srv__GetLabeledClusters_Event__fini(msg);
    return false;
  }
  // request
  if (!mesh_msgs__srv__GetLabeledClusters_Request__Sequence__init(&msg->request, 0)) {
    mesh_msgs__srv__GetLabeledClusters_Event__fini(msg);
    return false;
  }
  // response
  if (!mesh_msgs__srv__GetLabeledClusters_Response__Sequence__init(&msg->response, 0)) {
    mesh_msgs__srv__GetLabeledClusters_Event__fini(msg);
    return false;
  }
  return true;
}

void
mesh_msgs__srv__GetLabeledClusters_Event__fini(mesh_msgs__srv__GetLabeledClusters_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  mesh_msgs__srv__GetLabeledClusters_Request__Sequence__fini(&msg->request);
  // response
  mesh_msgs__srv__GetLabeledClusters_Response__Sequence__fini(&msg->response);
}

bool
mesh_msgs__srv__GetLabeledClusters_Event__are_equal(const mesh_msgs__srv__GetLabeledClusters_Event * lhs, const mesh_msgs__srv__GetLabeledClusters_Event * rhs)
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
  if (!mesh_msgs__srv__GetLabeledClusters_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!mesh_msgs__srv__GetLabeledClusters_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
mesh_msgs__srv__GetLabeledClusters_Event__copy(
  const mesh_msgs__srv__GetLabeledClusters_Event * input,
  mesh_msgs__srv__GetLabeledClusters_Event * output)
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
  if (!mesh_msgs__srv__GetLabeledClusters_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!mesh_msgs__srv__GetLabeledClusters_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

mesh_msgs__srv__GetLabeledClusters_Event *
mesh_msgs__srv__GetLabeledClusters_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__srv__GetLabeledClusters_Event * msg = (mesh_msgs__srv__GetLabeledClusters_Event *)allocator.allocate(sizeof(mesh_msgs__srv__GetLabeledClusters_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mesh_msgs__srv__GetLabeledClusters_Event));
  bool success = mesh_msgs__srv__GetLabeledClusters_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mesh_msgs__srv__GetLabeledClusters_Event__destroy(mesh_msgs__srv__GetLabeledClusters_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mesh_msgs__srv__GetLabeledClusters_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mesh_msgs__srv__GetLabeledClusters_Event__Sequence__init(mesh_msgs__srv__GetLabeledClusters_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__srv__GetLabeledClusters_Event * data = NULL;

  if (size) {
    data = (mesh_msgs__srv__GetLabeledClusters_Event *)allocator.zero_allocate(size, sizeof(mesh_msgs__srv__GetLabeledClusters_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mesh_msgs__srv__GetLabeledClusters_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mesh_msgs__srv__GetLabeledClusters_Event__fini(&data[i - 1]);
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
mesh_msgs__srv__GetLabeledClusters_Event__Sequence__fini(mesh_msgs__srv__GetLabeledClusters_Event__Sequence * array)
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
      mesh_msgs__srv__GetLabeledClusters_Event__fini(&array->data[i]);
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

mesh_msgs__srv__GetLabeledClusters_Event__Sequence *
mesh_msgs__srv__GetLabeledClusters_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__srv__GetLabeledClusters_Event__Sequence * array = (mesh_msgs__srv__GetLabeledClusters_Event__Sequence *)allocator.allocate(sizeof(mesh_msgs__srv__GetLabeledClusters_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mesh_msgs__srv__GetLabeledClusters_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mesh_msgs__srv__GetLabeledClusters_Event__Sequence__destroy(mesh_msgs__srv__GetLabeledClusters_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mesh_msgs__srv__GetLabeledClusters_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mesh_msgs__srv__GetLabeledClusters_Event__Sequence__are_equal(const mesh_msgs__srv__GetLabeledClusters_Event__Sequence * lhs, const mesh_msgs__srv__GetLabeledClusters_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mesh_msgs__srv__GetLabeledClusters_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mesh_msgs__srv__GetLabeledClusters_Event__Sequence__copy(
  const mesh_msgs__srv__GetLabeledClusters_Event__Sequence * input,
  mesh_msgs__srv__GetLabeledClusters_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mesh_msgs__srv__GetLabeledClusters_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mesh_msgs__srv__GetLabeledClusters_Event * data =
      (mesh_msgs__srv__GetLabeledClusters_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mesh_msgs__srv__GetLabeledClusters_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mesh_msgs__srv__GetLabeledClusters_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mesh_msgs__srv__GetLabeledClusters_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
