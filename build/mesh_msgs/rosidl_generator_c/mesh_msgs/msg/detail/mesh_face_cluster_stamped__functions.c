// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mesh_msgs:msg/MeshFaceClusterStamped.idl
// generated code does not contain a copyright notice
#include "mesh_msgs/msg/detail/mesh_face_cluster_stamped__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `uuid`
#include "rosidl_runtime_c/string_functions.h"
// Member `cluster`
#include "mesh_msgs/msg/detail/mesh_face_cluster__functions.h"

bool
mesh_msgs__msg__MeshFaceClusterStamped__init(mesh_msgs__msg__MeshFaceClusterStamped * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mesh_msgs__msg__MeshFaceClusterStamped__fini(msg);
    return false;
  }
  // uuid
  if (!rosidl_runtime_c__String__init(&msg->uuid)) {
    mesh_msgs__msg__MeshFaceClusterStamped__fini(msg);
    return false;
  }
  // cluster
  if (!mesh_msgs__msg__MeshFaceCluster__init(&msg->cluster)) {
    mesh_msgs__msg__MeshFaceClusterStamped__fini(msg);
    return false;
  }
  // override
  return true;
}

void
mesh_msgs__msg__MeshFaceClusterStamped__fini(mesh_msgs__msg__MeshFaceClusterStamped * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // uuid
  rosidl_runtime_c__String__fini(&msg->uuid);
  // cluster
  mesh_msgs__msg__MeshFaceCluster__fini(&msg->cluster);
  // override
}

bool
mesh_msgs__msg__MeshFaceClusterStamped__are_equal(const mesh_msgs__msg__MeshFaceClusterStamped * lhs, const mesh_msgs__msg__MeshFaceClusterStamped * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // uuid
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->uuid), &(rhs->uuid)))
  {
    return false;
  }
  // cluster
  if (!mesh_msgs__msg__MeshFaceCluster__are_equal(
      &(lhs->cluster), &(rhs->cluster)))
  {
    return false;
  }
  // override
  if (lhs->override != rhs->override) {
    return false;
  }
  return true;
}

bool
mesh_msgs__msg__MeshFaceClusterStamped__copy(
  const mesh_msgs__msg__MeshFaceClusterStamped * input,
  mesh_msgs__msg__MeshFaceClusterStamped * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // uuid
  if (!rosidl_runtime_c__String__copy(
      &(input->uuid), &(output->uuid)))
  {
    return false;
  }
  // cluster
  if (!mesh_msgs__msg__MeshFaceCluster__copy(
      &(input->cluster), &(output->cluster)))
  {
    return false;
  }
  // override
  output->override = input->override;
  return true;
}

mesh_msgs__msg__MeshFaceClusterStamped *
mesh_msgs__msg__MeshFaceClusterStamped__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshFaceClusterStamped * msg = (mesh_msgs__msg__MeshFaceClusterStamped *)allocator.allocate(sizeof(mesh_msgs__msg__MeshFaceClusterStamped), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mesh_msgs__msg__MeshFaceClusterStamped));
  bool success = mesh_msgs__msg__MeshFaceClusterStamped__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mesh_msgs__msg__MeshFaceClusterStamped__destroy(mesh_msgs__msg__MeshFaceClusterStamped * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mesh_msgs__msg__MeshFaceClusterStamped__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mesh_msgs__msg__MeshFaceClusterStamped__Sequence__init(mesh_msgs__msg__MeshFaceClusterStamped__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshFaceClusterStamped * data = NULL;

  if (size) {
    data = (mesh_msgs__msg__MeshFaceClusterStamped *)allocator.zero_allocate(size, sizeof(mesh_msgs__msg__MeshFaceClusterStamped), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mesh_msgs__msg__MeshFaceClusterStamped__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mesh_msgs__msg__MeshFaceClusterStamped__fini(&data[i - 1]);
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
mesh_msgs__msg__MeshFaceClusterStamped__Sequence__fini(mesh_msgs__msg__MeshFaceClusterStamped__Sequence * array)
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
      mesh_msgs__msg__MeshFaceClusterStamped__fini(&array->data[i]);
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

mesh_msgs__msg__MeshFaceClusterStamped__Sequence *
mesh_msgs__msg__MeshFaceClusterStamped__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshFaceClusterStamped__Sequence * array = (mesh_msgs__msg__MeshFaceClusterStamped__Sequence *)allocator.allocate(sizeof(mesh_msgs__msg__MeshFaceClusterStamped__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mesh_msgs__msg__MeshFaceClusterStamped__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mesh_msgs__msg__MeshFaceClusterStamped__Sequence__destroy(mesh_msgs__msg__MeshFaceClusterStamped__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mesh_msgs__msg__MeshFaceClusterStamped__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mesh_msgs__msg__MeshFaceClusterStamped__Sequence__are_equal(const mesh_msgs__msg__MeshFaceClusterStamped__Sequence * lhs, const mesh_msgs__msg__MeshFaceClusterStamped__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mesh_msgs__msg__MeshFaceClusterStamped__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mesh_msgs__msg__MeshFaceClusterStamped__Sequence__copy(
  const mesh_msgs__msg__MeshFaceClusterStamped__Sequence * input,
  mesh_msgs__msg__MeshFaceClusterStamped__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mesh_msgs__msg__MeshFaceClusterStamped);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mesh_msgs__msg__MeshFaceClusterStamped * data =
      (mesh_msgs__msg__MeshFaceClusterStamped *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mesh_msgs__msg__MeshFaceClusterStamped__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mesh_msgs__msg__MeshFaceClusterStamped__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mesh_msgs__msg__MeshFaceClusterStamped__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
