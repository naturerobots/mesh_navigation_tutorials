// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mesh_msgs:msg/MeshGeometryStamped.idl
// generated code does not contain a copyright notice
#include "mesh_msgs/msg/detail/mesh_geometry_stamped__functions.h"

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
// Member `mesh_geometry`
#include "mesh_msgs/msg/detail/mesh_geometry__functions.h"

bool
mesh_msgs__msg__MeshGeometryStamped__init(mesh_msgs__msg__MeshGeometryStamped * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mesh_msgs__msg__MeshGeometryStamped__fini(msg);
    return false;
  }
  // uuid
  if (!rosidl_runtime_c__String__init(&msg->uuid)) {
    mesh_msgs__msg__MeshGeometryStamped__fini(msg);
    return false;
  }
  // mesh_geometry
  if (!mesh_msgs__msg__MeshGeometry__init(&msg->mesh_geometry)) {
    mesh_msgs__msg__MeshGeometryStamped__fini(msg);
    return false;
  }
  return true;
}

void
mesh_msgs__msg__MeshGeometryStamped__fini(mesh_msgs__msg__MeshGeometryStamped * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // uuid
  rosidl_runtime_c__String__fini(&msg->uuid);
  // mesh_geometry
  mesh_msgs__msg__MeshGeometry__fini(&msg->mesh_geometry);
}

bool
mesh_msgs__msg__MeshGeometryStamped__are_equal(const mesh_msgs__msg__MeshGeometryStamped * lhs, const mesh_msgs__msg__MeshGeometryStamped * rhs)
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
  // mesh_geometry
  if (!mesh_msgs__msg__MeshGeometry__are_equal(
      &(lhs->mesh_geometry), &(rhs->mesh_geometry)))
  {
    return false;
  }
  return true;
}

bool
mesh_msgs__msg__MeshGeometryStamped__copy(
  const mesh_msgs__msg__MeshGeometryStamped * input,
  mesh_msgs__msg__MeshGeometryStamped * output)
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
  // mesh_geometry
  if (!mesh_msgs__msg__MeshGeometry__copy(
      &(input->mesh_geometry), &(output->mesh_geometry)))
  {
    return false;
  }
  return true;
}

mesh_msgs__msg__MeshGeometryStamped *
mesh_msgs__msg__MeshGeometryStamped__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshGeometryStamped * msg = (mesh_msgs__msg__MeshGeometryStamped *)allocator.allocate(sizeof(mesh_msgs__msg__MeshGeometryStamped), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mesh_msgs__msg__MeshGeometryStamped));
  bool success = mesh_msgs__msg__MeshGeometryStamped__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mesh_msgs__msg__MeshGeometryStamped__destroy(mesh_msgs__msg__MeshGeometryStamped * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mesh_msgs__msg__MeshGeometryStamped__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mesh_msgs__msg__MeshGeometryStamped__Sequence__init(mesh_msgs__msg__MeshGeometryStamped__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshGeometryStamped * data = NULL;

  if (size) {
    data = (mesh_msgs__msg__MeshGeometryStamped *)allocator.zero_allocate(size, sizeof(mesh_msgs__msg__MeshGeometryStamped), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mesh_msgs__msg__MeshGeometryStamped__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mesh_msgs__msg__MeshGeometryStamped__fini(&data[i - 1]);
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
mesh_msgs__msg__MeshGeometryStamped__Sequence__fini(mesh_msgs__msg__MeshGeometryStamped__Sequence * array)
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
      mesh_msgs__msg__MeshGeometryStamped__fini(&array->data[i]);
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

mesh_msgs__msg__MeshGeometryStamped__Sequence *
mesh_msgs__msg__MeshGeometryStamped__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshGeometryStamped__Sequence * array = (mesh_msgs__msg__MeshGeometryStamped__Sequence *)allocator.allocate(sizeof(mesh_msgs__msg__MeshGeometryStamped__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mesh_msgs__msg__MeshGeometryStamped__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mesh_msgs__msg__MeshGeometryStamped__Sequence__destroy(mesh_msgs__msg__MeshGeometryStamped__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mesh_msgs__msg__MeshGeometryStamped__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mesh_msgs__msg__MeshGeometryStamped__Sequence__are_equal(const mesh_msgs__msg__MeshGeometryStamped__Sequence * lhs, const mesh_msgs__msg__MeshGeometryStamped__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mesh_msgs__msg__MeshGeometryStamped__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mesh_msgs__msg__MeshGeometryStamped__Sequence__copy(
  const mesh_msgs__msg__MeshGeometryStamped__Sequence * input,
  mesh_msgs__msg__MeshGeometryStamped__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mesh_msgs__msg__MeshGeometryStamped);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mesh_msgs__msg__MeshGeometryStamped * data =
      (mesh_msgs__msg__MeshGeometryStamped *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mesh_msgs__msg__MeshGeometryStamped__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mesh_msgs__msg__MeshGeometryStamped__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mesh_msgs__msg__MeshGeometryStamped__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
