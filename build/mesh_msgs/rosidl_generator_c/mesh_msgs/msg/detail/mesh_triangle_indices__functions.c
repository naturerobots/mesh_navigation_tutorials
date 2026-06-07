// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mesh_msgs:msg/MeshTriangleIndices.idl
// generated code does not contain a copyright notice
#include "mesh_msgs/msg/detail/mesh_triangle_indices__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
mesh_msgs__msg__MeshTriangleIndices__init(mesh_msgs__msg__MeshTriangleIndices * msg)
{
  if (!msg) {
    return false;
  }
  // vertex_indices
  return true;
}

void
mesh_msgs__msg__MeshTriangleIndices__fini(mesh_msgs__msg__MeshTriangleIndices * msg)
{
  if (!msg) {
    return;
  }
  // vertex_indices
}

bool
mesh_msgs__msg__MeshTriangleIndices__are_equal(const mesh_msgs__msg__MeshTriangleIndices * lhs, const mesh_msgs__msg__MeshTriangleIndices * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // vertex_indices
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->vertex_indices[i] != rhs->vertex_indices[i]) {
      return false;
    }
  }
  return true;
}

bool
mesh_msgs__msg__MeshTriangleIndices__copy(
  const mesh_msgs__msg__MeshTriangleIndices * input,
  mesh_msgs__msg__MeshTriangleIndices * output)
{
  if (!input || !output) {
    return false;
  }
  // vertex_indices
  for (size_t i = 0; i < 3; ++i) {
    output->vertex_indices[i] = input->vertex_indices[i];
  }
  return true;
}

mesh_msgs__msg__MeshTriangleIndices *
mesh_msgs__msg__MeshTriangleIndices__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshTriangleIndices * msg = (mesh_msgs__msg__MeshTriangleIndices *)allocator.allocate(sizeof(mesh_msgs__msg__MeshTriangleIndices), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mesh_msgs__msg__MeshTriangleIndices));
  bool success = mesh_msgs__msg__MeshTriangleIndices__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mesh_msgs__msg__MeshTriangleIndices__destroy(mesh_msgs__msg__MeshTriangleIndices * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mesh_msgs__msg__MeshTriangleIndices__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mesh_msgs__msg__MeshTriangleIndices__Sequence__init(mesh_msgs__msg__MeshTriangleIndices__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshTriangleIndices * data = NULL;

  if (size) {
    data = (mesh_msgs__msg__MeshTriangleIndices *)allocator.zero_allocate(size, sizeof(mesh_msgs__msg__MeshTriangleIndices), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mesh_msgs__msg__MeshTriangleIndices__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mesh_msgs__msg__MeshTriangleIndices__fini(&data[i - 1]);
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
mesh_msgs__msg__MeshTriangleIndices__Sequence__fini(mesh_msgs__msg__MeshTriangleIndices__Sequence * array)
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
      mesh_msgs__msg__MeshTriangleIndices__fini(&array->data[i]);
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

mesh_msgs__msg__MeshTriangleIndices__Sequence *
mesh_msgs__msg__MeshTriangleIndices__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshTriangleIndices__Sequence * array = (mesh_msgs__msg__MeshTriangleIndices__Sequence *)allocator.allocate(sizeof(mesh_msgs__msg__MeshTriangleIndices__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mesh_msgs__msg__MeshTriangleIndices__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mesh_msgs__msg__MeshTriangleIndices__Sequence__destroy(mesh_msgs__msg__MeshTriangleIndices__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mesh_msgs__msg__MeshTriangleIndices__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mesh_msgs__msg__MeshTriangleIndices__Sequence__are_equal(const mesh_msgs__msg__MeshTriangleIndices__Sequence * lhs, const mesh_msgs__msg__MeshTriangleIndices__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mesh_msgs__msg__MeshTriangleIndices__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mesh_msgs__msg__MeshTriangleIndices__Sequence__copy(
  const mesh_msgs__msg__MeshTriangleIndices__Sequence * input,
  mesh_msgs__msg__MeshTriangleIndices__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mesh_msgs__msg__MeshTriangleIndices);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mesh_msgs__msg__MeshTriangleIndices * data =
      (mesh_msgs__msg__MeshTriangleIndices *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mesh_msgs__msg__MeshTriangleIndices__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mesh_msgs__msg__MeshTriangleIndices__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mesh_msgs__msg__MeshTriangleIndices__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
