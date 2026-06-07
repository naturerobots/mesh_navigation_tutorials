// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mesh_msgs:msg/MeshVertexColors.idl
// generated code does not contain a copyright notice
#include "mesh_msgs/msg/detail/mesh_vertex_colors__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `vertex_colors`
#include "std_msgs/msg/detail/color_rgba__functions.h"

bool
mesh_msgs__msg__MeshVertexColors__init(mesh_msgs__msg__MeshVertexColors * msg)
{
  if (!msg) {
    return false;
  }
  // vertex_colors
  if (!std_msgs__msg__ColorRGBA__Sequence__init(&msg->vertex_colors, 0)) {
    mesh_msgs__msg__MeshVertexColors__fini(msg);
    return false;
  }
  return true;
}

void
mesh_msgs__msg__MeshVertexColors__fini(mesh_msgs__msg__MeshVertexColors * msg)
{
  if (!msg) {
    return;
  }
  // vertex_colors
  std_msgs__msg__ColorRGBA__Sequence__fini(&msg->vertex_colors);
}

bool
mesh_msgs__msg__MeshVertexColors__are_equal(const mesh_msgs__msg__MeshVertexColors * lhs, const mesh_msgs__msg__MeshVertexColors * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // vertex_colors
  if (!std_msgs__msg__ColorRGBA__Sequence__are_equal(
      &(lhs->vertex_colors), &(rhs->vertex_colors)))
  {
    return false;
  }
  return true;
}

bool
mesh_msgs__msg__MeshVertexColors__copy(
  const mesh_msgs__msg__MeshVertexColors * input,
  mesh_msgs__msg__MeshVertexColors * output)
{
  if (!input || !output) {
    return false;
  }
  // vertex_colors
  if (!std_msgs__msg__ColorRGBA__Sequence__copy(
      &(input->vertex_colors), &(output->vertex_colors)))
  {
    return false;
  }
  return true;
}

mesh_msgs__msg__MeshVertexColors *
mesh_msgs__msg__MeshVertexColors__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshVertexColors * msg = (mesh_msgs__msg__MeshVertexColors *)allocator.allocate(sizeof(mesh_msgs__msg__MeshVertexColors), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mesh_msgs__msg__MeshVertexColors));
  bool success = mesh_msgs__msg__MeshVertexColors__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mesh_msgs__msg__MeshVertexColors__destroy(mesh_msgs__msg__MeshVertexColors * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mesh_msgs__msg__MeshVertexColors__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mesh_msgs__msg__MeshVertexColors__Sequence__init(mesh_msgs__msg__MeshVertexColors__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshVertexColors * data = NULL;

  if (size) {
    data = (mesh_msgs__msg__MeshVertexColors *)allocator.zero_allocate(size, sizeof(mesh_msgs__msg__MeshVertexColors), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mesh_msgs__msg__MeshVertexColors__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mesh_msgs__msg__MeshVertexColors__fini(&data[i - 1]);
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
mesh_msgs__msg__MeshVertexColors__Sequence__fini(mesh_msgs__msg__MeshVertexColors__Sequence * array)
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
      mesh_msgs__msg__MeshVertexColors__fini(&array->data[i]);
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

mesh_msgs__msg__MeshVertexColors__Sequence *
mesh_msgs__msg__MeshVertexColors__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshVertexColors__Sequence * array = (mesh_msgs__msg__MeshVertexColors__Sequence *)allocator.allocate(sizeof(mesh_msgs__msg__MeshVertexColors__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mesh_msgs__msg__MeshVertexColors__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mesh_msgs__msg__MeshVertexColors__Sequence__destroy(mesh_msgs__msg__MeshVertexColors__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mesh_msgs__msg__MeshVertexColors__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mesh_msgs__msg__MeshVertexColors__Sequence__are_equal(const mesh_msgs__msg__MeshVertexColors__Sequence * lhs, const mesh_msgs__msg__MeshVertexColors__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mesh_msgs__msg__MeshVertexColors__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mesh_msgs__msg__MeshVertexColors__Sequence__copy(
  const mesh_msgs__msg__MeshVertexColors__Sequence * input,
  mesh_msgs__msg__MeshVertexColors__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mesh_msgs__msg__MeshVertexColors);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mesh_msgs__msg__MeshVertexColors * data =
      (mesh_msgs__msg__MeshVertexColors *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mesh_msgs__msg__MeshVertexColors__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mesh_msgs__msg__MeshVertexColors__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mesh_msgs__msg__MeshVertexColors__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
