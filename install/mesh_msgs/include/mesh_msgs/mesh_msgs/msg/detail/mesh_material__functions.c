// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mesh_msgs:msg/MeshMaterial.idl
// generated code does not contain a copyright notice
#include "mesh_msgs/msg/detail/mesh_material__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `color`
#include "std_msgs/msg/detail/color_rgba__functions.h"

bool
mesh_msgs__msg__MeshMaterial__init(mesh_msgs__msg__MeshMaterial * msg)
{
  if (!msg) {
    return false;
  }
  // texture_index
  // color
  if (!std_msgs__msg__ColorRGBA__init(&msg->color)) {
    mesh_msgs__msg__MeshMaterial__fini(msg);
    return false;
  }
  // has_texture
  return true;
}

void
mesh_msgs__msg__MeshMaterial__fini(mesh_msgs__msg__MeshMaterial * msg)
{
  if (!msg) {
    return;
  }
  // texture_index
  // color
  std_msgs__msg__ColorRGBA__fini(&msg->color);
  // has_texture
}

bool
mesh_msgs__msg__MeshMaterial__are_equal(const mesh_msgs__msg__MeshMaterial * lhs, const mesh_msgs__msg__MeshMaterial * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // texture_index
  if (lhs->texture_index != rhs->texture_index) {
    return false;
  }
  // color
  if (!std_msgs__msg__ColorRGBA__are_equal(
      &(lhs->color), &(rhs->color)))
  {
    return false;
  }
  // has_texture
  if (lhs->has_texture != rhs->has_texture) {
    return false;
  }
  return true;
}

bool
mesh_msgs__msg__MeshMaterial__copy(
  const mesh_msgs__msg__MeshMaterial * input,
  mesh_msgs__msg__MeshMaterial * output)
{
  if (!input || !output) {
    return false;
  }
  // texture_index
  output->texture_index = input->texture_index;
  // color
  if (!std_msgs__msg__ColorRGBA__copy(
      &(input->color), &(output->color)))
  {
    return false;
  }
  // has_texture
  output->has_texture = input->has_texture;
  return true;
}

mesh_msgs__msg__MeshMaterial *
mesh_msgs__msg__MeshMaterial__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshMaterial * msg = (mesh_msgs__msg__MeshMaterial *)allocator.allocate(sizeof(mesh_msgs__msg__MeshMaterial), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mesh_msgs__msg__MeshMaterial));
  bool success = mesh_msgs__msg__MeshMaterial__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mesh_msgs__msg__MeshMaterial__destroy(mesh_msgs__msg__MeshMaterial * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mesh_msgs__msg__MeshMaterial__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mesh_msgs__msg__MeshMaterial__Sequence__init(mesh_msgs__msg__MeshMaterial__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshMaterial * data = NULL;

  if (size) {
    data = (mesh_msgs__msg__MeshMaterial *)allocator.zero_allocate(size, sizeof(mesh_msgs__msg__MeshMaterial), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mesh_msgs__msg__MeshMaterial__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mesh_msgs__msg__MeshMaterial__fini(&data[i - 1]);
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
mesh_msgs__msg__MeshMaterial__Sequence__fini(mesh_msgs__msg__MeshMaterial__Sequence * array)
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
      mesh_msgs__msg__MeshMaterial__fini(&array->data[i]);
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

mesh_msgs__msg__MeshMaterial__Sequence *
mesh_msgs__msg__MeshMaterial__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshMaterial__Sequence * array = (mesh_msgs__msg__MeshMaterial__Sequence *)allocator.allocate(sizeof(mesh_msgs__msg__MeshMaterial__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mesh_msgs__msg__MeshMaterial__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mesh_msgs__msg__MeshMaterial__Sequence__destroy(mesh_msgs__msg__MeshMaterial__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mesh_msgs__msg__MeshMaterial__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mesh_msgs__msg__MeshMaterial__Sequence__are_equal(const mesh_msgs__msg__MeshMaterial__Sequence * lhs, const mesh_msgs__msg__MeshMaterial__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mesh_msgs__msg__MeshMaterial__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mesh_msgs__msg__MeshMaterial__Sequence__copy(
  const mesh_msgs__msg__MeshMaterial__Sequence * input,
  mesh_msgs__msg__MeshMaterial__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mesh_msgs__msg__MeshMaterial);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mesh_msgs__msg__MeshMaterial * data =
      (mesh_msgs__msg__MeshMaterial *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mesh_msgs__msg__MeshMaterial__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mesh_msgs__msg__MeshMaterial__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mesh_msgs__msg__MeshMaterial__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
