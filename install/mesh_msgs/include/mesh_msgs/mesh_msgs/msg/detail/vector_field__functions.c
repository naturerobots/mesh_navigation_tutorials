// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mesh_msgs:msg/VectorField.idl
// generated code does not contain a copyright notice
#include "mesh_msgs/msg/detail/vector_field__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `positions`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `vectors`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
mesh_msgs__msg__VectorField__init(mesh_msgs__msg__VectorField * msg)
{
  if (!msg) {
    return false;
  }
  // positions
  if (!geometry_msgs__msg__Point__Sequence__init(&msg->positions, 0)) {
    mesh_msgs__msg__VectorField__fini(msg);
    return false;
  }
  // vectors
  if (!geometry_msgs__msg__Vector3__Sequence__init(&msg->vectors, 0)) {
    mesh_msgs__msg__VectorField__fini(msg);
    return false;
  }
  return true;
}

void
mesh_msgs__msg__VectorField__fini(mesh_msgs__msg__VectorField * msg)
{
  if (!msg) {
    return;
  }
  // positions
  geometry_msgs__msg__Point__Sequence__fini(&msg->positions);
  // vectors
  geometry_msgs__msg__Vector3__Sequence__fini(&msg->vectors);
}

bool
mesh_msgs__msg__VectorField__are_equal(const mesh_msgs__msg__VectorField * lhs, const mesh_msgs__msg__VectorField * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // positions
  if (!geometry_msgs__msg__Point__Sequence__are_equal(
      &(lhs->positions), &(rhs->positions)))
  {
    return false;
  }
  // vectors
  if (!geometry_msgs__msg__Vector3__Sequence__are_equal(
      &(lhs->vectors), &(rhs->vectors)))
  {
    return false;
  }
  return true;
}

bool
mesh_msgs__msg__VectorField__copy(
  const mesh_msgs__msg__VectorField * input,
  mesh_msgs__msg__VectorField * output)
{
  if (!input || !output) {
    return false;
  }
  // positions
  if (!geometry_msgs__msg__Point__Sequence__copy(
      &(input->positions), &(output->positions)))
  {
    return false;
  }
  // vectors
  if (!geometry_msgs__msg__Vector3__Sequence__copy(
      &(input->vectors), &(output->vectors)))
  {
    return false;
  }
  return true;
}

mesh_msgs__msg__VectorField *
mesh_msgs__msg__VectorField__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__VectorField * msg = (mesh_msgs__msg__VectorField *)allocator.allocate(sizeof(mesh_msgs__msg__VectorField), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mesh_msgs__msg__VectorField));
  bool success = mesh_msgs__msg__VectorField__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mesh_msgs__msg__VectorField__destroy(mesh_msgs__msg__VectorField * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mesh_msgs__msg__VectorField__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mesh_msgs__msg__VectorField__Sequence__init(mesh_msgs__msg__VectorField__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__VectorField * data = NULL;

  if (size) {
    data = (mesh_msgs__msg__VectorField *)allocator.zero_allocate(size, sizeof(mesh_msgs__msg__VectorField), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mesh_msgs__msg__VectorField__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mesh_msgs__msg__VectorField__fini(&data[i - 1]);
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
mesh_msgs__msg__VectorField__Sequence__fini(mesh_msgs__msg__VectorField__Sequence * array)
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
      mesh_msgs__msg__VectorField__fini(&array->data[i]);
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

mesh_msgs__msg__VectorField__Sequence *
mesh_msgs__msg__VectorField__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__VectorField__Sequence * array = (mesh_msgs__msg__VectorField__Sequence *)allocator.allocate(sizeof(mesh_msgs__msg__VectorField__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mesh_msgs__msg__VectorField__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mesh_msgs__msg__VectorField__Sequence__destroy(mesh_msgs__msg__VectorField__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mesh_msgs__msg__VectorField__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mesh_msgs__msg__VectorField__Sequence__are_equal(const mesh_msgs__msg__VectorField__Sequence * lhs, const mesh_msgs__msg__VectorField__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mesh_msgs__msg__VectorField__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mesh_msgs__msg__VectorField__Sequence__copy(
  const mesh_msgs__msg__VectorField__Sequence * input,
  mesh_msgs__msg__VectorField__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mesh_msgs__msg__VectorField);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mesh_msgs__msg__VectorField * data =
      (mesh_msgs__msg__VectorField *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mesh_msgs__msg__VectorField__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mesh_msgs__msg__VectorField__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mesh_msgs__msg__VectorField__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
