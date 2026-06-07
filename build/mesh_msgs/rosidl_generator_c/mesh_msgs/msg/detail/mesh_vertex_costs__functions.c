// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mesh_msgs:msg/MeshVertexCosts.idl
// generated code does not contain a copyright notice
#include "mesh_msgs/msg/detail/mesh_vertex_costs__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `costs`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
mesh_msgs__msg__MeshVertexCosts__init(mesh_msgs__msg__MeshVertexCosts * msg)
{
  if (!msg) {
    return false;
  }
  // costs
  if (!rosidl_runtime_c__float__Sequence__init(&msg->costs, 0)) {
    mesh_msgs__msg__MeshVertexCosts__fini(msg);
    return false;
  }
  return true;
}

void
mesh_msgs__msg__MeshVertexCosts__fini(mesh_msgs__msg__MeshVertexCosts * msg)
{
  if (!msg) {
    return;
  }
  // costs
  rosidl_runtime_c__float__Sequence__fini(&msg->costs);
}

bool
mesh_msgs__msg__MeshVertexCosts__are_equal(const mesh_msgs__msg__MeshVertexCosts * lhs, const mesh_msgs__msg__MeshVertexCosts * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // costs
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->costs), &(rhs->costs)))
  {
    return false;
  }
  return true;
}

bool
mesh_msgs__msg__MeshVertexCosts__copy(
  const mesh_msgs__msg__MeshVertexCosts * input,
  mesh_msgs__msg__MeshVertexCosts * output)
{
  if (!input || !output) {
    return false;
  }
  // costs
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->costs), &(output->costs)))
  {
    return false;
  }
  return true;
}

mesh_msgs__msg__MeshVertexCosts *
mesh_msgs__msg__MeshVertexCosts__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshVertexCosts * msg = (mesh_msgs__msg__MeshVertexCosts *)allocator.allocate(sizeof(mesh_msgs__msg__MeshVertexCosts), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mesh_msgs__msg__MeshVertexCosts));
  bool success = mesh_msgs__msg__MeshVertexCosts__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mesh_msgs__msg__MeshVertexCosts__destroy(mesh_msgs__msg__MeshVertexCosts * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mesh_msgs__msg__MeshVertexCosts__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mesh_msgs__msg__MeshVertexCosts__Sequence__init(mesh_msgs__msg__MeshVertexCosts__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshVertexCosts * data = NULL;

  if (size) {
    data = (mesh_msgs__msg__MeshVertexCosts *)allocator.zero_allocate(size, sizeof(mesh_msgs__msg__MeshVertexCosts), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mesh_msgs__msg__MeshVertexCosts__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mesh_msgs__msg__MeshVertexCosts__fini(&data[i - 1]);
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
mesh_msgs__msg__MeshVertexCosts__Sequence__fini(mesh_msgs__msg__MeshVertexCosts__Sequence * array)
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
      mesh_msgs__msg__MeshVertexCosts__fini(&array->data[i]);
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

mesh_msgs__msg__MeshVertexCosts__Sequence *
mesh_msgs__msg__MeshVertexCosts__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshVertexCosts__Sequence * array = (mesh_msgs__msg__MeshVertexCosts__Sequence *)allocator.allocate(sizeof(mesh_msgs__msg__MeshVertexCosts__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mesh_msgs__msg__MeshVertexCosts__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mesh_msgs__msg__MeshVertexCosts__Sequence__destroy(mesh_msgs__msg__MeshVertexCosts__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mesh_msgs__msg__MeshVertexCosts__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mesh_msgs__msg__MeshVertexCosts__Sequence__are_equal(const mesh_msgs__msg__MeshVertexCosts__Sequence * lhs, const mesh_msgs__msg__MeshVertexCosts__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mesh_msgs__msg__MeshVertexCosts__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mesh_msgs__msg__MeshVertexCosts__Sequence__copy(
  const mesh_msgs__msg__MeshVertexCosts__Sequence * input,
  mesh_msgs__msg__MeshVertexCosts__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mesh_msgs__msg__MeshVertexCosts);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mesh_msgs__msg__MeshVertexCosts * data =
      (mesh_msgs__msg__MeshVertexCosts *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mesh_msgs__msg__MeshVertexCosts__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mesh_msgs__msg__MeshVertexCosts__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mesh_msgs__msg__MeshVertexCosts__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
