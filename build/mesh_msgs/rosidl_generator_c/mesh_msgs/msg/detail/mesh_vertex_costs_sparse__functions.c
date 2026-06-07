// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mesh_msgs:msg/MeshVertexCostsSparse.idl
// generated code does not contain a copyright notice
#include "mesh_msgs/msg/detail/mesh_vertex_costs_sparse__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `vertices`
// Member `costs`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
mesh_msgs__msg__MeshVertexCostsSparse__init(mesh_msgs__msg__MeshVertexCostsSparse * msg)
{
  if (!msg) {
    return false;
  }
  // vertices
  if (!rosidl_runtime_c__uint32__Sequence__init(&msg->vertices, 0)) {
    mesh_msgs__msg__MeshVertexCostsSparse__fini(msg);
    return false;
  }
  // costs
  if (!rosidl_runtime_c__float__Sequence__init(&msg->costs, 0)) {
    mesh_msgs__msg__MeshVertexCostsSparse__fini(msg);
    return false;
  }
  // default_value
  return true;
}

void
mesh_msgs__msg__MeshVertexCostsSparse__fini(mesh_msgs__msg__MeshVertexCostsSparse * msg)
{
  if (!msg) {
    return;
  }
  // vertices
  rosidl_runtime_c__uint32__Sequence__fini(&msg->vertices);
  // costs
  rosidl_runtime_c__float__Sequence__fini(&msg->costs);
  // default_value
}

bool
mesh_msgs__msg__MeshVertexCostsSparse__are_equal(const mesh_msgs__msg__MeshVertexCostsSparse * lhs, const mesh_msgs__msg__MeshVertexCostsSparse * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // vertices
  if (!rosidl_runtime_c__uint32__Sequence__are_equal(
      &(lhs->vertices), &(rhs->vertices)))
  {
    return false;
  }
  // costs
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->costs), &(rhs->costs)))
  {
    return false;
  }
  // default_value
  if (lhs->default_value != rhs->default_value) {
    return false;
  }
  return true;
}

bool
mesh_msgs__msg__MeshVertexCostsSparse__copy(
  const mesh_msgs__msg__MeshVertexCostsSparse * input,
  mesh_msgs__msg__MeshVertexCostsSparse * output)
{
  if (!input || !output) {
    return false;
  }
  // vertices
  if (!rosidl_runtime_c__uint32__Sequence__copy(
      &(input->vertices), &(output->vertices)))
  {
    return false;
  }
  // costs
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->costs), &(output->costs)))
  {
    return false;
  }
  // default_value
  output->default_value = input->default_value;
  return true;
}

mesh_msgs__msg__MeshVertexCostsSparse *
mesh_msgs__msg__MeshVertexCostsSparse__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshVertexCostsSparse * msg = (mesh_msgs__msg__MeshVertexCostsSparse *)allocator.allocate(sizeof(mesh_msgs__msg__MeshVertexCostsSparse), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mesh_msgs__msg__MeshVertexCostsSparse));
  bool success = mesh_msgs__msg__MeshVertexCostsSparse__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mesh_msgs__msg__MeshVertexCostsSparse__destroy(mesh_msgs__msg__MeshVertexCostsSparse * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mesh_msgs__msg__MeshVertexCostsSparse__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mesh_msgs__msg__MeshVertexCostsSparse__Sequence__init(mesh_msgs__msg__MeshVertexCostsSparse__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshVertexCostsSparse * data = NULL;

  if (size) {
    data = (mesh_msgs__msg__MeshVertexCostsSparse *)allocator.zero_allocate(size, sizeof(mesh_msgs__msg__MeshVertexCostsSparse), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mesh_msgs__msg__MeshVertexCostsSparse__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mesh_msgs__msg__MeshVertexCostsSparse__fini(&data[i - 1]);
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
mesh_msgs__msg__MeshVertexCostsSparse__Sequence__fini(mesh_msgs__msg__MeshVertexCostsSparse__Sequence * array)
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
      mesh_msgs__msg__MeshVertexCostsSparse__fini(&array->data[i]);
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

mesh_msgs__msg__MeshVertexCostsSparse__Sequence *
mesh_msgs__msg__MeshVertexCostsSparse__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshVertexCostsSparse__Sequence * array = (mesh_msgs__msg__MeshVertexCostsSparse__Sequence *)allocator.allocate(sizeof(mesh_msgs__msg__MeshVertexCostsSparse__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mesh_msgs__msg__MeshVertexCostsSparse__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mesh_msgs__msg__MeshVertexCostsSparse__Sequence__destroy(mesh_msgs__msg__MeshVertexCostsSparse__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mesh_msgs__msg__MeshVertexCostsSparse__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mesh_msgs__msg__MeshVertexCostsSparse__Sequence__are_equal(const mesh_msgs__msg__MeshVertexCostsSparse__Sequence * lhs, const mesh_msgs__msg__MeshVertexCostsSparse__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mesh_msgs__msg__MeshVertexCostsSparse__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mesh_msgs__msg__MeshVertexCostsSparse__Sequence__copy(
  const mesh_msgs__msg__MeshVertexCostsSparse__Sequence * input,
  mesh_msgs__msg__MeshVertexCostsSparse__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mesh_msgs__msg__MeshVertexCostsSparse);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mesh_msgs__msg__MeshVertexCostsSparse * data =
      (mesh_msgs__msg__MeshVertexCostsSparse *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mesh_msgs__msg__MeshVertexCostsSparse__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mesh_msgs__msg__MeshVertexCostsSparse__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mesh_msgs__msg__MeshVertexCostsSparse__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
