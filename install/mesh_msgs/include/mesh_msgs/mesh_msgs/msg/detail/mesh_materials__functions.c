// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mesh_msgs:msg/MeshMaterials.idl
// generated code does not contain a copyright notice
#include "mesh_msgs/msg/detail/mesh_materials__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `clusters`
#include "mesh_msgs/msg/detail/mesh_face_cluster__functions.h"
// Member `materials`
#include "mesh_msgs/msg/detail/mesh_material__functions.h"
// Member `cluster_materials`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `vertex_tex_coords`
#include "mesh_msgs/msg/detail/mesh_vertex_tex_coords__functions.h"

bool
mesh_msgs__msg__MeshMaterials__init(mesh_msgs__msg__MeshMaterials * msg)
{
  if (!msg) {
    return false;
  }
  // clusters
  if (!mesh_msgs__msg__MeshFaceCluster__Sequence__init(&msg->clusters, 0)) {
    mesh_msgs__msg__MeshMaterials__fini(msg);
    return false;
  }
  // materials
  if (!mesh_msgs__msg__MeshMaterial__Sequence__init(&msg->materials, 0)) {
    mesh_msgs__msg__MeshMaterials__fini(msg);
    return false;
  }
  // cluster_materials
  if (!rosidl_runtime_c__uint32__Sequence__init(&msg->cluster_materials, 0)) {
    mesh_msgs__msg__MeshMaterials__fini(msg);
    return false;
  }
  // vertex_tex_coords
  if (!mesh_msgs__msg__MeshVertexTexCoords__Sequence__init(&msg->vertex_tex_coords, 0)) {
    mesh_msgs__msg__MeshMaterials__fini(msg);
    return false;
  }
  return true;
}

void
mesh_msgs__msg__MeshMaterials__fini(mesh_msgs__msg__MeshMaterials * msg)
{
  if (!msg) {
    return;
  }
  // clusters
  mesh_msgs__msg__MeshFaceCluster__Sequence__fini(&msg->clusters);
  // materials
  mesh_msgs__msg__MeshMaterial__Sequence__fini(&msg->materials);
  // cluster_materials
  rosidl_runtime_c__uint32__Sequence__fini(&msg->cluster_materials);
  // vertex_tex_coords
  mesh_msgs__msg__MeshVertexTexCoords__Sequence__fini(&msg->vertex_tex_coords);
}

bool
mesh_msgs__msg__MeshMaterials__are_equal(const mesh_msgs__msg__MeshMaterials * lhs, const mesh_msgs__msg__MeshMaterials * rhs)
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
  // materials
  if (!mesh_msgs__msg__MeshMaterial__Sequence__are_equal(
      &(lhs->materials), &(rhs->materials)))
  {
    return false;
  }
  // cluster_materials
  if (!rosidl_runtime_c__uint32__Sequence__are_equal(
      &(lhs->cluster_materials), &(rhs->cluster_materials)))
  {
    return false;
  }
  // vertex_tex_coords
  if (!mesh_msgs__msg__MeshVertexTexCoords__Sequence__are_equal(
      &(lhs->vertex_tex_coords), &(rhs->vertex_tex_coords)))
  {
    return false;
  }
  return true;
}

bool
mesh_msgs__msg__MeshMaterials__copy(
  const mesh_msgs__msg__MeshMaterials * input,
  mesh_msgs__msg__MeshMaterials * output)
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
  // materials
  if (!mesh_msgs__msg__MeshMaterial__Sequence__copy(
      &(input->materials), &(output->materials)))
  {
    return false;
  }
  // cluster_materials
  if (!rosidl_runtime_c__uint32__Sequence__copy(
      &(input->cluster_materials), &(output->cluster_materials)))
  {
    return false;
  }
  // vertex_tex_coords
  if (!mesh_msgs__msg__MeshVertexTexCoords__Sequence__copy(
      &(input->vertex_tex_coords), &(output->vertex_tex_coords)))
  {
    return false;
  }
  return true;
}

mesh_msgs__msg__MeshMaterials *
mesh_msgs__msg__MeshMaterials__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshMaterials * msg = (mesh_msgs__msg__MeshMaterials *)allocator.allocate(sizeof(mesh_msgs__msg__MeshMaterials), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mesh_msgs__msg__MeshMaterials));
  bool success = mesh_msgs__msg__MeshMaterials__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mesh_msgs__msg__MeshMaterials__destroy(mesh_msgs__msg__MeshMaterials * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mesh_msgs__msg__MeshMaterials__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mesh_msgs__msg__MeshMaterials__Sequence__init(mesh_msgs__msg__MeshMaterials__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshMaterials * data = NULL;

  if (size) {
    data = (mesh_msgs__msg__MeshMaterials *)allocator.zero_allocate(size, sizeof(mesh_msgs__msg__MeshMaterials), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mesh_msgs__msg__MeshMaterials__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mesh_msgs__msg__MeshMaterials__fini(&data[i - 1]);
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
mesh_msgs__msg__MeshMaterials__Sequence__fini(mesh_msgs__msg__MeshMaterials__Sequence * array)
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
      mesh_msgs__msg__MeshMaterials__fini(&array->data[i]);
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

mesh_msgs__msg__MeshMaterials__Sequence *
mesh_msgs__msg__MeshMaterials__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mesh_msgs__msg__MeshMaterials__Sequence * array = (mesh_msgs__msg__MeshMaterials__Sequence *)allocator.allocate(sizeof(mesh_msgs__msg__MeshMaterials__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mesh_msgs__msg__MeshMaterials__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mesh_msgs__msg__MeshMaterials__Sequence__destroy(mesh_msgs__msg__MeshMaterials__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mesh_msgs__msg__MeshMaterials__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mesh_msgs__msg__MeshMaterials__Sequence__are_equal(const mesh_msgs__msg__MeshMaterials__Sequence * lhs, const mesh_msgs__msg__MeshMaterials__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mesh_msgs__msg__MeshMaterials__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mesh_msgs__msg__MeshMaterials__Sequence__copy(
  const mesh_msgs__msg__MeshMaterials__Sequence * input,
  mesh_msgs__msg__MeshMaterials__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mesh_msgs__msg__MeshMaterials);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mesh_msgs__msg__MeshMaterials * data =
      (mesh_msgs__msg__MeshMaterials *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mesh_msgs__msg__MeshMaterials__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mesh_msgs__msg__MeshMaterials__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mesh_msgs__msg__MeshMaterials__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
