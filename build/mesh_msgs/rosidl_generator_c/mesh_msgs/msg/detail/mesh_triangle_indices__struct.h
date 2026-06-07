// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mesh_msgs:msg/MeshTriangleIndices.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_triangle_indices.h"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_TRIANGLE_INDICES__STRUCT_H_
#define MESH_MSGS__MSG__DETAIL__MESH_TRIANGLE_INDICES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/MeshTriangleIndices in the package mesh_msgs.
/**
  * Definition of a triangle's vertices
 */
typedef struct mesh_msgs__msg__MeshTriangleIndices
{
  uint32_t vertex_indices[3];
} mesh_msgs__msg__MeshTriangleIndices;

// Struct for a sequence of mesh_msgs__msg__MeshTriangleIndices.
typedef struct mesh_msgs__msg__MeshTriangleIndices__Sequence
{
  mesh_msgs__msg__MeshTriangleIndices * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__msg__MeshTriangleIndices__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESH_MSGS__MSG__DETAIL__MESH_TRIANGLE_INDICES__STRUCT_H_
