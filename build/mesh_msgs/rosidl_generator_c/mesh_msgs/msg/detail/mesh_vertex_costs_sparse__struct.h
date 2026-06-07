// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mesh_msgs:msg/MeshVertexCostsSparse.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_vertex_costs_sparse.h"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS_SPARSE__STRUCT_H_
#define MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS_SPARSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'vertices'
// Member 'costs'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/MeshVertexCostsSparse in the package mesh_msgs.
/**
  * MeshVertexCostsSparse
  * Holds costs for a subset of the mesh vertices
 */
typedef struct mesh_msgs__msg__MeshVertexCostsSparse
{
  /// The vertex indices
  rosidl_runtime_c__uint32__Sequence vertices;
  /// The cost per vertex
  rosidl_runtime_c__float__Sequence costs;
  /// The default value for all other vertices
  float default_value;
} mesh_msgs__msg__MeshVertexCostsSparse;

// Struct for a sequence of mesh_msgs__msg__MeshVertexCostsSparse.
typedef struct mesh_msgs__msg__MeshVertexCostsSparse__Sequence
{
  mesh_msgs__msg__MeshVertexCostsSparse * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__msg__MeshVertexCostsSparse__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS_SPARSE__STRUCT_H_
