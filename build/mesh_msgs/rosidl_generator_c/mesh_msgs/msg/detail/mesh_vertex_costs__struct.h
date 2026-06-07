// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mesh_msgs:msg/MeshVertexCosts.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_vertex_costs.h"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS__STRUCT_H_
#define MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'costs'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/MeshVertexCosts in the package mesh_msgs.
/**
  * Mesh Attribute Message
 */
typedef struct mesh_msgs__msg__MeshVertexCosts
{
  rosidl_runtime_c__float__Sequence costs;
} mesh_msgs__msg__MeshVertexCosts;

// Struct for a sequence of mesh_msgs__msg__MeshVertexCosts.
typedef struct mesh_msgs__msg__MeshVertexCosts__Sequence
{
  mesh_msgs__msg__MeshVertexCosts * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__msg__MeshVertexCosts__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS__STRUCT_H_
