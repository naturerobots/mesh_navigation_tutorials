// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mesh_msgs:msg/MeshVertexCostsStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_vertex_costs_stamped.h"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS_STAMPED__STRUCT_H_
#define MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS_STAMPED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'uuid'
// Member 'type'
#include "rosidl_runtime_c/string.h"
// Member 'mesh_vertex_costs'
#include "mesh_msgs/msg/detail/mesh_vertex_costs__struct.h"

/// Struct defined in msg/MeshVertexCostsStamped in the package mesh_msgs.
/**
  * Mesh Attribute Message
 */
typedef struct mesh_msgs__msg__MeshVertexCostsStamped
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String uuid;
  rosidl_runtime_c__String type;
  mesh_msgs__msg__MeshVertexCosts mesh_vertex_costs;
} mesh_msgs__msg__MeshVertexCostsStamped;

// Struct for a sequence of mesh_msgs__msg__MeshVertexCostsStamped.
typedef struct mesh_msgs__msg__MeshVertexCostsStamped__Sequence
{
  mesh_msgs__msg__MeshVertexCostsStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__msg__MeshVertexCostsStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS_STAMPED__STRUCT_H_
