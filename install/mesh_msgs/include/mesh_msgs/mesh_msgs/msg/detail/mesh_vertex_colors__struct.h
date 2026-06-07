// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mesh_msgs:msg/MeshVertexColors.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_vertex_colors.h"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COLORS__STRUCT_H_
#define MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COLORS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'vertex_colors'
#include "std_msgs/msg/detail/color_rgba__struct.h"

/// Struct defined in msg/MeshVertexColors in the package mesh_msgs.
/**
  * Mesh Attribute Message
 */
typedef struct mesh_msgs__msg__MeshVertexColors
{
  std_msgs__msg__ColorRGBA__Sequence vertex_colors;
} mesh_msgs__msg__MeshVertexColors;

// Struct for a sequence of mesh_msgs__msg__MeshVertexColors.
typedef struct mesh_msgs__msg__MeshVertexColors__Sequence
{
  mesh_msgs__msg__MeshVertexColors * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__msg__MeshVertexColors__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COLORS__STRUCT_H_
