// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mesh_msgs:msg/MeshVertexTexCoords.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_vertex_tex_coords.h"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_VERTEX_TEX_COORDS__STRUCT_H_
#define MESH_MSGS__MSG__DETAIL__MESH_VERTEX_TEX_COORDS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/MeshVertexTexCoords in the package mesh_msgs.
/**
  * Mesh Attribute Type
 */
typedef struct mesh_msgs__msg__MeshVertexTexCoords
{
  float u;
  float v;
} mesh_msgs__msg__MeshVertexTexCoords;

// Struct for a sequence of mesh_msgs__msg__MeshVertexTexCoords.
typedef struct mesh_msgs__msg__MeshVertexTexCoords__Sequence
{
  mesh_msgs__msg__MeshVertexTexCoords * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__msg__MeshVertexTexCoords__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESH_MSGS__MSG__DETAIL__MESH_VERTEX_TEX_COORDS__STRUCT_H_
