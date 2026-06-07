// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mesh_msgs:msg/MeshMaterial.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_material.h"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_MATERIAL__STRUCT_H_
#define MESH_MSGS__MSG__DETAIL__MESH_MATERIAL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'color'
#include "std_msgs/msg/detail/color_rgba__struct.h"

/// Struct defined in msg/MeshMaterial in the package mesh_msgs.
typedef struct mesh_msgs__msg__MeshMaterial
{
  uint32_t texture_index;
  std_msgs__msg__ColorRGBA color;
  bool has_texture;
} mesh_msgs__msg__MeshMaterial;

// Struct for a sequence of mesh_msgs__msg__MeshMaterial.
typedef struct mesh_msgs__msg__MeshMaterial__Sequence
{
  mesh_msgs__msg__MeshMaterial * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__msg__MeshMaterial__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESH_MSGS__MSG__DETAIL__MESH_MATERIAL__STRUCT_H_
