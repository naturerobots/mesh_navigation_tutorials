// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mesh_msgs:msg/MeshTexture.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_texture.h"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_TEXTURE__STRUCT_H_
#define MESH_MSGS__MSG__DETAIL__MESH_TEXTURE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'uuid'
#include "rosidl_runtime_c/string.h"
// Member 'image'
#include "sensor_msgs/msg/detail/image__struct.h"

/// Struct defined in msg/MeshTexture in the package mesh_msgs.
/**
  * Mesh Attribute Message
 */
typedef struct mesh_msgs__msg__MeshTexture
{
  rosidl_runtime_c__String uuid;
  uint32_t texture_index;
  sensor_msgs__msg__Image image;
} mesh_msgs__msg__MeshTexture;

// Struct for a sequence of mesh_msgs__msg__MeshTexture.
typedef struct mesh_msgs__msg__MeshTexture__Sequence
{
  mesh_msgs__msg__MeshTexture * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__msg__MeshTexture__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESH_MSGS__MSG__DETAIL__MESH_TEXTURE__STRUCT_H_
