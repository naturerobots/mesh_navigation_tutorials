// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mesh_msgs:msg/MeshGeometryStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_geometry_stamped.h"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_GEOMETRY_STAMPED__STRUCT_H_
#define MESH_MSGS__MSG__DETAIL__MESH_GEOMETRY_STAMPED__STRUCT_H_

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
#include "rosidl_runtime_c/string.h"
// Member 'mesh_geometry'
#include "mesh_msgs/msg/detail/mesh_geometry__struct.h"

/// Struct defined in msg/MeshGeometryStamped in the package mesh_msgs.
/**
  * Mesh Geometry Message
 */
typedef struct mesh_msgs__msg__MeshGeometryStamped
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String uuid;
  mesh_msgs__msg__MeshGeometry mesh_geometry;
} mesh_msgs__msg__MeshGeometryStamped;

// Struct for a sequence of mesh_msgs__msg__MeshGeometryStamped.
typedef struct mesh_msgs__msg__MeshGeometryStamped__Sequence
{
  mesh_msgs__msg__MeshGeometryStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__msg__MeshGeometryStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESH_MSGS__MSG__DETAIL__MESH_GEOMETRY_STAMPED__STRUCT_H_
