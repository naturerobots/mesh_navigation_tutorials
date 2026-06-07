// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mesh_msgs:msg/MeshGeometry.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_geometry.h"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_GEOMETRY__STRUCT_H_
#define MESH_MSGS__MSG__DETAIL__MESH_GEOMETRY__STRUCT_H_

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
// Member 'vertex_normals'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'faces'
#include "mesh_msgs/msg/detail/mesh_triangle_indices__struct.h"

/// Struct defined in msg/MeshGeometry in the package mesh_msgs.
/**
  * Mesh Geometry Message
 */
typedef struct mesh_msgs__msg__MeshGeometry
{
  geometry_msgs__msg__Point__Sequence vertices;
  geometry_msgs__msg__Point__Sequence vertex_normals;
  mesh_msgs__msg__MeshTriangleIndices__Sequence faces;
} mesh_msgs__msg__MeshGeometry;

// Struct for a sequence of mesh_msgs__msg__MeshGeometry.
typedef struct mesh_msgs__msg__MeshGeometry__Sequence
{
  mesh_msgs__msg__MeshGeometry * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__msg__MeshGeometry__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESH_MSGS__MSG__DETAIL__MESH_GEOMETRY__STRUCT_H_
