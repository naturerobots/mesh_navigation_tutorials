// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mesh_msgs:msg/MeshMaterials.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_materials.h"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_MATERIALS__STRUCT_H_
#define MESH_MSGS__MSG__DETAIL__MESH_MATERIALS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'clusters'
#include "mesh_msgs/msg/detail/mesh_face_cluster__struct.h"
// Member 'materials'
#include "mesh_msgs/msg/detail/mesh_material__struct.h"
// Member 'cluster_materials'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'vertex_tex_coords'
#include "mesh_msgs/msg/detail/mesh_vertex_tex_coords__struct.h"

/// Struct defined in msg/MeshMaterials in the package mesh_msgs.
/**
  * Mesh Attribute Message
 */
typedef struct mesh_msgs__msg__MeshMaterials
{
  mesh_msgs__msg__MeshFaceCluster__Sequence clusters;
  mesh_msgs__msg__MeshMaterial__Sequence materials;
  rosidl_runtime_c__uint32__Sequence cluster_materials;
  mesh_msgs__msg__MeshVertexTexCoords__Sequence vertex_tex_coords;
} mesh_msgs__msg__MeshMaterials;

// Struct for a sequence of mesh_msgs__msg__MeshMaterials.
typedef struct mesh_msgs__msg__MeshMaterials__Sequence
{
  mesh_msgs__msg__MeshMaterials * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__msg__MeshMaterials__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESH_MSGS__MSG__DETAIL__MESH_MATERIALS__STRUCT_H_
