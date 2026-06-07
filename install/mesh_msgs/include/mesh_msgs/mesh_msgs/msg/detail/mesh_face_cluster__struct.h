// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mesh_msgs:msg/MeshFaceCluster.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_face_cluster.h"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_FACE_CLUSTER__STRUCT_H_
#define MESH_MSGS__MSG__DETAIL__MESH_FACE_CLUSTER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'face_indices'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'label'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/MeshFaceCluster in the package mesh_msgs.
/**
  * Cluster
 */
typedef struct mesh_msgs__msg__MeshFaceCluster
{
  rosidl_runtime_c__uint32__Sequence face_indices;
  /// optional
  rosidl_runtime_c__String label;
} mesh_msgs__msg__MeshFaceCluster;

// Struct for a sequence of mesh_msgs__msg__MeshFaceCluster.
typedef struct mesh_msgs__msg__MeshFaceCluster__Sequence
{
  mesh_msgs__msg__MeshFaceCluster * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__msg__MeshFaceCluster__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESH_MSGS__MSG__DETAIL__MESH_FACE_CLUSTER__STRUCT_H_
