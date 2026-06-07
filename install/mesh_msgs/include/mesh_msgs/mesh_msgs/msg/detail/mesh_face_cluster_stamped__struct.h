// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mesh_msgs:msg/MeshFaceClusterStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_face_cluster_stamped.h"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_FACE_CLUSTER_STAMPED__STRUCT_H_
#define MESH_MSGS__MSG__DETAIL__MESH_FACE_CLUSTER_STAMPED__STRUCT_H_

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
// Member 'cluster'
#include "mesh_msgs/msg/detail/mesh_face_cluster__struct.h"

/// Struct defined in msg/MeshFaceClusterStamped in the package mesh_msgs.
/**
  * header
 */
typedef struct mesh_msgs__msg__MeshFaceClusterStamped
{
  std_msgs__msg__Header header;
  /// mesh uuid
  rosidl_runtime_c__String uuid;
  /// Cluster
  mesh_msgs__msg__MeshFaceCluster cluster;
  /// overwrite existing labeled faces
  bool override;
} mesh_msgs__msg__MeshFaceClusterStamped;

// Struct for a sequence of mesh_msgs__msg__MeshFaceClusterStamped.
typedef struct mesh_msgs__msg__MeshFaceClusterStamped__Sequence
{
  mesh_msgs__msg__MeshFaceClusterStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__msg__MeshFaceClusterStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESH_MSGS__MSG__DETAIL__MESH_FACE_CLUSTER_STAMPED__STRUCT_H_
