// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mesh_msgs:msg/VectorFieldStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/vector_field_stamped.h"


#ifndef MESH_MSGS__MSG__DETAIL__VECTOR_FIELD_STAMPED__STRUCT_H_
#define MESH_MSGS__MSG__DETAIL__VECTOR_FIELD_STAMPED__STRUCT_H_

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
// Member 'vector_field'
#include "mesh_msgs/msg/detail/vector_field__struct.h"

/// Struct defined in msg/VectorFieldStamped in the package mesh_msgs.
typedef struct mesh_msgs__msg__VectorFieldStamped
{
  std_msgs__msg__Header header;
  mesh_msgs__msg__VectorField vector_field;
} mesh_msgs__msg__VectorFieldStamped;

// Struct for a sequence of mesh_msgs__msg__VectorFieldStamped.
typedef struct mesh_msgs__msg__VectorFieldStamped__Sequence
{
  mesh_msgs__msg__VectorFieldStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__msg__VectorFieldStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESH_MSGS__MSG__DETAIL__VECTOR_FIELD_STAMPED__STRUCT_H_
