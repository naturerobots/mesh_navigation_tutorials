// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mesh_msgs:msg/VectorField.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/vector_field.h"


#ifndef MESH_MSGS__MSG__DETAIL__VECTOR_FIELD__STRUCT_H_
#define MESH_MSGS__MSG__DETAIL__VECTOR_FIELD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'positions'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'vectors'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/VectorField in the package mesh_msgs.
typedef struct mesh_msgs__msg__VectorField
{
  geometry_msgs__msg__Point__Sequence positions;
  geometry_msgs__msg__Vector3__Sequence vectors;
} mesh_msgs__msg__VectorField;

// Struct for a sequence of mesh_msgs__msg__VectorField.
typedef struct mesh_msgs__msg__VectorField__Sequence
{
  mesh_msgs__msg__VectorField * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__msg__VectorField__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESH_MSGS__MSG__DETAIL__VECTOR_FIELD__STRUCT_H_
