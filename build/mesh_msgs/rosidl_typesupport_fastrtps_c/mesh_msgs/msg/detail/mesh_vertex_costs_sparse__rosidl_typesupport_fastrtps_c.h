// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from mesh_msgs:msg/MeshVertexCostsSparse.idl
// generated code does not contain a copyright notice
#ifndef MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS_SPARSE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS_SPARSE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "mesh_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "mesh_msgs/msg/detail/mesh_vertex_costs_sparse__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mesh_msgs
bool cdr_serialize_mesh_msgs__msg__MeshVertexCostsSparse(
  const mesh_msgs__msg__MeshVertexCostsSparse * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mesh_msgs
bool cdr_deserialize_mesh_msgs__msg__MeshVertexCostsSparse(
  eprosima::fastcdr::Cdr &,
  mesh_msgs__msg__MeshVertexCostsSparse * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mesh_msgs
size_t get_serialized_size_mesh_msgs__msg__MeshVertexCostsSparse(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mesh_msgs
size_t max_serialized_size_mesh_msgs__msg__MeshVertexCostsSparse(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mesh_msgs
bool cdr_serialize_key_mesh_msgs__msg__MeshVertexCostsSparse(
  const mesh_msgs__msg__MeshVertexCostsSparse * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mesh_msgs
size_t get_serialized_size_key_mesh_msgs__msg__MeshVertexCostsSparse(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mesh_msgs
size_t max_serialized_size_key_mesh_msgs__msg__MeshVertexCostsSparse(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mesh_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mesh_msgs, msg, MeshVertexCostsSparse)();

#ifdef __cplusplus
}
#endif

#endif  // MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS_SPARSE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
