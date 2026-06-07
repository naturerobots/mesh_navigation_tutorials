// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mesh_msgs:msg/MeshTriangleIndices.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mesh_msgs/msg/detail/mesh_triangle_indices__rosidl_typesupport_introspection_c.h"
#include "mesh_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mesh_msgs/msg/detail/mesh_triangle_indices__functions.h"
#include "mesh_msgs/msg/detail/mesh_triangle_indices__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__MeshTriangleIndices_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mesh_msgs__msg__MeshTriangleIndices__init(message_memory);
}

void mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__MeshTriangleIndices_fini_function(void * message_memory)
{
  mesh_msgs__msg__MeshTriangleIndices__fini(message_memory);
}

size_t mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__size_function__MeshTriangleIndices__vertex_indices(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__get_const_function__MeshTriangleIndices__vertex_indices(
  const void * untyped_member, size_t index)
{
  const uint32_t * member =
    (const uint32_t *)(untyped_member);
  return &member[index];
}

void * mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__get_function__MeshTriangleIndices__vertex_indices(
  void * untyped_member, size_t index)
{
  uint32_t * member =
    (uint32_t *)(untyped_member);
  return &member[index];
}

void mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__fetch_function__MeshTriangleIndices__vertex_indices(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint32_t * item =
    ((const uint32_t *)
    mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__get_const_function__MeshTriangleIndices__vertex_indices(untyped_member, index));
  uint32_t * value =
    (uint32_t *)(untyped_value);
  *value = *item;
}

void mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__assign_function__MeshTriangleIndices__vertex_indices(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint32_t * item =
    ((uint32_t *)
    mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__get_function__MeshTriangleIndices__vertex_indices(untyped_member, index));
  const uint32_t * value =
    (const uint32_t *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__MeshTriangleIndices_message_member_array[1] = {
  {
    "vertex_indices",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs__msg__MeshTriangleIndices, vertex_indices),  // bytes offset in struct
    NULL,  // default value
    mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__size_function__MeshTriangleIndices__vertex_indices,  // size() function pointer
    mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__get_const_function__MeshTriangleIndices__vertex_indices,  // get_const(index) function pointer
    mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__get_function__MeshTriangleIndices__vertex_indices,  // get(index) function pointer
    mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__fetch_function__MeshTriangleIndices__vertex_indices,  // fetch(index, &value) function pointer
    mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__assign_function__MeshTriangleIndices__vertex_indices,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__MeshTriangleIndices_message_members = {
  "mesh_msgs__msg",  // message namespace
  "MeshTriangleIndices",  // message name
  1,  // number of fields
  sizeof(mesh_msgs__msg__MeshTriangleIndices),
  false,  // has_any_key_member_
  mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__MeshTriangleIndices_message_member_array,  // message members
  mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__MeshTriangleIndices_init_function,  // function to initialize message memory (memory has to be allocated)
  mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__MeshTriangleIndices_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__MeshTriangleIndices_message_type_support_handle = {
  0,
  &mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__MeshTriangleIndices_message_members,
  get_message_typesupport_handle_function,
  &mesh_msgs__msg__MeshTriangleIndices__get_type_hash,
  &mesh_msgs__msg__MeshTriangleIndices__get_type_description,
  &mesh_msgs__msg__MeshTriangleIndices__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mesh_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, msg, MeshTriangleIndices)() {
  if (!mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__MeshTriangleIndices_message_type_support_handle.typesupport_identifier) {
    mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__MeshTriangleIndices_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mesh_msgs__msg__MeshTriangleIndices__rosidl_typesupport_introspection_c__MeshTriangleIndices_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
