// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mesh_msgs:msg/MeshFaceCluster.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mesh_msgs/msg/detail/mesh_face_cluster__rosidl_typesupport_introspection_c.h"
#include "mesh_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mesh_msgs/msg/detail/mesh_face_cluster__functions.h"
#include "mesh_msgs/msg/detail/mesh_face_cluster__struct.h"


// Include directives for member types
// Member `face_indices`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `label`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__MeshFaceCluster_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mesh_msgs__msg__MeshFaceCluster__init(message_memory);
}

void mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__MeshFaceCluster_fini_function(void * message_memory)
{
  mesh_msgs__msg__MeshFaceCluster__fini(message_memory);
}

size_t mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__size_function__MeshFaceCluster__face_indices(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return member->size;
}

const void * mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__get_const_function__MeshFaceCluster__face_indices(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__get_function__MeshFaceCluster__face_indices(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__fetch_function__MeshFaceCluster__face_indices(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint32_t * item =
    ((const uint32_t *)
    mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__get_const_function__MeshFaceCluster__face_indices(untyped_member, index));
  uint32_t * value =
    (uint32_t *)(untyped_value);
  *value = *item;
}

void mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__assign_function__MeshFaceCluster__face_indices(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint32_t * item =
    ((uint32_t *)
    mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__get_function__MeshFaceCluster__face_indices(untyped_member, index));
  const uint32_t * value =
    (const uint32_t *)(untyped_value);
  *item = *value;
}

bool mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__resize_function__MeshFaceCluster__face_indices(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  rosidl_runtime_c__uint32__Sequence__fini(member);
  return rosidl_runtime_c__uint32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__MeshFaceCluster_message_member_array[2] = {
  {
    "face_indices",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs__msg__MeshFaceCluster, face_indices),  // bytes offset in struct
    NULL,  // default value
    mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__size_function__MeshFaceCluster__face_indices,  // size() function pointer
    mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__get_const_function__MeshFaceCluster__face_indices,  // get_const(index) function pointer
    mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__get_function__MeshFaceCluster__face_indices,  // get(index) function pointer
    mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__fetch_function__MeshFaceCluster__face_indices,  // fetch(index, &value) function pointer
    mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__assign_function__MeshFaceCluster__face_indices,  // assign(index, value) function pointer
    mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__resize_function__MeshFaceCluster__face_indices  // resize(index) function pointer
  },
  {
    "label",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs__msg__MeshFaceCluster, label),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__MeshFaceCluster_message_members = {
  "mesh_msgs__msg",  // message namespace
  "MeshFaceCluster",  // message name
  2,  // number of fields
  sizeof(mesh_msgs__msg__MeshFaceCluster),
  false,  // has_any_key_member_
  mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__MeshFaceCluster_message_member_array,  // message members
  mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__MeshFaceCluster_init_function,  // function to initialize message memory (memory has to be allocated)
  mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__MeshFaceCluster_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__MeshFaceCluster_message_type_support_handle = {
  0,
  &mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__MeshFaceCluster_message_members,
  get_message_typesupport_handle_function,
  &mesh_msgs__msg__MeshFaceCluster__get_type_hash,
  &mesh_msgs__msg__MeshFaceCluster__get_type_description,
  &mesh_msgs__msg__MeshFaceCluster__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mesh_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, msg, MeshFaceCluster)() {
  if (!mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__MeshFaceCluster_message_type_support_handle.typesupport_identifier) {
    mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__MeshFaceCluster_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mesh_msgs__msg__MeshFaceCluster__rosidl_typesupport_introspection_c__MeshFaceCluster_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
