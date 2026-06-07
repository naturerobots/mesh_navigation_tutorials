// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mesh_msgs:msg/MeshMaterials.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mesh_msgs/msg/detail/mesh_materials__rosidl_typesupport_introspection_c.h"
#include "mesh_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mesh_msgs/msg/detail/mesh_materials__functions.h"
#include "mesh_msgs/msg/detail/mesh_materials__struct.h"


// Include directives for member types
// Member `clusters`
#include "mesh_msgs/msg/mesh_face_cluster.h"
// Member `clusters`
#include "mesh_msgs/msg/detail/mesh_face_cluster__rosidl_typesupport_introspection_c.h"
// Member `materials`
#include "mesh_msgs/msg/mesh_material.h"
// Member `materials`
#include "mesh_msgs/msg/detail/mesh_material__rosidl_typesupport_introspection_c.h"
// Member `cluster_materials`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `vertex_tex_coords`
#include "mesh_msgs/msg/mesh_vertex_tex_coords.h"
// Member `vertex_tex_coords`
#include "mesh_msgs/msg/detail/mesh_vertex_tex_coords__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__MeshMaterials_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mesh_msgs__msg__MeshMaterials__init(message_memory);
}

void mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__MeshMaterials_fini_function(void * message_memory)
{
  mesh_msgs__msg__MeshMaterials__fini(message_memory);
}

size_t mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__size_function__MeshMaterials__clusters(
  const void * untyped_member)
{
  const mesh_msgs__msg__MeshFaceCluster__Sequence * member =
    (const mesh_msgs__msg__MeshFaceCluster__Sequence *)(untyped_member);
  return member->size;
}

const void * mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_const_function__MeshMaterials__clusters(
  const void * untyped_member, size_t index)
{
  const mesh_msgs__msg__MeshFaceCluster__Sequence * member =
    (const mesh_msgs__msg__MeshFaceCluster__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_function__MeshMaterials__clusters(
  void * untyped_member, size_t index)
{
  mesh_msgs__msg__MeshFaceCluster__Sequence * member =
    (mesh_msgs__msg__MeshFaceCluster__Sequence *)(untyped_member);
  return &member->data[index];
}

void mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__fetch_function__MeshMaterials__clusters(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const mesh_msgs__msg__MeshFaceCluster * item =
    ((const mesh_msgs__msg__MeshFaceCluster *)
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_const_function__MeshMaterials__clusters(untyped_member, index));
  mesh_msgs__msg__MeshFaceCluster * value =
    (mesh_msgs__msg__MeshFaceCluster *)(untyped_value);
  *value = *item;
}

void mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__assign_function__MeshMaterials__clusters(
  void * untyped_member, size_t index, const void * untyped_value)
{
  mesh_msgs__msg__MeshFaceCluster * item =
    ((mesh_msgs__msg__MeshFaceCluster *)
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_function__MeshMaterials__clusters(untyped_member, index));
  const mesh_msgs__msg__MeshFaceCluster * value =
    (const mesh_msgs__msg__MeshFaceCluster *)(untyped_value);
  *item = *value;
}

bool mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__resize_function__MeshMaterials__clusters(
  void * untyped_member, size_t size)
{
  mesh_msgs__msg__MeshFaceCluster__Sequence * member =
    (mesh_msgs__msg__MeshFaceCluster__Sequence *)(untyped_member);
  mesh_msgs__msg__MeshFaceCluster__Sequence__fini(member);
  return mesh_msgs__msg__MeshFaceCluster__Sequence__init(member, size);
}

size_t mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__size_function__MeshMaterials__materials(
  const void * untyped_member)
{
  const mesh_msgs__msg__MeshMaterial__Sequence * member =
    (const mesh_msgs__msg__MeshMaterial__Sequence *)(untyped_member);
  return member->size;
}

const void * mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_const_function__MeshMaterials__materials(
  const void * untyped_member, size_t index)
{
  const mesh_msgs__msg__MeshMaterial__Sequence * member =
    (const mesh_msgs__msg__MeshMaterial__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_function__MeshMaterials__materials(
  void * untyped_member, size_t index)
{
  mesh_msgs__msg__MeshMaterial__Sequence * member =
    (mesh_msgs__msg__MeshMaterial__Sequence *)(untyped_member);
  return &member->data[index];
}

void mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__fetch_function__MeshMaterials__materials(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const mesh_msgs__msg__MeshMaterial * item =
    ((const mesh_msgs__msg__MeshMaterial *)
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_const_function__MeshMaterials__materials(untyped_member, index));
  mesh_msgs__msg__MeshMaterial * value =
    (mesh_msgs__msg__MeshMaterial *)(untyped_value);
  *value = *item;
}

void mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__assign_function__MeshMaterials__materials(
  void * untyped_member, size_t index, const void * untyped_value)
{
  mesh_msgs__msg__MeshMaterial * item =
    ((mesh_msgs__msg__MeshMaterial *)
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_function__MeshMaterials__materials(untyped_member, index));
  const mesh_msgs__msg__MeshMaterial * value =
    (const mesh_msgs__msg__MeshMaterial *)(untyped_value);
  *item = *value;
}

bool mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__resize_function__MeshMaterials__materials(
  void * untyped_member, size_t size)
{
  mesh_msgs__msg__MeshMaterial__Sequence * member =
    (mesh_msgs__msg__MeshMaterial__Sequence *)(untyped_member);
  mesh_msgs__msg__MeshMaterial__Sequence__fini(member);
  return mesh_msgs__msg__MeshMaterial__Sequence__init(member, size);
}

size_t mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__size_function__MeshMaterials__cluster_materials(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return member->size;
}

const void * mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_const_function__MeshMaterials__cluster_materials(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_function__MeshMaterials__cluster_materials(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__fetch_function__MeshMaterials__cluster_materials(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint32_t * item =
    ((const uint32_t *)
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_const_function__MeshMaterials__cluster_materials(untyped_member, index));
  uint32_t * value =
    (uint32_t *)(untyped_value);
  *value = *item;
}

void mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__assign_function__MeshMaterials__cluster_materials(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint32_t * item =
    ((uint32_t *)
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_function__MeshMaterials__cluster_materials(untyped_member, index));
  const uint32_t * value =
    (const uint32_t *)(untyped_value);
  *item = *value;
}

bool mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__resize_function__MeshMaterials__cluster_materials(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  rosidl_runtime_c__uint32__Sequence__fini(member);
  return rosidl_runtime_c__uint32__Sequence__init(member, size);
}

size_t mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__size_function__MeshMaterials__vertex_tex_coords(
  const void * untyped_member)
{
  const mesh_msgs__msg__MeshVertexTexCoords__Sequence * member =
    (const mesh_msgs__msg__MeshVertexTexCoords__Sequence *)(untyped_member);
  return member->size;
}

const void * mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_const_function__MeshMaterials__vertex_tex_coords(
  const void * untyped_member, size_t index)
{
  const mesh_msgs__msg__MeshVertexTexCoords__Sequence * member =
    (const mesh_msgs__msg__MeshVertexTexCoords__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_function__MeshMaterials__vertex_tex_coords(
  void * untyped_member, size_t index)
{
  mesh_msgs__msg__MeshVertexTexCoords__Sequence * member =
    (mesh_msgs__msg__MeshVertexTexCoords__Sequence *)(untyped_member);
  return &member->data[index];
}

void mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__fetch_function__MeshMaterials__vertex_tex_coords(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const mesh_msgs__msg__MeshVertexTexCoords * item =
    ((const mesh_msgs__msg__MeshVertexTexCoords *)
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_const_function__MeshMaterials__vertex_tex_coords(untyped_member, index));
  mesh_msgs__msg__MeshVertexTexCoords * value =
    (mesh_msgs__msg__MeshVertexTexCoords *)(untyped_value);
  *value = *item;
}

void mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__assign_function__MeshMaterials__vertex_tex_coords(
  void * untyped_member, size_t index, const void * untyped_value)
{
  mesh_msgs__msg__MeshVertexTexCoords * item =
    ((mesh_msgs__msg__MeshVertexTexCoords *)
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_function__MeshMaterials__vertex_tex_coords(untyped_member, index));
  const mesh_msgs__msg__MeshVertexTexCoords * value =
    (const mesh_msgs__msg__MeshVertexTexCoords *)(untyped_value);
  *item = *value;
}

bool mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__resize_function__MeshMaterials__vertex_tex_coords(
  void * untyped_member, size_t size)
{
  mesh_msgs__msg__MeshVertexTexCoords__Sequence * member =
    (mesh_msgs__msg__MeshVertexTexCoords__Sequence *)(untyped_member);
  mesh_msgs__msg__MeshVertexTexCoords__Sequence__fini(member);
  return mesh_msgs__msg__MeshVertexTexCoords__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__MeshMaterials_message_member_array[4] = {
  {
    "clusters",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs__msg__MeshMaterials, clusters),  // bytes offset in struct
    NULL,  // default value
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__size_function__MeshMaterials__clusters,  // size() function pointer
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_const_function__MeshMaterials__clusters,  // get_const(index) function pointer
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_function__MeshMaterials__clusters,  // get(index) function pointer
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__fetch_function__MeshMaterials__clusters,  // fetch(index, &value) function pointer
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__assign_function__MeshMaterials__clusters,  // assign(index, value) function pointer
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__resize_function__MeshMaterials__clusters  // resize(index) function pointer
  },
  {
    "materials",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs__msg__MeshMaterials, materials),  // bytes offset in struct
    NULL,  // default value
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__size_function__MeshMaterials__materials,  // size() function pointer
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_const_function__MeshMaterials__materials,  // get_const(index) function pointer
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_function__MeshMaterials__materials,  // get(index) function pointer
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__fetch_function__MeshMaterials__materials,  // fetch(index, &value) function pointer
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__assign_function__MeshMaterials__materials,  // assign(index, value) function pointer
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__resize_function__MeshMaterials__materials  // resize(index) function pointer
  },
  {
    "cluster_materials",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs__msg__MeshMaterials, cluster_materials),  // bytes offset in struct
    NULL,  // default value
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__size_function__MeshMaterials__cluster_materials,  // size() function pointer
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_const_function__MeshMaterials__cluster_materials,  // get_const(index) function pointer
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_function__MeshMaterials__cluster_materials,  // get(index) function pointer
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__fetch_function__MeshMaterials__cluster_materials,  // fetch(index, &value) function pointer
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__assign_function__MeshMaterials__cluster_materials,  // assign(index, value) function pointer
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__resize_function__MeshMaterials__cluster_materials  // resize(index) function pointer
  },
  {
    "vertex_tex_coords",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs__msg__MeshMaterials, vertex_tex_coords),  // bytes offset in struct
    NULL,  // default value
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__size_function__MeshMaterials__vertex_tex_coords,  // size() function pointer
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_const_function__MeshMaterials__vertex_tex_coords,  // get_const(index) function pointer
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__get_function__MeshMaterials__vertex_tex_coords,  // get(index) function pointer
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__fetch_function__MeshMaterials__vertex_tex_coords,  // fetch(index, &value) function pointer
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__assign_function__MeshMaterials__vertex_tex_coords,  // assign(index, value) function pointer
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__resize_function__MeshMaterials__vertex_tex_coords  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__MeshMaterials_message_members = {
  "mesh_msgs__msg",  // message namespace
  "MeshMaterials",  // message name
  4,  // number of fields
  sizeof(mesh_msgs__msg__MeshMaterials),
  false,  // has_any_key_member_
  mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__MeshMaterials_message_member_array,  // message members
  mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__MeshMaterials_init_function,  // function to initialize message memory (memory has to be allocated)
  mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__MeshMaterials_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__MeshMaterials_message_type_support_handle = {
  0,
  &mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__MeshMaterials_message_members,
  get_message_typesupport_handle_function,
  &mesh_msgs__msg__MeshMaterials__get_type_hash,
  &mesh_msgs__msg__MeshMaterials__get_type_description,
  &mesh_msgs__msg__MeshMaterials__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mesh_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, msg, MeshMaterials)() {
  mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__MeshMaterials_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, msg, MeshFaceCluster)();
  mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__MeshMaterials_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, msg, MeshMaterial)();
  mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__MeshMaterials_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, msg, MeshVertexTexCoords)();
  if (!mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__MeshMaterials_message_type_support_handle.typesupport_identifier) {
    mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__MeshMaterials_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mesh_msgs__msg__MeshMaterials__rosidl_typesupport_introspection_c__MeshMaterials_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
