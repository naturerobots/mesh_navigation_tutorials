// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mesh_msgs:msg/MeshGeometry.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mesh_msgs/msg/detail/mesh_geometry__rosidl_typesupport_introspection_c.h"
#include "mesh_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mesh_msgs/msg/detail/mesh_geometry__functions.h"
#include "mesh_msgs/msg/detail/mesh_geometry__struct.h"


// Include directives for member types
// Member `vertices`
// Member `vertex_normals`
#include "geometry_msgs/msg/point.h"
// Member `vertices`
// Member `vertex_normals`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"
// Member `faces`
#include "mesh_msgs/msg/mesh_triangle_indices.h"
// Member `faces`
#include "mesh_msgs/msg/detail/mesh_triangle_indices__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__MeshGeometry_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mesh_msgs__msg__MeshGeometry__init(message_memory);
}

void mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__MeshGeometry_fini_function(void * message_memory)
{
  mesh_msgs__msg__MeshGeometry__fini(message_memory);
}

size_t mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__size_function__MeshGeometry__vertices(
  const void * untyped_member)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__get_const_function__MeshGeometry__vertices(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__get_function__MeshGeometry__vertices(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__fetch_function__MeshGeometry__vertices(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Point * item =
    ((const geometry_msgs__msg__Point *)
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__get_const_function__MeshGeometry__vertices(untyped_member, index));
  geometry_msgs__msg__Point * value =
    (geometry_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__assign_function__MeshGeometry__vertices(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Point * item =
    ((geometry_msgs__msg__Point *)
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__get_function__MeshGeometry__vertices(untyped_member, index));
  const geometry_msgs__msg__Point * value =
    (const geometry_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__resize_function__MeshGeometry__vertices(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  geometry_msgs__msg__Point__Sequence__fini(member);
  return geometry_msgs__msg__Point__Sequence__init(member, size);
}

size_t mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__size_function__MeshGeometry__vertex_normals(
  const void * untyped_member)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__get_const_function__MeshGeometry__vertex_normals(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__get_function__MeshGeometry__vertex_normals(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__fetch_function__MeshGeometry__vertex_normals(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Point * item =
    ((const geometry_msgs__msg__Point *)
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__get_const_function__MeshGeometry__vertex_normals(untyped_member, index));
  geometry_msgs__msg__Point * value =
    (geometry_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__assign_function__MeshGeometry__vertex_normals(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Point * item =
    ((geometry_msgs__msg__Point *)
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__get_function__MeshGeometry__vertex_normals(untyped_member, index));
  const geometry_msgs__msg__Point * value =
    (const geometry_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__resize_function__MeshGeometry__vertex_normals(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  geometry_msgs__msg__Point__Sequence__fini(member);
  return geometry_msgs__msg__Point__Sequence__init(member, size);
}

size_t mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__size_function__MeshGeometry__faces(
  const void * untyped_member)
{
  const mesh_msgs__msg__MeshTriangleIndices__Sequence * member =
    (const mesh_msgs__msg__MeshTriangleIndices__Sequence *)(untyped_member);
  return member->size;
}

const void * mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__get_const_function__MeshGeometry__faces(
  const void * untyped_member, size_t index)
{
  const mesh_msgs__msg__MeshTriangleIndices__Sequence * member =
    (const mesh_msgs__msg__MeshTriangleIndices__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__get_function__MeshGeometry__faces(
  void * untyped_member, size_t index)
{
  mesh_msgs__msg__MeshTriangleIndices__Sequence * member =
    (mesh_msgs__msg__MeshTriangleIndices__Sequence *)(untyped_member);
  return &member->data[index];
}

void mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__fetch_function__MeshGeometry__faces(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const mesh_msgs__msg__MeshTriangleIndices * item =
    ((const mesh_msgs__msg__MeshTriangleIndices *)
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__get_const_function__MeshGeometry__faces(untyped_member, index));
  mesh_msgs__msg__MeshTriangleIndices * value =
    (mesh_msgs__msg__MeshTriangleIndices *)(untyped_value);
  *value = *item;
}

void mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__assign_function__MeshGeometry__faces(
  void * untyped_member, size_t index, const void * untyped_value)
{
  mesh_msgs__msg__MeshTriangleIndices * item =
    ((mesh_msgs__msg__MeshTriangleIndices *)
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__get_function__MeshGeometry__faces(untyped_member, index));
  const mesh_msgs__msg__MeshTriangleIndices * value =
    (const mesh_msgs__msg__MeshTriangleIndices *)(untyped_value);
  *item = *value;
}

bool mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__resize_function__MeshGeometry__faces(
  void * untyped_member, size_t size)
{
  mesh_msgs__msg__MeshTriangleIndices__Sequence * member =
    (mesh_msgs__msg__MeshTriangleIndices__Sequence *)(untyped_member);
  mesh_msgs__msg__MeshTriangleIndices__Sequence__fini(member);
  return mesh_msgs__msg__MeshTriangleIndices__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__MeshGeometry_message_member_array[3] = {
  {
    "vertices",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs__msg__MeshGeometry, vertices),  // bytes offset in struct
    NULL,  // default value
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__size_function__MeshGeometry__vertices,  // size() function pointer
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__get_const_function__MeshGeometry__vertices,  // get_const(index) function pointer
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__get_function__MeshGeometry__vertices,  // get(index) function pointer
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__fetch_function__MeshGeometry__vertices,  // fetch(index, &value) function pointer
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__assign_function__MeshGeometry__vertices,  // assign(index, value) function pointer
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__resize_function__MeshGeometry__vertices  // resize(index) function pointer
  },
  {
    "vertex_normals",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs__msg__MeshGeometry, vertex_normals),  // bytes offset in struct
    NULL,  // default value
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__size_function__MeshGeometry__vertex_normals,  // size() function pointer
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__get_const_function__MeshGeometry__vertex_normals,  // get_const(index) function pointer
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__get_function__MeshGeometry__vertex_normals,  // get(index) function pointer
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__fetch_function__MeshGeometry__vertex_normals,  // fetch(index, &value) function pointer
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__assign_function__MeshGeometry__vertex_normals,  // assign(index, value) function pointer
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__resize_function__MeshGeometry__vertex_normals  // resize(index) function pointer
  },
  {
    "faces",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs__msg__MeshGeometry, faces),  // bytes offset in struct
    NULL,  // default value
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__size_function__MeshGeometry__faces,  // size() function pointer
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__get_const_function__MeshGeometry__faces,  // get_const(index) function pointer
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__get_function__MeshGeometry__faces,  // get(index) function pointer
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__fetch_function__MeshGeometry__faces,  // fetch(index, &value) function pointer
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__assign_function__MeshGeometry__faces,  // assign(index, value) function pointer
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__resize_function__MeshGeometry__faces  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__MeshGeometry_message_members = {
  "mesh_msgs__msg",  // message namespace
  "MeshGeometry",  // message name
  3,  // number of fields
  sizeof(mesh_msgs__msg__MeshGeometry),
  false,  // has_any_key_member_
  mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__MeshGeometry_message_member_array,  // message members
  mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__MeshGeometry_init_function,  // function to initialize message memory (memory has to be allocated)
  mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__MeshGeometry_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__MeshGeometry_message_type_support_handle = {
  0,
  &mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__MeshGeometry_message_members,
  get_message_typesupport_handle_function,
  &mesh_msgs__msg__MeshGeometry__get_type_hash,
  &mesh_msgs__msg__MeshGeometry__get_type_description,
  &mesh_msgs__msg__MeshGeometry__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mesh_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, msg, MeshGeometry)() {
  mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__MeshGeometry_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__MeshGeometry_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__MeshGeometry_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, msg, MeshTriangleIndices)();
  if (!mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__MeshGeometry_message_type_support_handle.typesupport_identifier) {
    mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__MeshGeometry_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mesh_msgs__msg__MeshGeometry__rosidl_typesupport_introspection_c__MeshGeometry_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
