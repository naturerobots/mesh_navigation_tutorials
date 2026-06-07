// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mesh_msgs:msg/VectorField.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mesh_msgs/msg/detail/vector_field__rosidl_typesupport_introspection_c.h"
#include "mesh_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mesh_msgs/msg/detail/vector_field__functions.h"
#include "mesh_msgs/msg/detail/vector_field__struct.h"


// Include directives for member types
// Member `positions`
#include "geometry_msgs/msg/point.h"
// Member `positions`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"
// Member `vectors`
#include "geometry_msgs/msg/vector3.h"
// Member `vectors`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__VectorField_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mesh_msgs__msg__VectorField__init(message_memory);
}

void mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__VectorField_fini_function(void * message_memory)
{
  mesh_msgs__msg__VectorField__fini(message_memory);
}

size_t mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__size_function__VectorField__positions(
  const void * untyped_member)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__get_const_function__VectorField__positions(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__get_function__VectorField__positions(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__fetch_function__VectorField__positions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Point * item =
    ((const geometry_msgs__msg__Point *)
    mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__get_const_function__VectorField__positions(untyped_member, index));
  geometry_msgs__msg__Point * value =
    (geometry_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__assign_function__VectorField__positions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Point * item =
    ((geometry_msgs__msg__Point *)
    mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__get_function__VectorField__positions(untyped_member, index));
  const geometry_msgs__msg__Point * value =
    (const geometry_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__resize_function__VectorField__positions(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  geometry_msgs__msg__Point__Sequence__fini(member);
  return geometry_msgs__msg__Point__Sequence__init(member, size);
}

size_t mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__size_function__VectorField__vectors(
  const void * untyped_member)
{
  const geometry_msgs__msg__Vector3__Sequence * member =
    (const geometry_msgs__msg__Vector3__Sequence *)(untyped_member);
  return member->size;
}

const void * mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__get_const_function__VectorField__vectors(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Vector3__Sequence * member =
    (const geometry_msgs__msg__Vector3__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__get_function__VectorField__vectors(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Vector3__Sequence * member =
    (geometry_msgs__msg__Vector3__Sequence *)(untyped_member);
  return &member->data[index];
}

void mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__fetch_function__VectorField__vectors(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Vector3 * item =
    ((const geometry_msgs__msg__Vector3 *)
    mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__get_const_function__VectorField__vectors(untyped_member, index));
  geometry_msgs__msg__Vector3 * value =
    (geometry_msgs__msg__Vector3 *)(untyped_value);
  *value = *item;
}

void mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__assign_function__VectorField__vectors(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Vector3 * item =
    ((geometry_msgs__msg__Vector3 *)
    mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__get_function__VectorField__vectors(untyped_member, index));
  const geometry_msgs__msg__Vector3 * value =
    (const geometry_msgs__msg__Vector3 *)(untyped_value);
  *item = *value;
}

bool mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__resize_function__VectorField__vectors(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Vector3__Sequence * member =
    (geometry_msgs__msg__Vector3__Sequence *)(untyped_member);
  geometry_msgs__msg__Vector3__Sequence__fini(member);
  return geometry_msgs__msg__Vector3__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__VectorField_message_member_array[2] = {
  {
    "positions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs__msg__VectorField, positions),  // bytes offset in struct
    NULL,  // default value
    mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__size_function__VectorField__positions,  // size() function pointer
    mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__get_const_function__VectorField__positions,  // get_const(index) function pointer
    mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__get_function__VectorField__positions,  // get(index) function pointer
    mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__fetch_function__VectorField__positions,  // fetch(index, &value) function pointer
    mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__assign_function__VectorField__positions,  // assign(index, value) function pointer
    mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__resize_function__VectorField__positions  // resize(index) function pointer
  },
  {
    "vectors",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs__msg__VectorField, vectors),  // bytes offset in struct
    NULL,  // default value
    mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__size_function__VectorField__vectors,  // size() function pointer
    mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__get_const_function__VectorField__vectors,  // get_const(index) function pointer
    mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__get_function__VectorField__vectors,  // get(index) function pointer
    mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__fetch_function__VectorField__vectors,  // fetch(index, &value) function pointer
    mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__assign_function__VectorField__vectors,  // assign(index, value) function pointer
    mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__resize_function__VectorField__vectors  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__VectorField_message_members = {
  "mesh_msgs__msg",  // message namespace
  "VectorField",  // message name
  2,  // number of fields
  sizeof(mesh_msgs__msg__VectorField),
  false,  // has_any_key_member_
  mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__VectorField_message_member_array,  // message members
  mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__VectorField_init_function,  // function to initialize message memory (memory has to be allocated)
  mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__VectorField_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__VectorField_message_type_support_handle = {
  0,
  &mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__VectorField_message_members,
  get_message_typesupport_handle_function,
  &mesh_msgs__msg__VectorField__get_type_hash,
  &mesh_msgs__msg__VectorField__get_type_description,
  &mesh_msgs__msg__VectorField__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mesh_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, msg, VectorField)() {
  mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__VectorField_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__VectorField_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__VectorField_message_type_support_handle.typesupport_identifier) {
    mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__VectorField_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mesh_msgs__msg__VectorField__rosidl_typesupport_introspection_c__VectorField_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
