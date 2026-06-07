// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mesh_msgs:msg/MeshMaterialsStamped.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mesh_msgs/msg/detail/mesh_materials_stamped__rosidl_typesupport_introspection_c.h"
#include "mesh_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mesh_msgs/msg/detail/mesh_materials_stamped__functions.h"
#include "mesh_msgs/msg/detail/mesh_materials_stamped__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `uuid`
#include "rosidl_runtime_c/string_functions.h"
// Member `mesh_materials`
#include "mesh_msgs/msg/mesh_materials.h"
// Member `mesh_materials`
#include "mesh_msgs/msg/detail/mesh_materials__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mesh_msgs__msg__MeshMaterialsStamped__rosidl_typesupport_introspection_c__MeshMaterialsStamped_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mesh_msgs__msg__MeshMaterialsStamped__init(message_memory);
}

void mesh_msgs__msg__MeshMaterialsStamped__rosidl_typesupport_introspection_c__MeshMaterialsStamped_fini_function(void * message_memory)
{
  mesh_msgs__msg__MeshMaterialsStamped__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mesh_msgs__msg__MeshMaterialsStamped__rosidl_typesupport_introspection_c__MeshMaterialsStamped_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs__msg__MeshMaterialsStamped, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "uuid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs__msg__MeshMaterialsStamped, uuid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mesh_materials",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs__msg__MeshMaterialsStamped, mesh_materials),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mesh_msgs__msg__MeshMaterialsStamped__rosidl_typesupport_introspection_c__MeshMaterialsStamped_message_members = {
  "mesh_msgs__msg",  // message namespace
  "MeshMaterialsStamped",  // message name
  3,  // number of fields
  sizeof(mesh_msgs__msg__MeshMaterialsStamped),
  false,  // has_any_key_member_
  mesh_msgs__msg__MeshMaterialsStamped__rosidl_typesupport_introspection_c__MeshMaterialsStamped_message_member_array,  // message members
  mesh_msgs__msg__MeshMaterialsStamped__rosidl_typesupport_introspection_c__MeshMaterialsStamped_init_function,  // function to initialize message memory (memory has to be allocated)
  mesh_msgs__msg__MeshMaterialsStamped__rosidl_typesupport_introspection_c__MeshMaterialsStamped_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mesh_msgs__msg__MeshMaterialsStamped__rosidl_typesupport_introspection_c__MeshMaterialsStamped_message_type_support_handle = {
  0,
  &mesh_msgs__msg__MeshMaterialsStamped__rosidl_typesupport_introspection_c__MeshMaterialsStamped_message_members,
  get_message_typesupport_handle_function,
  &mesh_msgs__msg__MeshMaterialsStamped__get_type_hash,
  &mesh_msgs__msg__MeshMaterialsStamped__get_type_description,
  &mesh_msgs__msg__MeshMaterialsStamped__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mesh_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, msg, MeshMaterialsStamped)() {
  mesh_msgs__msg__MeshMaterialsStamped__rosidl_typesupport_introspection_c__MeshMaterialsStamped_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  mesh_msgs__msg__MeshMaterialsStamped__rosidl_typesupport_introspection_c__MeshMaterialsStamped_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, msg, MeshMaterials)();
  if (!mesh_msgs__msg__MeshMaterialsStamped__rosidl_typesupport_introspection_c__MeshMaterialsStamped_message_type_support_handle.typesupport_identifier) {
    mesh_msgs__msg__MeshMaterialsStamped__rosidl_typesupport_introspection_c__MeshMaterialsStamped_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mesh_msgs__msg__MeshMaterialsStamped__rosidl_typesupport_introspection_c__MeshMaterialsStamped_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
