// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mesh_msgs:msg/MeshVertexTexCoords.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mesh_msgs/msg/detail/mesh_vertex_tex_coords__functions.h"
#include "mesh_msgs/msg/detail/mesh_vertex_tex_coords__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mesh_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MeshVertexTexCoords_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mesh_msgs::msg::MeshVertexTexCoords(_init);
}

void MeshVertexTexCoords_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mesh_msgs::msg::MeshVertexTexCoords *>(message_memory);
  typed_message->~MeshVertexTexCoords();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MeshVertexTexCoords_message_member_array[2] = {
  {
    "u",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs::msg::MeshVertexTexCoords, u),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "v",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs::msg::MeshVertexTexCoords, v),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MeshVertexTexCoords_message_members = {
  "mesh_msgs::msg",  // message namespace
  "MeshVertexTexCoords",  // message name
  2,  // number of fields
  sizeof(mesh_msgs::msg::MeshVertexTexCoords),
  false,  // has_any_key_member_
  MeshVertexTexCoords_message_member_array,  // message members
  MeshVertexTexCoords_init_function,  // function to initialize message memory (memory has to be allocated)
  MeshVertexTexCoords_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MeshVertexTexCoords_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MeshVertexTexCoords_message_members,
  get_message_typesupport_handle_function,
  &mesh_msgs__msg__MeshVertexTexCoords__get_type_hash,
  &mesh_msgs__msg__MeshVertexTexCoords__get_type_description,
  &mesh_msgs__msg__MeshVertexTexCoords__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mesh_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mesh_msgs::msg::MeshVertexTexCoords>()
{
  return &::mesh_msgs::msg::rosidl_typesupport_introspection_cpp::MeshVertexTexCoords_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mesh_msgs, msg, MeshVertexTexCoords)() {
  return &::mesh_msgs::msg::rosidl_typesupport_introspection_cpp::MeshVertexTexCoords_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
