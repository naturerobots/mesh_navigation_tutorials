// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mesh_msgs:msg/MeshTexture.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mesh_msgs/msg/detail/mesh_texture__functions.h"
#include "mesh_msgs/msg/detail/mesh_texture__struct.hpp"
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

void MeshTexture_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mesh_msgs::msg::MeshTexture(_init);
}

void MeshTexture_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mesh_msgs::msg::MeshTexture *>(message_memory);
  typed_message->~MeshTexture();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MeshTexture_message_member_array[3] = {
  {
    "uuid",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs::msg::MeshTexture, uuid),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "texture_index",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs::msg::MeshTexture, texture_index),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "image",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sensor_msgs::msg::Image>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs::msg::MeshTexture, image),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MeshTexture_message_members = {
  "mesh_msgs::msg",  // message namespace
  "MeshTexture",  // message name
  3,  // number of fields
  sizeof(mesh_msgs::msg::MeshTexture),
  false,  // has_any_key_member_
  MeshTexture_message_member_array,  // message members
  MeshTexture_init_function,  // function to initialize message memory (memory has to be allocated)
  MeshTexture_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MeshTexture_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MeshTexture_message_members,
  get_message_typesupport_handle_function,
  &mesh_msgs__msg__MeshTexture__get_type_hash,
  &mesh_msgs__msg__MeshTexture__get_type_description,
  &mesh_msgs__msg__MeshTexture__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mesh_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mesh_msgs::msg::MeshTexture>()
{
  return &::mesh_msgs::msg::rosidl_typesupport_introspection_cpp::MeshTexture_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mesh_msgs, msg, MeshTexture)() {
  return &::mesh_msgs::msg::rosidl_typesupport_introspection_cpp::MeshTexture_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
