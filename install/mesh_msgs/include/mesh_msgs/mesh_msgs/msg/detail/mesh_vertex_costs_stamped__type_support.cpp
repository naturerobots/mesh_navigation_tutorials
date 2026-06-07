// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mesh_msgs:msg/MeshVertexCostsStamped.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mesh_msgs/msg/detail/mesh_vertex_costs_stamped__functions.h"
#include "mesh_msgs/msg/detail/mesh_vertex_costs_stamped__struct.hpp"
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

void MeshVertexCostsStamped_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mesh_msgs::msg::MeshVertexCostsStamped(_init);
}

void MeshVertexCostsStamped_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mesh_msgs::msg::MeshVertexCostsStamped *>(message_memory);
  typed_message->~MeshVertexCostsStamped();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MeshVertexCostsStamped_message_member_array[4] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs::msg::MeshVertexCostsStamped, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "uuid",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs::msg::MeshVertexCostsStamped, uuid),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "type",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs::msg::MeshVertexCostsStamped, type),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "mesh_vertex_costs",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<mesh_msgs::msg::MeshVertexCosts>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs::msg::MeshVertexCostsStamped, mesh_vertex_costs),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MeshVertexCostsStamped_message_members = {
  "mesh_msgs::msg",  // message namespace
  "MeshVertexCostsStamped",  // message name
  4,  // number of fields
  sizeof(mesh_msgs::msg::MeshVertexCostsStamped),
  false,  // has_any_key_member_
  MeshVertexCostsStamped_message_member_array,  // message members
  MeshVertexCostsStamped_init_function,  // function to initialize message memory (memory has to be allocated)
  MeshVertexCostsStamped_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MeshVertexCostsStamped_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MeshVertexCostsStamped_message_members,
  get_message_typesupport_handle_function,
  &mesh_msgs__msg__MeshVertexCostsStamped__get_type_hash,
  &mesh_msgs__msg__MeshVertexCostsStamped__get_type_description,
  &mesh_msgs__msg__MeshVertexCostsStamped__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mesh_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mesh_msgs::msg::MeshVertexCostsStamped>()
{
  return &::mesh_msgs::msg::rosidl_typesupport_introspection_cpp::MeshVertexCostsStamped_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mesh_msgs, msg, MeshVertexCostsStamped)() {
  return &::mesh_msgs::msg::rosidl_typesupport_introspection_cpp::MeshVertexCostsStamped_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
