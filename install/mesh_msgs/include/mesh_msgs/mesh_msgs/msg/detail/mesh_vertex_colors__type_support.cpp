// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mesh_msgs:msg/MeshVertexColors.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mesh_msgs/msg/detail/mesh_vertex_colors__functions.h"
#include "mesh_msgs/msg/detail/mesh_vertex_colors__struct.hpp"
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

void MeshVertexColors_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mesh_msgs::msg::MeshVertexColors(_init);
}

void MeshVertexColors_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mesh_msgs::msg::MeshVertexColors *>(message_memory);
  typed_message->~MeshVertexColors();
}

size_t size_function__MeshVertexColors__vertex_colors(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std_msgs::msg::ColorRGBA> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MeshVertexColors__vertex_colors(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std_msgs::msg::ColorRGBA> *>(untyped_member);
  return &member[index];
}

void * get_function__MeshVertexColors__vertex_colors(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std_msgs::msg::ColorRGBA> *>(untyped_member);
  return &member[index];
}

void fetch_function__MeshVertexColors__vertex_colors(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std_msgs::msg::ColorRGBA *>(
    get_const_function__MeshVertexColors__vertex_colors(untyped_member, index));
  auto & value = *reinterpret_cast<std_msgs::msg::ColorRGBA *>(untyped_value);
  value = item;
}

void assign_function__MeshVertexColors__vertex_colors(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std_msgs::msg::ColorRGBA *>(
    get_function__MeshVertexColors__vertex_colors(untyped_member, index));
  const auto & value = *reinterpret_cast<const std_msgs::msg::ColorRGBA *>(untyped_value);
  item = value;
}

void resize_function__MeshVertexColors__vertex_colors(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std_msgs::msg::ColorRGBA> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MeshVertexColors_message_member_array[1] = {
  {
    "vertex_colors",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::ColorRGBA>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs::msg::MeshVertexColors, vertex_colors),  // bytes offset in struct
    nullptr,  // default value
    size_function__MeshVertexColors__vertex_colors,  // size() function pointer
    get_const_function__MeshVertexColors__vertex_colors,  // get_const(index) function pointer
    get_function__MeshVertexColors__vertex_colors,  // get(index) function pointer
    fetch_function__MeshVertexColors__vertex_colors,  // fetch(index, &value) function pointer
    assign_function__MeshVertexColors__vertex_colors,  // assign(index, value) function pointer
    resize_function__MeshVertexColors__vertex_colors  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MeshVertexColors_message_members = {
  "mesh_msgs::msg",  // message namespace
  "MeshVertexColors",  // message name
  1,  // number of fields
  sizeof(mesh_msgs::msg::MeshVertexColors),
  false,  // has_any_key_member_
  MeshVertexColors_message_member_array,  // message members
  MeshVertexColors_init_function,  // function to initialize message memory (memory has to be allocated)
  MeshVertexColors_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MeshVertexColors_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MeshVertexColors_message_members,
  get_message_typesupport_handle_function,
  &mesh_msgs__msg__MeshVertexColors__get_type_hash,
  &mesh_msgs__msg__MeshVertexColors__get_type_description,
  &mesh_msgs__msg__MeshVertexColors__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mesh_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mesh_msgs::msg::MeshVertexColors>()
{
  return &::mesh_msgs::msg::rosidl_typesupport_introspection_cpp::MeshVertexColors_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mesh_msgs, msg, MeshVertexColors)() {
  return &::mesh_msgs::msg::rosidl_typesupport_introspection_cpp::MeshVertexColors_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
