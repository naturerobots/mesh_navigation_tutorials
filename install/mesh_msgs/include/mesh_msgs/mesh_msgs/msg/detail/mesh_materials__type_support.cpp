// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mesh_msgs:msg/MeshMaterials.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mesh_msgs/msg/detail/mesh_materials__functions.h"
#include "mesh_msgs/msg/detail/mesh_materials__struct.hpp"
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

void MeshMaterials_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mesh_msgs::msg::MeshMaterials(_init);
}

void MeshMaterials_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mesh_msgs::msg::MeshMaterials *>(message_memory);
  typed_message->~MeshMaterials();
}

size_t size_function__MeshMaterials__clusters(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<mesh_msgs::msg::MeshFaceCluster> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MeshMaterials__clusters(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<mesh_msgs::msg::MeshFaceCluster> *>(untyped_member);
  return &member[index];
}

void * get_function__MeshMaterials__clusters(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<mesh_msgs::msg::MeshFaceCluster> *>(untyped_member);
  return &member[index];
}

void fetch_function__MeshMaterials__clusters(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const mesh_msgs::msg::MeshFaceCluster *>(
    get_const_function__MeshMaterials__clusters(untyped_member, index));
  auto & value = *reinterpret_cast<mesh_msgs::msg::MeshFaceCluster *>(untyped_value);
  value = item;
}

void assign_function__MeshMaterials__clusters(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<mesh_msgs::msg::MeshFaceCluster *>(
    get_function__MeshMaterials__clusters(untyped_member, index));
  const auto & value = *reinterpret_cast<const mesh_msgs::msg::MeshFaceCluster *>(untyped_value);
  item = value;
}

void resize_function__MeshMaterials__clusters(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<mesh_msgs::msg::MeshFaceCluster> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MeshMaterials__materials(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<mesh_msgs::msg::MeshMaterial> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MeshMaterials__materials(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<mesh_msgs::msg::MeshMaterial> *>(untyped_member);
  return &member[index];
}

void * get_function__MeshMaterials__materials(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<mesh_msgs::msg::MeshMaterial> *>(untyped_member);
  return &member[index];
}

void fetch_function__MeshMaterials__materials(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const mesh_msgs::msg::MeshMaterial *>(
    get_const_function__MeshMaterials__materials(untyped_member, index));
  auto & value = *reinterpret_cast<mesh_msgs::msg::MeshMaterial *>(untyped_value);
  value = item;
}

void assign_function__MeshMaterials__materials(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<mesh_msgs::msg::MeshMaterial *>(
    get_function__MeshMaterials__materials(untyped_member, index));
  const auto & value = *reinterpret_cast<const mesh_msgs::msg::MeshMaterial *>(untyped_value);
  item = value;
}

void resize_function__MeshMaterials__materials(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<mesh_msgs::msg::MeshMaterial> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MeshMaterials__cluster_materials(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MeshMaterials__cluster_materials(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__MeshMaterials__cluster_materials(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__MeshMaterials__cluster_materials(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint32_t *>(
    get_const_function__MeshMaterials__cluster_materials(untyped_member, index));
  auto & value = *reinterpret_cast<uint32_t *>(untyped_value);
  value = item;
}

void assign_function__MeshMaterials__cluster_materials(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint32_t *>(
    get_function__MeshMaterials__cluster_materials(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint32_t *>(untyped_value);
  item = value;
}

void resize_function__MeshMaterials__cluster_materials(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint32_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MeshMaterials__vertex_tex_coords(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<mesh_msgs::msg::MeshVertexTexCoords> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MeshMaterials__vertex_tex_coords(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<mesh_msgs::msg::MeshVertexTexCoords> *>(untyped_member);
  return &member[index];
}

void * get_function__MeshMaterials__vertex_tex_coords(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<mesh_msgs::msg::MeshVertexTexCoords> *>(untyped_member);
  return &member[index];
}

void fetch_function__MeshMaterials__vertex_tex_coords(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const mesh_msgs::msg::MeshVertexTexCoords *>(
    get_const_function__MeshMaterials__vertex_tex_coords(untyped_member, index));
  auto & value = *reinterpret_cast<mesh_msgs::msg::MeshVertexTexCoords *>(untyped_value);
  value = item;
}

void assign_function__MeshMaterials__vertex_tex_coords(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<mesh_msgs::msg::MeshVertexTexCoords *>(
    get_function__MeshMaterials__vertex_tex_coords(untyped_member, index));
  const auto & value = *reinterpret_cast<const mesh_msgs::msg::MeshVertexTexCoords *>(untyped_value);
  item = value;
}

void resize_function__MeshMaterials__vertex_tex_coords(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<mesh_msgs::msg::MeshVertexTexCoords> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MeshMaterials_message_member_array[4] = {
  {
    "clusters",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<mesh_msgs::msg::MeshFaceCluster>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs::msg::MeshMaterials, clusters),  // bytes offset in struct
    nullptr,  // default value
    size_function__MeshMaterials__clusters,  // size() function pointer
    get_const_function__MeshMaterials__clusters,  // get_const(index) function pointer
    get_function__MeshMaterials__clusters,  // get(index) function pointer
    fetch_function__MeshMaterials__clusters,  // fetch(index, &value) function pointer
    assign_function__MeshMaterials__clusters,  // assign(index, value) function pointer
    resize_function__MeshMaterials__clusters  // resize(index) function pointer
  },
  {
    "materials",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<mesh_msgs::msg::MeshMaterial>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs::msg::MeshMaterials, materials),  // bytes offset in struct
    nullptr,  // default value
    size_function__MeshMaterials__materials,  // size() function pointer
    get_const_function__MeshMaterials__materials,  // get_const(index) function pointer
    get_function__MeshMaterials__materials,  // get(index) function pointer
    fetch_function__MeshMaterials__materials,  // fetch(index, &value) function pointer
    assign_function__MeshMaterials__materials,  // assign(index, value) function pointer
    resize_function__MeshMaterials__materials  // resize(index) function pointer
  },
  {
    "cluster_materials",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs::msg::MeshMaterials, cluster_materials),  // bytes offset in struct
    nullptr,  // default value
    size_function__MeshMaterials__cluster_materials,  // size() function pointer
    get_const_function__MeshMaterials__cluster_materials,  // get_const(index) function pointer
    get_function__MeshMaterials__cluster_materials,  // get(index) function pointer
    fetch_function__MeshMaterials__cluster_materials,  // fetch(index, &value) function pointer
    assign_function__MeshMaterials__cluster_materials,  // assign(index, value) function pointer
    resize_function__MeshMaterials__cluster_materials  // resize(index) function pointer
  },
  {
    "vertex_tex_coords",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<mesh_msgs::msg::MeshVertexTexCoords>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs::msg::MeshMaterials, vertex_tex_coords),  // bytes offset in struct
    nullptr,  // default value
    size_function__MeshMaterials__vertex_tex_coords,  // size() function pointer
    get_const_function__MeshMaterials__vertex_tex_coords,  // get_const(index) function pointer
    get_function__MeshMaterials__vertex_tex_coords,  // get(index) function pointer
    fetch_function__MeshMaterials__vertex_tex_coords,  // fetch(index, &value) function pointer
    assign_function__MeshMaterials__vertex_tex_coords,  // assign(index, value) function pointer
    resize_function__MeshMaterials__vertex_tex_coords  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MeshMaterials_message_members = {
  "mesh_msgs::msg",  // message namespace
  "MeshMaterials",  // message name
  4,  // number of fields
  sizeof(mesh_msgs::msg::MeshMaterials),
  false,  // has_any_key_member_
  MeshMaterials_message_member_array,  // message members
  MeshMaterials_init_function,  // function to initialize message memory (memory has to be allocated)
  MeshMaterials_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MeshMaterials_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MeshMaterials_message_members,
  get_message_typesupport_handle_function,
  &mesh_msgs__msg__MeshMaterials__get_type_hash,
  &mesh_msgs__msg__MeshMaterials__get_type_description,
  &mesh_msgs__msg__MeshMaterials__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mesh_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mesh_msgs::msg::MeshMaterials>()
{
  return &::mesh_msgs::msg::rosidl_typesupport_introspection_cpp::MeshMaterials_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mesh_msgs, msg, MeshMaterials)() {
  return &::mesh_msgs::msg::rosidl_typesupport_introspection_cpp::MeshMaterials_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
