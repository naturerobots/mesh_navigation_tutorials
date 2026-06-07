// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mesh_msgs:msg/MeshGeometry.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mesh_msgs/msg/detail/mesh_geometry__functions.h"
#include "mesh_msgs/msg/detail/mesh_geometry__struct.hpp"
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

void MeshGeometry_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mesh_msgs::msg::MeshGeometry(_init);
}

void MeshGeometry_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mesh_msgs::msg::MeshGeometry *>(message_memory);
  typed_message->~MeshGeometry();
}

size_t size_function__MeshGeometry__vertices(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MeshGeometry__vertices(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void * get_function__MeshGeometry__vertices(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void fetch_function__MeshGeometry__vertices(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const geometry_msgs::msg::Point *>(
    get_const_function__MeshGeometry__vertices(untyped_member, index));
  auto & value = *reinterpret_cast<geometry_msgs::msg::Point *>(untyped_value);
  value = item;
}

void assign_function__MeshGeometry__vertices(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<geometry_msgs::msg::Point *>(
    get_function__MeshGeometry__vertices(untyped_member, index));
  const auto & value = *reinterpret_cast<const geometry_msgs::msg::Point *>(untyped_value);
  item = value;
}

void resize_function__MeshGeometry__vertices(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MeshGeometry__vertex_normals(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MeshGeometry__vertex_normals(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void * get_function__MeshGeometry__vertex_normals(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  return &member[index];
}

void fetch_function__MeshGeometry__vertex_normals(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const geometry_msgs::msg::Point *>(
    get_const_function__MeshGeometry__vertex_normals(untyped_member, index));
  auto & value = *reinterpret_cast<geometry_msgs::msg::Point *>(untyped_value);
  value = item;
}

void assign_function__MeshGeometry__vertex_normals(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<geometry_msgs::msg::Point *>(
    get_function__MeshGeometry__vertex_normals(untyped_member, index));
  const auto & value = *reinterpret_cast<const geometry_msgs::msg::Point *>(untyped_value);
  item = value;
}

void resize_function__MeshGeometry__vertex_normals(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<geometry_msgs::msg::Point> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MeshGeometry__faces(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<mesh_msgs::msg::MeshTriangleIndices> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MeshGeometry__faces(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<mesh_msgs::msg::MeshTriangleIndices> *>(untyped_member);
  return &member[index];
}

void * get_function__MeshGeometry__faces(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<mesh_msgs::msg::MeshTriangleIndices> *>(untyped_member);
  return &member[index];
}

void fetch_function__MeshGeometry__faces(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const mesh_msgs::msg::MeshTriangleIndices *>(
    get_const_function__MeshGeometry__faces(untyped_member, index));
  auto & value = *reinterpret_cast<mesh_msgs::msg::MeshTriangleIndices *>(untyped_value);
  value = item;
}

void assign_function__MeshGeometry__faces(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<mesh_msgs::msg::MeshTriangleIndices *>(
    get_function__MeshGeometry__faces(untyped_member, index));
  const auto & value = *reinterpret_cast<const mesh_msgs::msg::MeshTriangleIndices *>(untyped_value);
  item = value;
}

void resize_function__MeshGeometry__faces(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<mesh_msgs::msg::MeshTriangleIndices> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MeshGeometry_message_member_array[3] = {
  {
    "vertices",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Point>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs::msg::MeshGeometry, vertices),  // bytes offset in struct
    nullptr,  // default value
    size_function__MeshGeometry__vertices,  // size() function pointer
    get_const_function__MeshGeometry__vertices,  // get_const(index) function pointer
    get_function__MeshGeometry__vertices,  // get(index) function pointer
    fetch_function__MeshGeometry__vertices,  // fetch(index, &value) function pointer
    assign_function__MeshGeometry__vertices,  // assign(index, value) function pointer
    resize_function__MeshGeometry__vertices  // resize(index) function pointer
  },
  {
    "vertex_normals",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Point>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs::msg::MeshGeometry, vertex_normals),  // bytes offset in struct
    nullptr,  // default value
    size_function__MeshGeometry__vertex_normals,  // size() function pointer
    get_const_function__MeshGeometry__vertex_normals,  // get_const(index) function pointer
    get_function__MeshGeometry__vertex_normals,  // get(index) function pointer
    fetch_function__MeshGeometry__vertex_normals,  // fetch(index, &value) function pointer
    assign_function__MeshGeometry__vertex_normals,  // assign(index, value) function pointer
    resize_function__MeshGeometry__vertex_normals  // resize(index) function pointer
  },
  {
    "faces",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<mesh_msgs::msg::MeshTriangleIndices>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs::msg::MeshGeometry, faces),  // bytes offset in struct
    nullptr,  // default value
    size_function__MeshGeometry__faces,  // size() function pointer
    get_const_function__MeshGeometry__faces,  // get_const(index) function pointer
    get_function__MeshGeometry__faces,  // get(index) function pointer
    fetch_function__MeshGeometry__faces,  // fetch(index, &value) function pointer
    assign_function__MeshGeometry__faces,  // assign(index, value) function pointer
    resize_function__MeshGeometry__faces  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MeshGeometry_message_members = {
  "mesh_msgs::msg",  // message namespace
  "MeshGeometry",  // message name
  3,  // number of fields
  sizeof(mesh_msgs::msg::MeshGeometry),
  false,  // has_any_key_member_
  MeshGeometry_message_member_array,  // message members
  MeshGeometry_init_function,  // function to initialize message memory (memory has to be allocated)
  MeshGeometry_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MeshGeometry_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MeshGeometry_message_members,
  get_message_typesupport_handle_function,
  &mesh_msgs__msg__MeshGeometry__get_type_hash,
  &mesh_msgs__msg__MeshGeometry__get_type_description,
  &mesh_msgs__msg__MeshGeometry__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mesh_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mesh_msgs::msg::MeshGeometry>()
{
  return &::mesh_msgs::msg::rosidl_typesupport_introspection_cpp::MeshGeometry_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mesh_msgs, msg, MeshGeometry)() {
  return &::mesh_msgs::msg::rosidl_typesupport_introspection_cpp::MeshGeometry_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
