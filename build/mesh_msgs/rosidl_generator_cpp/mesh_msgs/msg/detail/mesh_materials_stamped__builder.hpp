// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mesh_msgs:msg/MeshMaterialsStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_materials_stamped.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_MATERIALS_STAMPED__BUILDER_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_MATERIALS_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mesh_msgs/msg/detail/mesh_materials_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mesh_msgs
{

namespace msg
{

namespace builder
{

class Init_MeshMaterialsStamped_mesh_materials
{
public:
  explicit Init_MeshMaterialsStamped_mesh_materials(::mesh_msgs::msg::MeshMaterialsStamped & msg)
  : msg_(msg)
  {}
  ::mesh_msgs::msg::MeshMaterialsStamped mesh_materials(::mesh_msgs::msg::MeshMaterialsStamped::_mesh_materials_type arg)
  {
    msg_.mesh_materials = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::msg::MeshMaterialsStamped msg_;
};

class Init_MeshMaterialsStamped_uuid
{
public:
  explicit Init_MeshMaterialsStamped_uuid(::mesh_msgs::msg::MeshMaterialsStamped & msg)
  : msg_(msg)
  {}
  Init_MeshMaterialsStamped_mesh_materials uuid(::mesh_msgs::msg::MeshMaterialsStamped::_uuid_type arg)
  {
    msg_.uuid = std::move(arg);
    return Init_MeshMaterialsStamped_mesh_materials(msg_);
  }

private:
  ::mesh_msgs::msg::MeshMaterialsStamped msg_;
};

class Init_MeshMaterialsStamped_header
{
public:
  Init_MeshMaterialsStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MeshMaterialsStamped_uuid header(::mesh_msgs::msg::MeshMaterialsStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MeshMaterialsStamped_uuid(msg_);
  }

private:
  ::mesh_msgs::msg::MeshMaterialsStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::msg::MeshMaterialsStamped>()
{
  return mesh_msgs::msg::builder::Init_MeshMaterialsStamped_header();
}

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__MESH_MATERIALS_STAMPED__BUILDER_HPP_
