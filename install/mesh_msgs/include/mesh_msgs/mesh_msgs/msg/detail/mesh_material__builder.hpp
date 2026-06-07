// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mesh_msgs:msg/MeshMaterial.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_material.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_MATERIAL__BUILDER_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_MATERIAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mesh_msgs/msg/detail/mesh_material__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mesh_msgs
{

namespace msg
{

namespace builder
{

class Init_MeshMaterial_has_texture
{
public:
  explicit Init_MeshMaterial_has_texture(::mesh_msgs::msg::MeshMaterial & msg)
  : msg_(msg)
  {}
  ::mesh_msgs::msg::MeshMaterial has_texture(::mesh_msgs::msg::MeshMaterial::_has_texture_type arg)
  {
    msg_.has_texture = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::msg::MeshMaterial msg_;
};

class Init_MeshMaterial_color
{
public:
  explicit Init_MeshMaterial_color(::mesh_msgs::msg::MeshMaterial & msg)
  : msg_(msg)
  {}
  Init_MeshMaterial_has_texture color(::mesh_msgs::msg::MeshMaterial::_color_type arg)
  {
    msg_.color = std::move(arg);
    return Init_MeshMaterial_has_texture(msg_);
  }

private:
  ::mesh_msgs::msg::MeshMaterial msg_;
};

class Init_MeshMaterial_texture_index
{
public:
  Init_MeshMaterial_texture_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MeshMaterial_color texture_index(::mesh_msgs::msg::MeshMaterial::_texture_index_type arg)
  {
    msg_.texture_index = std::move(arg);
    return Init_MeshMaterial_color(msg_);
  }

private:
  ::mesh_msgs::msg::MeshMaterial msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::msg::MeshMaterial>()
{
  return mesh_msgs::msg::builder::Init_MeshMaterial_texture_index();
}

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__MESH_MATERIAL__BUILDER_HPP_
