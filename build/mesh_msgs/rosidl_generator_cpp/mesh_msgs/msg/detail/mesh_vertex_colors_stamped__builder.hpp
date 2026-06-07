// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mesh_msgs:msg/MeshVertexColorsStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_vertex_colors_stamped.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COLORS_STAMPED__BUILDER_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COLORS_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mesh_msgs/msg/detail/mesh_vertex_colors_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mesh_msgs
{

namespace msg
{

namespace builder
{

class Init_MeshVertexColorsStamped_mesh_vertex_colors
{
public:
  explicit Init_MeshVertexColorsStamped_mesh_vertex_colors(::mesh_msgs::msg::MeshVertexColorsStamped & msg)
  : msg_(msg)
  {}
  ::mesh_msgs::msg::MeshVertexColorsStamped mesh_vertex_colors(::mesh_msgs::msg::MeshVertexColorsStamped::_mesh_vertex_colors_type arg)
  {
    msg_.mesh_vertex_colors = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::msg::MeshVertexColorsStamped msg_;
};

class Init_MeshVertexColorsStamped_uuid
{
public:
  explicit Init_MeshVertexColorsStamped_uuid(::mesh_msgs::msg::MeshVertexColorsStamped & msg)
  : msg_(msg)
  {}
  Init_MeshVertexColorsStamped_mesh_vertex_colors uuid(::mesh_msgs::msg::MeshVertexColorsStamped::_uuid_type arg)
  {
    msg_.uuid = std::move(arg);
    return Init_MeshVertexColorsStamped_mesh_vertex_colors(msg_);
  }

private:
  ::mesh_msgs::msg::MeshVertexColorsStamped msg_;
};

class Init_MeshVertexColorsStamped_header
{
public:
  Init_MeshVertexColorsStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MeshVertexColorsStamped_uuid header(::mesh_msgs::msg::MeshVertexColorsStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MeshVertexColorsStamped_uuid(msg_);
  }

private:
  ::mesh_msgs::msg::MeshVertexColorsStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::msg::MeshVertexColorsStamped>()
{
  return mesh_msgs::msg::builder::Init_MeshVertexColorsStamped_header();
}

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COLORS_STAMPED__BUILDER_HPP_
