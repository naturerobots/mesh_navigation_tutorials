// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mesh_msgs:msg/MeshVertexColors.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_vertex_colors.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COLORS__BUILDER_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COLORS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mesh_msgs/msg/detail/mesh_vertex_colors__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mesh_msgs
{

namespace msg
{

namespace builder
{

class Init_MeshVertexColors_vertex_colors
{
public:
  Init_MeshVertexColors_vertex_colors()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mesh_msgs::msg::MeshVertexColors vertex_colors(::mesh_msgs::msg::MeshVertexColors::_vertex_colors_type arg)
  {
    msg_.vertex_colors = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::msg::MeshVertexColors msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::msg::MeshVertexColors>()
{
  return mesh_msgs::msg::builder::Init_MeshVertexColors_vertex_colors();
}

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COLORS__BUILDER_HPP_
