// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mesh_msgs:msg/MeshVertexTexCoords.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_vertex_tex_coords.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_VERTEX_TEX_COORDS__BUILDER_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_VERTEX_TEX_COORDS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mesh_msgs/msg/detail/mesh_vertex_tex_coords__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mesh_msgs
{

namespace msg
{

namespace builder
{

class Init_MeshVertexTexCoords_v
{
public:
  explicit Init_MeshVertexTexCoords_v(::mesh_msgs::msg::MeshVertexTexCoords & msg)
  : msg_(msg)
  {}
  ::mesh_msgs::msg::MeshVertexTexCoords v(::mesh_msgs::msg::MeshVertexTexCoords::_v_type arg)
  {
    msg_.v = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::msg::MeshVertexTexCoords msg_;
};

class Init_MeshVertexTexCoords_u
{
public:
  Init_MeshVertexTexCoords_u()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MeshVertexTexCoords_v u(::mesh_msgs::msg::MeshVertexTexCoords::_u_type arg)
  {
    msg_.u = std::move(arg);
    return Init_MeshVertexTexCoords_v(msg_);
  }

private:
  ::mesh_msgs::msg::MeshVertexTexCoords msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::msg::MeshVertexTexCoords>()
{
  return mesh_msgs::msg::builder::Init_MeshVertexTexCoords_u();
}

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__MESH_VERTEX_TEX_COORDS__BUILDER_HPP_
