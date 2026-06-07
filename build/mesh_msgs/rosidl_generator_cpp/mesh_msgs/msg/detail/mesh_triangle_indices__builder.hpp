// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mesh_msgs:msg/MeshTriangleIndices.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_triangle_indices.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_TRIANGLE_INDICES__BUILDER_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_TRIANGLE_INDICES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mesh_msgs/msg/detail/mesh_triangle_indices__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mesh_msgs
{

namespace msg
{

namespace builder
{

class Init_MeshTriangleIndices_vertex_indices
{
public:
  Init_MeshTriangleIndices_vertex_indices()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mesh_msgs::msg::MeshTriangleIndices vertex_indices(::mesh_msgs::msg::MeshTriangleIndices::_vertex_indices_type arg)
  {
    msg_.vertex_indices = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::msg::MeshTriangleIndices msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::msg::MeshTriangleIndices>()
{
  return mesh_msgs::msg::builder::Init_MeshTriangleIndices_vertex_indices();
}

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__MESH_TRIANGLE_INDICES__BUILDER_HPP_
