// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mesh_msgs:msg/MeshVertexCostsSparse.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_vertex_costs_sparse.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS_SPARSE__BUILDER_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS_SPARSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mesh_msgs/msg/detail/mesh_vertex_costs_sparse__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mesh_msgs
{

namespace msg
{

namespace builder
{

class Init_MeshVertexCostsSparse_default_value
{
public:
  explicit Init_MeshVertexCostsSparse_default_value(::mesh_msgs::msg::MeshVertexCostsSparse & msg)
  : msg_(msg)
  {}
  ::mesh_msgs::msg::MeshVertexCostsSparse default_value(::mesh_msgs::msg::MeshVertexCostsSparse::_default_value_type arg)
  {
    msg_.default_value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::msg::MeshVertexCostsSparse msg_;
};

class Init_MeshVertexCostsSparse_costs
{
public:
  explicit Init_MeshVertexCostsSparse_costs(::mesh_msgs::msg::MeshVertexCostsSparse & msg)
  : msg_(msg)
  {}
  Init_MeshVertexCostsSparse_default_value costs(::mesh_msgs::msg::MeshVertexCostsSparse::_costs_type arg)
  {
    msg_.costs = std::move(arg);
    return Init_MeshVertexCostsSparse_default_value(msg_);
  }

private:
  ::mesh_msgs::msg::MeshVertexCostsSparse msg_;
};

class Init_MeshVertexCostsSparse_vertices
{
public:
  Init_MeshVertexCostsSparse_vertices()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MeshVertexCostsSparse_costs vertices(::mesh_msgs::msg::MeshVertexCostsSparse::_vertices_type arg)
  {
    msg_.vertices = std::move(arg);
    return Init_MeshVertexCostsSparse_costs(msg_);
  }

private:
  ::mesh_msgs::msg::MeshVertexCostsSparse msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::msg::MeshVertexCostsSparse>()
{
  return mesh_msgs::msg::builder::Init_MeshVertexCostsSparse_vertices();
}

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS_SPARSE__BUILDER_HPP_
