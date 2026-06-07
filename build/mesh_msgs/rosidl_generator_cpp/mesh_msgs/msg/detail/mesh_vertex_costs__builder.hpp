// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mesh_msgs:msg/MeshVertexCosts.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_vertex_costs.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS__BUILDER_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mesh_msgs/msg/detail/mesh_vertex_costs__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mesh_msgs
{

namespace msg
{

namespace builder
{

class Init_MeshVertexCosts_costs
{
public:
  Init_MeshVertexCosts_costs()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mesh_msgs::msg::MeshVertexCosts costs(::mesh_msgs::msg::MeshVertexCosts::_costs_type arg)
  {
    msg_.costs = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::msg::MeshVertexCosts msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::msg::MeshVertexCosts>()
{
  return mesh_msgs::msg::builder::Init_MeshVertexCosts_costs();
}

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS__BUILDER_HPP_
