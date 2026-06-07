// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mesh_msgs:msg/MeshVertexCostsStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_vertex_costs_stamped.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS_STAMPED__BUILDER_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mesh_msgs/msg/detail/mesh_vertex_costs_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mesh_msgs
{

namespace msg
{

namespace builder
{

class Init_MeshVertexCostsStamped_mesh_vertex_costs
{
public:
  explicit Init_MeshVertexCostsStamped_mesh_vertex_costs(::mesh_msgs::msg::MeshVertexCostsStamped & msg)
  : msg_(msg)
  {}
  ::mesh_msgs::msg::MeshVertexCostsStamped mesh_vertex_costs(::mesh_msgs::msg::MeshVertexCostsStamped::_mesh_vertex_costs_type arg)
  {
    msg_.mesh_vertex_costs = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::msg::MeshVertexCostsStamped msg_;
};

class Init_MeshVertexCostsStamped_type
{
public:
  explicit Init_MeshVertexCostsStamped_type(::mesh_msgs::msg::MeshVertexCostsStamped & msg)
  : msg_(msg)
  {}
  Init_MeshVertexCostsStamped_mesh_vertex_costs type(::mesh_msgs::msg::MeshVertexCostsStamped::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_MeshVertexCostsStamped_mesh_vertex_costs(msg_);
  }

private:
  ::mesh_msgs::msg::MeshVertexCostsStamped msg_;
};

class Init_MeshVertexCostsStamped_uuid
{
public:
  explicit Init_MeshVertexCostsStamped_uuid(::mesh_msgs::msg::MeshVertexCostsStamped & msg)
  : msg_(msg)
  {}
  Init_MeshVertexCostsStamped_type uuid(::mesh_msgs::msg::MeshVertexCostsStamped::_uuid_type arg)
  {
    msg_.uuid = std::move(arg);
    return Init_MeshVertexCostsStamped_type(msg_);
  }

private:
  ::mesh_msgs::msg::MeshVertexCostsStamped msg_;
};

class Init_MeshVertexCostsStamped_header
{
public:
  Init_MeshVertexCostsStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MeshVertexCostsStamped_uuid header(::mesh_msgs::msg::MeshVertexCostsStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MeshVertexCostsStamped_uuid(msg_);
  }

private:
  ::mesh_msgs::msg::MeshVertexCostsStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::msg::MeshVertexCostsStamped>()
{
  return mesh_msgs::msg::builder::Init_MeshVertexCostsStamped_header();
}

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS_STAMPED__BUILDER_HPP_
