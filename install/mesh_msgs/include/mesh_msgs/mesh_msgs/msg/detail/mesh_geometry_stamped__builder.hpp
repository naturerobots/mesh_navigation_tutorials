// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mesh_msgs:msg/MeshGeometryStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_geometry_stamped.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_GEOMETRY_STAMPED__BUILDER_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_GEOMETRY_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mesh_msgs/msg/detail/mesh_geometry_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mesh_msgs
{

namespace msg
{

namespace builder
{

class Init_MeshGeometryStamped_mesh_geometry
{
public:
  explicit Init_MeshGeometryStamped_mesh_geometry(::mesh_msgs::msg::MeshGeometryStamped & msg)
  : msg_(msg)
  {}
  ::mesh_msgs::msg::MeshGeometryStamped mesh_geometry(::mesh_msgs::msg::MeshGeometryStamped::_mesh_geometry_type arg)
  {
    msg_.mesh_geometry = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::msg::MeshGeometryStamped msg_;
};

class Init_MeshGeometryStamped_uuid
{
public:
  explicit Init_MeshGeometryStamped_uuid(::mesh_msgs::msg::MeshGeometryStamped & msg)
  : msg_(msg)
  {}
  Init_MeshGeometryStamped_mesh_geometry uuid(::mesh_msgs::msg::MeshGeometryStamped::_uuid_type arg)
  {
    msg_.uuid = std::move(arg);
    return Init_MeshGeometryStamped_mesh_geometry(msg_);
  }

private:
  ::mesh_msgs::msg::MeshGeometryStamped msg_;
};

class Init_MeshGeometryStamped_header
{
public:
  Init_MeshGeometryStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MeshGeometryStamped_uuid header(::mesh_msgs::msg::MeshGeometryStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MeshGeometryStamped_uuid(msg_);
  }

private:
  ::mesh_msgs::msg::MeshGeometryStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::msg::MeshGeometryStamped>()
{
  return mesh_msgs::msg::builder::Init_MeshGeometryStamped_header();
}

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__MESH_GEOMETRY_STAMPED__BUILDER_HPP_
