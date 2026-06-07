// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mesh_msgs:msg/MeshGeometry.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_geometry.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_GEOMETRY__BUILDER_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_GEOMETRY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mesh_msgs/msg/detail/mesh_geometry__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mesh_msgs
{

namespace msg
{

namespace builder
{

class Init_MeshGeometry_faces
{
public:
  explicit Init_MeshGeometry_faces(::mesh_msgs::msg::MeshGeometry & msg)
  : msg_(msg)
  {}
  ::mesh_msgs::msg::MeshGeometry faces(::mesh_msgs::msg::MeshGeometry::_faces_type arg)
  {
    msg_.faces = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::msg::MeshGeometry msg_;
};

class Init_MeshGeometry_vertex_normals
{
public:
  explicit Init_MeshGeometry_vertex_normals(::mesh_msgs::msg::MeshGeometry & msg)
  : msg_(msg)
  {}
  Init_MeshGeometry_faces vertex_normals(::mesh_msgs::msg::MeshGeometry::_vertex_normals_type arg)
  {
    msg_.vertex_normals = std::move(arg);
    return Init_MeshGeometry_faces(msg_);
  }

private:
  ::mesh_msgs::msg::MeshGeometry msg_;
};

class Init_MeshGeometry_vertices
{
public:
  Init_MeshGeometry_vertices()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MeshGeometry_vertex_normals vertices(::mesh_msgs::msg::MeshGeometry::_vertices_type arg)
  {
    msg_.vertices = std::move(arg);
    return Init_MeshGeometry_vertex_normals(msg_);
  }

private:
  ::mesh_msgs::msg::MeshGeometry msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::msg::MeshGeometry>()
{
  return mesh_msgs::msg::builder::Init_MeshGeometry_vertices();
}

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__MESH_GEOMETRY__BUILDER_HPP_
