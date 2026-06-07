// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mesh_msgs:msg/MeshMaterials.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_materials.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_MATERIALS__BUILDER_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_MATERIALS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mesh_msgs/msg/detail/mesh_materials__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mesh_msgs
{

namespace msg
{

namespace builder
{

class Init_MeshMaterials_vertex_tex_coords
{
public:
  explicit Init_MeshMaterials_vertex_tex_coords(::mesh_msgs::msg::MeshMaterials & msg)
  : msg_(msg)
  {}
  ::mesh_msgs::msg::MeshMaterials vertex_tex_coords(::mesh_msgs::msg::MeshMaterials::_vertex_tex_coords_type arg)
  {
    msg_.vertex_tex_coords = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::msg::MeshMaterials msg_;
};

class Init_MeshMaterials_cluster_materials
{
public:
  explicit Init_MeshMaterials_cluster_materials(::mesh_msgs::msg::MeshMaterials & msg)
  : msg_(msg)
  {}
  Init_MeshMaterials_vertex_tex_coords cluster_materials(::mesh_msgs::msg::MeshMaterials::_cluster_materials_type arg)
  {
    msg_.cluster_materials = std::move(arg);
    return Init_MeshMaterials_vertex_tex_coords(msg_);
  }

private:
  ::mesh_msgs::msg::MeshMaterials msg_;
};

class Init_MeshMaterials_materials
{
public:
  explicit Init_MeshMaterials_materials(::mesh_msgs::msg::MeshMaterials & msg)
  : msg_(msg)
  {}
  Init_MeshMaterials_cluster_materials materials(::mesh_msgs::msg::MeshMaterials::_materials_type arg)
  {
    msg_.materials = std::move(arg);
    return Init_MeshMaterials_cluster_materials(msg_);
  }

private:
  ::mesh_msgs::msg::MeshMaterials msg_;
};

class Init_MeshMaterials_clusters
{
public:
  Init_MeshMaterials_clusters()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MeshMaterials_materials clusters(::mesh_msgs::msg::MeshMaterials::_clusters_type arg)
  {
    msg_.clusters = std::move(arg);
    return Init_MeshMaterials_materials(msg_);
  }

private:
  ::mesh_msgs::msg::MeshMaterials msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::msg::MeshMaterials>()
{
  return mesh_msgs::msg::builder::Init_MeshMaterials_clusters();
}

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__MESH_MATERIALS__BUILDER_HPP_
