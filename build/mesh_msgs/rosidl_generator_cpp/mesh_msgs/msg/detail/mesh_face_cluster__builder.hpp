// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mesh_msgs:msg/MeshFaceCluster.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_face_cluster.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_FACE_CLUSTER__BUILDER_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_FACE_CLUSTER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mesh_msgs/msg/detail/mesh_face_cluster__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mesh_msgs
{

namespace msg
{

namespace builder
{

class Init_MeshFaceCluster_label
{
public:
  explicit Init_MeshFaceCluster_label(::mesh_msgs::msg::MeshFaceCluster & msg)
  : msg_(msg)
  {}
  ::mesh_msgs::msg::MeshFaceCluster label(::mesh_msgs::msg::MeshFaceCluster::_label_type arg)
  {
    msg_.label = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::msg::MeshFaceCluster msg_;
};

class Init_MeshFaceCluster_face_indices
{
public:
  Init_MeshFaceCluster_face_indices()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MeshFaceCluster_label face_indices(::mesh_msgs::msg::MeshFaceCluster::_face_indices_type arg)
  {
    msg_.face_indices = std::move(arg);
    return Init_MeshFaceCluster_label(msg_);
  }

private:
  ::mesh_msgs::msg::MeshFaceCluster msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::msg::MeshFaceCluster>()
{
  return mesh_msgs::msg::builder::Init_MeshFaceCluster_face_indices();
}

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__MESH_FACE_CLUSTER__BUILDER_HPP_
