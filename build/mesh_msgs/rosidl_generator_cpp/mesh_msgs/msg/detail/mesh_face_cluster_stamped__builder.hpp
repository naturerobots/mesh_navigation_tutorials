// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mesh_msgs:msg/MeshFaceClusterStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_face_cluster_stamped.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_FACE_CLUSTER_STAMPED__BUILDER_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_FACE_CLUSTER_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mesh_msgs/msg/detail/mesh_face_cluster_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mesh_msgs
{

namespace msg
{

namespace builder
{

class Init_MeshFaceClusterStamped_override
{
public:
  explicit Init_MeshFaceClusterStamped_override(::mesh_msgs::msg::MeshFaceClusterStamped & msg)
  : msg_(msg)
  {}
  ::mesh_msgs::msg::MeshFaceClusterStamped override(::mesh_msgs::msg::MeshFaceClusterStamped::_override_type arg)
  {
    msg_.override = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::msg::MeshFaceClusterStamped msg_;
};

class Init_MeshFaceClusterStamped_cluster
{
public:
  explicit Init_MeshFaceClusterStamped_cluster(::mesh_msgs::msg::MeshFaceClusterStamped & msg)
  : msg_(msg)
  {}
  Init_MeshFaceClusterStamped_override cluster(::mesh_msgs::msg::MeshFaceClusterStamped::_cluster_type arg)
  {
    msg_.cluster = std::move(arg);
    return Init_MeshFaceClusterStamped_override(msg_);
  }

private:
  ::mesh_msgs::msg::MeshFaceClusterStamped msg_;
};

class Init_MeshFaceClusterStamped_uuid
{
public:
  explicit Init_MeshFaceClusterStamped_uuid(::mesh_msgs::msg::MeshFaceClusterStamped & msg)
  : msg_(msg)
  {}
  Init_MeshFaceClusterStamped_cluster uuid(::mesh_msgs::msg::MeshFaceClusterStamped::_uuid_type arg)
  {
    msg_.uuid = std::move(arg);
    return Init_MeshFaceClusterStamped_cluster(msg_);
  }

private:
  ::mesh_msgs::msg::MeshFaceClusterStamped msg_;
};

class Init_MeshFaceClusterStamped_header
{
public:
  Init_MeshFaceClusterStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MeshFaceClusterStamped_uuid header(::mesh_msgs::msg::MeshFaceClusterStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MeshFaceClusterStamped_uuid(msg_);
  }

private:
  ::mesh_msgs::msg::MeshFaceClusterStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::msg::MeshFaceClusterStamped>()
{
  return mesh_msgs::msg::builder::Init_MeshFaceClusterStamped_header();
}

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__MESH_FACE_CLUSTER_STAMPED__BUILDER_HPP_
