// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mesh_msgs:msg/MeshFaceClusterStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_face_cluster_stamped.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_FACE_CLUSTER_STAMPED__TRAITS_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_FACE_CLUSTER_STAMPED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mesh_msgs/msg/detail/mesh_face_cluster_stamped__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'cluster'
#include "mesh_msgs/msg/detail/mesh_face_cluster__traits.hpp"

namespace mesh_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MeshFaceClusterStamped & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: uuid
  {
    out << "uuid: ";
    rosidl_generator_traits::value_to_yaml(msg.uuid, out);
    out << ", ";
  }

  // member: cluster
  {
    out << "cluster: ";
    to_flow_style_yaml(msg.cluster, out);
    out << ", ";
  }

  // member: override
  {
    out << "override: ";
    rosidl_generator_traits::value_to_yaml(msg.override, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MeshFaceClusterStamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: uuid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "uuid: ";
    rosidl_generator_traits::value_to_yaml(msg.uuid, out);
    out << "\n";
  }

  // member: cluster
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cluster:\n";
    to_block_style_yaml(msg.cluster, out, indentation + 2);
  }

  // member: override
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "override: ";
    rosidl_generator_traits::value_to_yaml(msg.override, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MeshFaceClusterStamped & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace mesh_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mesh_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mesh_msgs::msg::MeshFaceClusterStamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  mesh_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mesh_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mesh_msgs::msg::MeshFaceClusterStamped & msg)
{
  return mesh_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mesh_msgs::msg::MeshFaceClusterStamped>()
{
  return "mesh_msgs::msg::MeshFaceClusterStamped";
}

template<>
inline const char * name<mesh_msgs::msg::MeshFaceClusterStamped>()
{
  return "mesh_msgs/msg/MeshFaceClusterStamped";
}

template<>
struct has_fixed_size<mesh_msgs::msg::MeshFaceClusterStamped>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mesh_msgs::msg::MeshFaceClusterStamped>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mesh_msgs::msg::MeshFaceClusterStamped>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESH_MSGS__MSG__DETAIL__MESH_FACE_CLUSTER_STAMPED__TRAITS_HPP_
