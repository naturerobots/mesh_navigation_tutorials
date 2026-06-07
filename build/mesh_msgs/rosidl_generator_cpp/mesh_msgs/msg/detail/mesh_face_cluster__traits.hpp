// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mesh_msgs:msg/MeshFaceCluster.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_face_cluster.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_FACE_CLUSTER__TRAITS_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_FACE_CLUSTER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mesh_msgs/msg/detail/mesh_face_cluster__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mesh_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MeshFaceCluster & msg,
  std::ostream & out)
{
  out << "{";
  // member: face_indices
  {
    if (msg.face_indices.size() == 0) {
      out << "face_indices: []";
    } else {
      out << "face_indices: [";
      size_t pending_items = msg.face_indices.size();
      for (auto item : msg.face_indices) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: label
  {
    out << "label: ";
    rosidl_generator_traits::value_to_yaml(msg.label, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MeshFaceCluster & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: face_indices
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.face_indices.size() == 0) {
      out << "face_indices: []\n";
    } else {
      out << "face_indices:\n";
      for (auto item : msg.face_indices) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: label
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "label: ";
    rosidl_generator_traits::value_to_yaml(msg.label, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MeshFaceCluster & msg, bool use_flow_style = false)
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
  const mesh_msgs::msg::MeshFaceCluster & msg,
  std::ostream & out, size_t indentation = 0)
{
  mesh_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mesh_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mesh_msgs::msg::MeshFaceCluster & msg)
{
  return mesh_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mesh_msgs::msg::MeshFaceCluster>()
{
  return "mesh_msgs::msg::MeshFaceCluster";
}

template<>
inline const char * name<mesh_msgs::msg::MeshFaceCluster>()
{
  return "mesh_msgs/msg/MeshFaceCluster";
}

template<>
struct has_fixed_size<mesh_msgs::msg::MeshFaceCluster>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mesh_msgs::msg::MeshFaceCluster>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mesh_msgs::msg::MeshFaceCluster>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESH_MSGS__MSG__DETAIL__MESH_FACE_CLUSTER__TRAITS_HPP_
