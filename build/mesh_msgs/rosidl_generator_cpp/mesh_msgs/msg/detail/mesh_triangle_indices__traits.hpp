// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mesh_msgs:msg/MeshTriangleIndices.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_triangle_indices.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_TRIANGLE_INDICES__TRAITS_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_TRIANGLE_INDICES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mesh_msgs/msg/detail/mesh_triangle_indices__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mesh_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MeshTriangleIndices & msg,
  std::ostream & out)
{
  out << "{";
  // member: vertex_indices
  {
    if (msg.vertex_indices.size() == 0) {
      out << "vertex_indices: []";
    } else {
      out << "vertex_indices: [";
      size_t pending_items = msg.vertex_indices.size();
      for (auto item : msg.vertex_indices) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MeshTriangleIndices & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: vertex_indices
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vertex_indices.size() == 0) {
      out << "vertex_indices: []\n";
    } else {
      out << "vertex_indices:\n";
      for (auto item : msg.vertex_indices) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MeshTriangleIndices & msg, bool use_flow_style = false)
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
  const mesh_msgs::msg::MeshTriangleIndices & msg,
  std::ostream & out, size_t indentation = 0)
{
  mesh_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mesh_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mesh_msgs::msg::MeshTriangleIndices & msg)
{
  return mesh_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mesh_msgs::msg::MeshTriangleIndices>()
{
  return "mesh_msgs::msg::MeshTriangleIndices";
}

template<>
inline const char * name<mesh_msgs::msg::MeshTriangleIndices>()
{
  return "mesh_msgs/msg/MeshTriangleIndices";
}

template<>
struct has_fixed_size<mesh_msgs::msg::MeshTriangleIndices>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mesh_msgs::msg::MeshTriangleIndices>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mesh_msgs::msg::MeshTriangleIndices>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESH_MSGS__MSG__DETAIL__MESH_TRIANGLE_INDICES__TRAITS_HPP_
