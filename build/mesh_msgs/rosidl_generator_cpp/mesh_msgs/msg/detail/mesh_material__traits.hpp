// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mesh_msgs:msg/MeshMaterial.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_material.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_MATERIAL__TRAITS_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_MATERIAL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mesh_msgs/msg/detail/mesh_material__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'color'
#include "std_msgs/msg/detail/color_rgba__traits.hpp"

namespace mesh_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MeshMaterial & msg,
  std::ostream & out)
{
  out << "{";
  // member: texture_index
  {
    out << "texture_index: ";
    rosidl_generator_traits::value_to_yaml(msg.texture_index, out);
    out << ", ";
  }

  // member: color
  {
    out << "color: ";
    to_flow_style_yaml(msg.color, out);
    out << ", ";
  }

  // member: has_texture
  {
    out << "has_texture: ";
    rosidl_generator_traits::value_to_yaml(msg.has_texture, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MeshMaterial & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: texture_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "texture_index: ";
    rosidl_generator_traits::value_to_yaml(msg.texture_index, out);
    out << "\n";
  }

  // member: color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color:\n";
    to_block_style_yaml(msg.color, out, indentation + 2);
  }

  // member: has_texture
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "has_texture: ";
    rosidl_generator_traits::value_to_yaml(msg.has_texture, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MeshMaterial & msg, bool use_flow_style = false)
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
  const mesh_msgs::msg::MeshMaterial & msg,
  std::ostream & out, size_t indentation = 0)
{
  mesh_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mesh_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mesh_msgs::msg::MeshMaterial & msg)
{
  return mesh_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mesh_msgs::msg::MeshMaterial>()
{
  return "mesh_msgs::msg::MeshMaterial";
}

template<>
inline const char * name<mesh_msgs::msg::MeshMaterial>()
{
  return "mesh_msgs/msg/MeshMaterial";
}

template<>
struct has_fixed_size<mesh_msgs::msg::MeshMaterial>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::ColorRGBA>::value> {};

template<>
struct has_bounded_size<mesh_msgs::msg::MeshMaterial>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::ColorRGBA>::value> {};

template<>
struct is_message<mesh_msgs::msg::MeshMaterial>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESH_MSGS__MSG__DETAIL__MESH_MATERIAL__TRAITS_HPP_
