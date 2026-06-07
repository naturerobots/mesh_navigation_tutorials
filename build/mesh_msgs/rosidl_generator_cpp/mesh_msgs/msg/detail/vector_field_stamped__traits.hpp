// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mesh_msgs:msg/VectorFieldStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/vector_field_stamped.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__VECTOR_FIELD_STAMPED__TRAITS_HPP_
#define MESH_MSGS__MSG__DETAIL__VECTOR_FIELD_STAMPED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mesh_msgs/msg/detail/vector_field_stamped__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'vector_field'
#include "mesh_msgs/msg/detail/vector_field__traits.hpp"

namespace mesh_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const VectorFieldStamped & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: vector_field
  {
    out << "vector_field: ";
    to_flow_style_yaml(msg.vector_field, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VectorFieldStamped & msg,
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

  // member: vector_field
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vector_field:\n";
    to_block_style_yaml(msg.vector_field, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VectorFieldStamped & msg, bool use_flow_style = false)
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
  const mesh_msgs::msg::VectorFieldStamped & msg,
  std::ostream & out, size_t indentation = 0)
{
  mesh_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mesh_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mesh_msgs::msg::VectorFieldStamped & msg)
{
  return mesh_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mesh_msgs::msg::VectorFieldStamped>()
{
  return "mesh_msgs::msg::VectorFieldStamped";
}

template<>
inline const char * name<mesh_msgs::msg::VectorFieldStamped>()
{
  return "mesh_msgs/msg/VectorFieldStamped";
}

template<>
struct has_fixed_size<mesh_msgs::msg::VectorFieldStamped>
  : std::integral_constant<bool, has_fixed_size<mesh_msgs::msg::VectorField>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mesh_msgs::msg::VectorFieldStamped>
  : std::integral_constant<bool, has_bounded_size<mesh_msgs::msg::VectorField>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mesh_msgs::msg::VectorFieldStamped>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESH_MSGS__MSG__DETAIL__VECTOR_FIELD_STAMPED__TRAITS_HPP_
