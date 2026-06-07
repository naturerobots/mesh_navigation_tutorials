// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mesh_msgs:msg/MeshVertexCostsSparse.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_vertex_costs_sparse.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS_SPARSE__TRAITS_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS_SPARSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mesh_msgs/msg/detail/mesh_vertex_costs_sparse__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mesh_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MeshVertexCostsSparse & msg,
  std::ostream & out)
{
  out << "{";
  // member: vertices
  {
    if (msg.vertices.size() == 0) {
      out << "vertices: []";
    } else {
      out << "vertices: [";
      size_t pending_items = msg.vertices.size();
      for (auto item : msg.vertices) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: costs
  {
    if (msg.costs.size() == 0) {
      out << "costs: []";
    } else {
      out << "costs: [";
      size_t pending_items = msg.costs.size();
      for (auto item : msg.costs) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: default_value
  {
    out << "default_value: ";
    rosidl_generator_traits::value_to_yaml(msg.default_value, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MeshVertexCostsSparse & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: vertices
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vertices.size() == 0) {
      out << "vertices: []\n";
    } else {
      out << "vertices:\n";
      for (auto item : msg.vertices) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: costs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.costs.size() == 0) {
      out << "costs: []\n";
    } else {
      out << "costs:\n";
      for (auto item : msg.costs) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: default_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "default_value: ";
    rosidl_generator_traits::value_to_yaml(msg.default_value, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MeshVertexCostsSparse & msg, bool use_flow_style = false)
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
  const mesh_msgs::msg::MeshVertexCostsSparse & msg,
  std::ostream & out, size_t indentation = 0)
{
  mesh_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mesh_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mesh_msgs::msg::MeshVertexCostsSparse & msg)
{
  return mesh_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mesh_msgs::msg::MeshVertexCostsSparse>()
{
  return "mesh_msgs::msg::MeshVertexCostsSparse";
}

template<>
inline const char * name<mesh_msgs::msg::MeshVertexCostsSparse>()
{
  return "mesh_msgs/msg/MeshVertexCostsSparse";
}

template<>
struct has_fixed_size<mesh_msgs::msg::MeshVertexCostsSparse>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mesh_msgs::msg::MeshVertexCostsSparse>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mesh_msgs::msg::MeshVertexCostsSparse>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS_SPARSE__TRAITS_HPP_
