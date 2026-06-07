// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mesh_msgs:msg/MeshGeometry.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_geometry.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_GEOMETRY__TRAITS_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_GEOMETRY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mesh_msgs/msg/detail/mesh_geometry__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'vertices'
// Member 'vertex_normals'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'faces'
#include "mesh_msgs/msg/detail/mesh_triangle_indices__traits.hpp"

namespace mesh_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MeshGeometry & msg,
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
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: vertex_normals
  {
    if (msg.vertex_normals.size() == 0) {
      out << "vertex_normals: []";
    } else {
      out << "vertex_normals: [";
      size_t pending_items = msg.vertex_normals.size();
      for (auto item : msg.vertex_normals) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: faces
  {
    if (msg.faces.size() == 0) {
      out << "faces: []";
    } else {
      out << "faces: [";
      size_t pending_items = msg.faces.size();
      for (auto item : msg.faces) {
        to_flow_style_yaml(item, out);
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
  const MeshGeometry & msg,
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
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: vertex_normals
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vertex_normals.size() == 0) {
      out << "vertex_normals: []\n";
    } else {
      out << "vertex_normals:\n";
      for (auto item : msg.vertex_normals) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: faces
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.faces.size() == 0) {
      out << "faces: []\n";
    } else {
      out << "faces:\n";
      for (auto item : msg.faces) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MeshGeometry & msg, bool use_flow_style = false)
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
  const mesh_msgs::msg::MeshGeometry & msg,
  std::ostream & out, size_t indentation = 0)
{
  mesh_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mesh_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mesh_msgs::msg::MeshGeometry & msg)
{
  return mesh_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mesh_msgs::msg::MeshGeometry>()
{
  return "mesh_msgs::msg::MeshGeometry";
}

template<>
inline const char * name<mesh_msgs::msg::MeshGeometry>()
{
  return "mesh_msgs/msg/MeshGeometry";
}

template<>
struct has_fixed_size<mesh_msgs::msg::MeshGeometry>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mesh_msgs::msg::MeshGeometry>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mesh_msgs::msg::MeshGeometry>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESH_MSGS__MSG__DETAIL__MESH_GEOMETRY__TRAITS_HPP_
