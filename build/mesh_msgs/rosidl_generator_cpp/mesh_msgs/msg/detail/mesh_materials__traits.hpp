// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mesh_msgs:msg/MeshMaterials.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_materials.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_MATERIALS__TRAITS_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_MATERIALS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mesh_msgs/msg/detail/mesh_materials__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'clusters'
#include "mesh_msgs/msg/detail/mesh_face_cluster__traits.hpp"
// Member 'materials'
#include "mesh_msgs/msg/detail/mesh_material__traits.hpp"
// Member 'vertex_tex_coords'
#include "mesh_msgs/msg/detail/mesh_vertex_tex_coords__traits.hpp"

namespace mesh_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MeshMaterials & msg,
  std::ostream & out)
{
  out << "{";
  // member: clusters
  {
    if (msg.clusters.size() == 0) {
      out << "clusters: []";
    } else {
      out << "clusters: [";
      size_t pending_items = msg.clusters.size();
      for (auto item : msg.clusters) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: materials
  {
    if (msg.materials.size() == 0) {
      out << "materials: []";
    } else {
      out << "materials: [";
      size_t pending_items = msg.materials.size();
      for (auto item : msg.materials) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: cluster_materials
  {
    if (msg.cluster_materials.size() == 0) {
      out << "cluster_materials: []";
    } else {
      out << "cluster_materials: [";
      size_t pending_items = msg.cluster_materials.size();
      for (auto item : msg.cluster_materials) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: vertex_tex_coords
  {
    if (msg.vertex_tex_coords.size() == 0) {
      out << "vertex_tex_coords: []";
    } else {
      out << "vertex_tex_coords: [";
      size_t pending_items = msg.vertex_tex_coords.size();
      for (auto item : msg.vertex_tex_coords) {
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
  const MeshMaterials & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: clusters
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.clusters.size() == 0) {
      out << "clusters: []\n";
    } else {
      out << "clusters:\n";
      for (auto item : msg.clusters) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: materials
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.materials.size() == 0) {
      out << "materials: []\n";
    } else {
      out << "materials:\n";
      for (auto item : msg.materials) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: cluster_materials
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.cluster_materials.size() == 0) {
      out << "cluster_materials: []\n";
    } else {
      out << "cluster_materials:\n";
      for (auto item : msg.cluster_materials) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: vertex_tex_coords
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vertex_tex_coords.size() == 0) {
      out << "vertex_tex_coords: []\n";
    } else {
      out << "vertex_tex_coords:\n";
      for (auto item : msg.vertex_tex_coords) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MeshMaterials & msg, bool use_flow_style = false)
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
  const mesh_msgs::msg::MeshMaterials & msg,
  std::ostream & out, size_t indentation = 0)
{
  mesh_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mesh_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mesh_msgs::msg::MeshMaterials & msg)
{
  return mesh_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mesh_msgs::msg::MeshMaterials>()
{
  return "mesh_msgs::msg::MeshMaterials";
}

template<>
inline const char * name<mesh_msgs::msg::MeshMaterials>()
{
  return "mesh_msgs/msg/MeshMaterials";
}

template<>
struct has_fixed_size<mesh_msgs::msg::MeshMaterials>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mesh_msgs::msg::MeshMaterials>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mesh_msgs::msg::MeshMaterials>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESH_MSGS__MSG__DETAIL__MESH_MATERIALS__TRAITS_HPP_
