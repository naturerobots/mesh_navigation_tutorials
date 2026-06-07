// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mesh_msgs:msg/MeshMaterials.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_materials.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_MATERIALS__STRUCT_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_MATERIALS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'clusters'
#include "mesh_msgs/msg/detail/mesh_face_cluster__struct.hpp"
// Member 'materials'
#include "mesh_msgs/msg/detail/mesh_material__struct.hpp"
// Member 'vertex_tex_coords'
#include "mesh_msgs/msg/detail/mesh_vertex_tex_coords__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mesh_msgs__msg__MeshMaterials __attribute__((deprecated))
#else
# define DEPRECATED__mesh_msgs__msg__MeshMaterials __declspec(deprecated)
#endif

namespace mesh_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MeshMaterials_
{
  using Type = MeshMaterials_<ContainerAllocator>;

  explicit MeshMaterials_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit MeshMaterials_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _clusters_type =
    std::vector<mesh_msgs::msg::MeshFaceCluster_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mesh_msgs::msg::MeshFaceCluster_<ContainerAllocator>>>;
  _clusters_type clusters;
  using _materials_type =
    std::vector<mesh_msgs::msg::MeshMaterial_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mesh_msgs::msg::MeshMaterial_<ContainerAllocator>>>;
  _materials_type materials;
  using _cluster_materials_type =
    std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>>;
  _cluster_materials_type cluster_materials;
  using _vertex_tex_coords_type =
    std::vector<mesh_msgs::msg::MeshVertexTexCoords_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mesh_msgs::msg::MeshVertexTexCoords_<ContainerAllocator>>>;
  _vertex_tex_coords_type vertex_tex_coords;

  // setters for named parameter idiom
  Type & set__clusters(
    const std::vector<mesh_msgs::msg::MeshFaceCluster_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mesh_msgs::msg::MeshFaceCluster_<ContainerAllocator>>> & _arg)
  {
    this->clusters = _arg;
    return *this;
  }
  Type & set__materials(
    const std::vector<mesh_msgs::msg::MeshMaterial_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mesh_msgs::msg::MeshMaterial_<ContainerAllocator>>> & _arg)
  {
    this->materials = _arg;
    return *this;
  }
  Type & set__cluster_materials(
    const std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>> & _arg)
  {
    this->cluster_materials = _arg;
    return *this;
  }
  Type & set__vertex_tex_coords(
    const std::vector<mesh_msgs::msg::MeshVertexTexCoords_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mesh_msgs::msg::MeshVertexTexCoords_<ContainerAllocator>>> & _arg)
  {
    this->vertex_tex_coords = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mesh_msgs::msg::MeshMaterials_<ContainerAllocator> *;
  using ConstRawPtr =
    const mesh_msgs::msg::MeshMaterials_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mesh_msgs::msg::MeshMaterials_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mesh_msgs::msg::MeshMaterials_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::msg::MeshMaterials_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::msg::MeshMaterials_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::msg::MeshMaterials_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::msg::MeshMaterials_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mesh_msgs::msg::MeshMaterials_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mesh_msgs::msg::MeshMaterials_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mesh_msgs__msg__MeshMaterials
    std::shared_ptr<mesh_msgs::msg::MeshMaterials_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mesh_msgs__msg__MeshMaterials
    std::shared_ptr<mesh_msgs::msg::MeshMaterials_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MeshMaterials_ & other) const
  {
    if (this->clusters != other.clusters) {
      return false;
    }
    if (this->materials != other.materials) {
      return false;
    }
    if (this->cluster_materials != other.cluster_materials) {
      return false;
    }
    if (this->vertex_tex_coords != other.vertex_tex_coords) {
      return false;
    }
    return true;
  }
  bool operator!=(const MeshMaterials_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MeshMaterials_

// alias to use template instance with default allocator
using MeshMaterials =
  mesh_msgs::msg::MeshMaterials_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__MESH_MATERIALS__STRUCT_HPP_
