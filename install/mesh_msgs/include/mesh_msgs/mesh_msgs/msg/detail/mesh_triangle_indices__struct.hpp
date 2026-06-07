// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mesh_msgs:msg/MeshTriangleIndices.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_triangle_indices.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_TRIANGLE_INDICES__STRUCT_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_TRIANGLE_INDICES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mesh_msgs__msg__MeshTriangleIndices __attribute__((deprecated))
#else
# define DEPRECATED__mesh_msgs__msg__MeshTriangleIndices __declspec(deprecated)
#endif

namespace mesh_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MeshTriangleIndices_
{
  using Type = MeshTriangleIndices_<ContainerAllocator>;

  explicit MeshTriangleIndices_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint32_t, 3>::iterator, uint32_t>(this->vertex_indices.begin(), this->vertex_indices.end(), 0ul);
    }
  }

  explicit MeshTriangleIndices_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : vertex_indices(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<uint32_t, 3>::iterator, uint32_t>(this->vertex_indices.begin(), this->vertex_indices.end(), 0ul);
    }
  }

  // field types and members
  using _vertex_indices_type =
    std::array<uint32_t, 3>;
  _vertex_indices_type vertex_indices;

  // setters for named parameter idiom
  Type & set__vertex_indices(
    const std::array<uint32_t, 3> & _arg)
  {
    this->vertex_indices = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mesh_msgs::msg::MeshTriangleIndices_<ContainerAllocator> *;
  using ConstRawPtr =
    const mesh_msgs::msg::MeshTriangleIndices_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mesh_msgs::msg::MeshTriangleIndices_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mesh_msgs::msg::MeshTriangleIndices_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::msg::MeshTriangleIndices_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::msg::MeshTriangleIndices_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::msg::MeshTriangleIndices_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::msg::MeshTriangleIndices_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mesh_msgs::msg::MeshTriangleIndices_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mesh_msgs::msg::MeshTriangleIndices_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mesh_msgs__msg__MeshTriangleIndices
    std::shared_ptr<mesh_msgs::msg::MeshTriangleIndices_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mesh_msgs__msg__MeshTriangleIndices
    std::shared_ptr<mesh_msgs::msg::MeshTriangleIndices_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MeshTriangleIndices_ & other) const
  {
    if (this->vertex_indices != other.vertex_indices) {
      return false;
    }
    return true;
  }
  bool operator!=(const MeshTriangleIndices_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MeshTriangleIndices_

// alias to use template instance with default allocator
using MeshTriangleIndices =
  mesh_msgs::msg::MeshTriangleIndices_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__MESH_TRIANGLE_INDICES__STRUCT_HPP_
