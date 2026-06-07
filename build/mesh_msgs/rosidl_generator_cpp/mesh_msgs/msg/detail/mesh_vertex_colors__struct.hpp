// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mesh_msgs:msg/MeshVertexColors.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_vertex_colors.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COLORS__STRUCT_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COLORS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'vertex_colors'
#include "std_msgs/msg/detail/color_rgba__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mesh_msgs__msg__MeshVertexColors __attribute__((deprecated))
#else
# define DEPRECATED__mesh_msgs__msg__MeshVertexColors __declspec(deprecated)
#endif

namespace mesh_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MeshVertexColors_
{
  using Type = MeshVertexColors_<ContainerAllocator>;

  explicit MeshVertexColors_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit MeshVertexColors_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _vertex_colors_type =
    std::vector<std_msgs::msg::ColorRGBA_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std_msgs::msg::ColorRGBA_<ContainerAllocator>>>;
  _vertex_colors_type vertex_colors;

  // setters for named parameter idiom
  Type & set__vertex_colors(
    const std::vector<std_msgs::msg::ColorRGBA_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std_msgs::msg::ColorRGBA_<ContainerAllocator>>> & _arg)
  {
    this->vertex_colors = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mesh_msgs::msg::MeshVertexColors_<ContainerAllocator> *;
  using ConstRawPtr =
    const mesh_msgs::msg::MeshVertexColors_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mesh_msgs::msg::MeshVertexColors_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mesh_msgs::msg::MeshVertexColors_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::msg::MeshVertexColors_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::msg::MeshVertexColors_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::msg::MeshVertexColors_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::msg::MeshVertexColors_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mesh_msgs::msg::MeshVertexColors_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mesh_msgs::msg::MeshVertexColors_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mesh_msgs__msg__MeshVertexColors
    std::shared_ptr<mesh_msgs::msg::MeshVertexColors_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mesh_msgs__msg__MeshVertexColors
    std::shared_ptr<mesh_msgs::msg::MeshVertexColors_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MeshVertexColors_ & other) const
  {
    if (this->vertex_colors != other.vertex_colors) {
      return false;
    }
    return true;
  }
  bool operator!=(const MeshVertexColors_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MeshVertexColors_

// alias to use template instance with default allocator
using MeshVertexColors =
  mesh_msgs::msg::MeshVertexColors_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COLORS__STRUCT_HPP_
