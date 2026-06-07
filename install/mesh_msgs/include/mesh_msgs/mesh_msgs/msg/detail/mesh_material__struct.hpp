// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mesh_msgs:msg/MeshMaterial.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_material.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_MATERIAL__STRUCT_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_MATERIAL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'color'
#include "std_msgs/msg/detail/color_rgba__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mesh_msgs__msg__MeshMaterial __attribute__((deprecated))
#else
# define DEPRECATED__mesh_msgs__msg__MeshMaterial __declspec(deprecated)
#endif

namespace mesh_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MeshMaterial_
{
  using Type = MeshMaterial_<ContainerAllocator>;

  explicit MeshMaterial_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : color(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->texture_index = 0ul;
      this->has_texture = false;
    }
  }

  explicit MeshMaterial_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : color(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->texture_index = 0ul;
      this->has_texture = false;
    }
  }

  // field types and members
  using _texture_index_type =
    uint32_t;
  _texture_index_type texture_index;
  using _color_type =
    std_msgs::msg::ColorRGBA_<ContainerAllocator>;
  _color_type color;
  using _has_texture_type =
    bool;
  _has_texture_type has_texture;

  // setters for named parameter idiom
  Type & set__texture_index(
    const uint32_t & _arg)
  {
    this->texture_index = _arg;
    return *this;
  }
  Type & set__color(
    const std_msgs::msg::ColorRGBA_<ContainerAllocator> & _arg)
  {
    this->color = _arg;
    return *this;
  }
  Type & set__has_texture(
    const bool & _arg)
  {
    this->has_texture = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mesh_msgs::msg::MeshMaterial_<ContainerAllocator> *;
  using ConstRawPtr =
    const mesh_msgs::msg::MeshMaterial_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mesh_msgs::msg::MeshMaterial_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mesh_msgs::msg::MeshMaterial_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::msg::MeshMaterial_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::msg::MeshMaterial_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::msg::MeshMaterial_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::msg::MeshMaterial_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mesh_msgs::msg::MeshMaterial_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mesh_msgs::msg::MeshMaterial_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mesh_msgs__msg__MeshMaterial
    std::shared_ptr<mesh_msgs::msg::MeshMaterial_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mesh_msgs__msg__MeshMaterial
    std::shared_ptr<mesh_msgs::msg::MeshMaterial_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MeshMaterial_ & other) const
  {
    if (this->texture_index != other.texture_index) {
      return false;
    }
    if (this->color != other.color) {
      return false;
    }
    if (this->has_texture != other.has_texture) {
      return false;
    }
    return true;
  }
  bool operator!=(const MeshMaterial_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MeshMaterial_

// alias to use template instance with default allocator
using MeshMaterial =
  mesh_msgs::msg::MeshMaterial_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__MESH_MATERIAL__STRUCT_HPP_
