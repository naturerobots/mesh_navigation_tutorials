// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mesh_msgs:msg/MeshVertexTexCoords.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_vertex_tex_coords.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_VERTEX_TEX_COORDS__STRUCT_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_VERTEX_TEX_COORDS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mesh_msgs__msg__MeshVertexTexCoords __attribute__((deprecated))
#else
# define DEPRECATED__mesh_msgs__msg__MeshVertexTexCoords __declspec(deprecated)
#endif

namespace mesh_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MeshVertexTexCoords_
{
  using Type = MeshVertexTexCoords_<ContainerAllocator>;

  explicit MeshVertexTexCoords_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->u = 0.0f;
      this->v = 0.0f;
    }
  }

  explicit MeshVertexTexCoords_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->u = 0.0f;
      this->v = 0.0f;
    }
  }

  // field types and members
  using _u_type =
    float;
  _u_type u;
  using _v_type =
    float;
  _v_type v;

  // setters for named parameter idiom
  Type & set__u(
    const float & _arg)
  {
    this->u = _arg;
    return *this;
  }
  Type & set__v(
    const float & _arg)
  {
    this->v = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mesh_msgs::msg::MeshVertexTexCoords_<ContainerAllocator> *;
  using ConstRawPtr =
    const mesh_msgs::msg::MeshVertexTexCoords_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mesh_msgs::msg::MeshVertexTexCoords_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mesh_msgs::msg::MeshVertexTexCoords_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::msg::MeshVertexTexCoords_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::msg::MeshVertexTexCoords_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::msg::MeshVertexTexCoords_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::msg::MeshVertexTexCoords_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mesh_msgs::msg::MeshVertexTexCoords_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mesh_msgs::msg::MeshVertexTexCoords_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mesh_msgs__msg__MeshVertexTexCoords
    std::shared_ptr<mesh_msgs::msg::MeshVertexTexCoords_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mesh_msgs__msg__MeshVertexTexCoords
    std::shared_ptr<mesh_msgs::msg::MeshVertexTexCoords_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MeshVertexTexCoords_ & other) const
  {
    if (this->u != other.u) {
      return false;
    }
    if (this->v != other.v) {
      return false;
    }
    return true;
  }
  bool operator!=(const MeshVertexTexCoords_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MeshVertexTexCoords_

// alias to use template instance with default allocator
using MeshVertexTexCoords =
  mesh_msgs::msg::MeshVertexTexCoords_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__MESH_VERTEX_TEX_COORDS__STRUCT_HPP_
