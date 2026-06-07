// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mesh_msgs:msg/MeshVertexCosts.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_vertex_costs.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS__STRUCT_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mesh_msgs__msg__MeshVertexCosts __attribute__((deprecated))
#else
# define DEPRECATED__mesh_msgs__msg__MeshVertexCosts __declspec(deprecated)
#endif

namespace mesh_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MeshVertexCosts_
{
  using Type = MeshVertexCosts_<ContainerAllocator>;

  explicit MeshVertexCosts_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit MeshVertexCosts_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _costs_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _costs_type costs;

  // setters for named parameter idiom
  Type & set__costs(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->costs = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mesh_msgs::msg::MeshVertexCosts_<ContainerAllocator> *;
  using ConstRawPtr =
    const mesh_msgs::msg::MeshVertexCosts_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mesh_msgs::msg::MeshVertexCosts_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mesh_msgs::msg::MeshVertexCosts_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::msg::MeshVertexCosts_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::msg::MeshVertexCosts_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::msg::MeshVertexCosts_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::msg::MeshVertexCosts_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mesh_msgs::msg::MeshVertexCosts_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mesh_msgs::msg::MeshVertexCosts_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mesh_msgs__msg__MeshVertexCosts
    std::shared_ptr<mesh_msgs::msg::MeshVertexCosts_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mesh_msgs__msg__MeshVertexCosts
    std::shared_ptr<mesh_msgs::msg::MeshVertexCosts_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MeshVertexCosts_ & other) const
  {
    if (this->costs != other.costs) {
      return false;
    }
    return true;
  }
  bool operator!=(const MeshVertexCosts_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MeshVertexCosts_

// alias to use template instance with default allocator
using MeshVertexCosts =
  mesh_msgs::msg::MeshVertexCosts_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS__STRUCT_HPP_
