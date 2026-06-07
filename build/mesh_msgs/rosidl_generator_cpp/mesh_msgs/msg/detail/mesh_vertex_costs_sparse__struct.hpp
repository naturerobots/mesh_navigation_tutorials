// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mesh_msgs:msg/MeshVertexCostsSparse.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_vertex_costs_sparse.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS_SPARSE__STRUCT_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS_SPARSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mesh_msgs__msg__MeshVertexCostsSparse __attribute__((deprecated))
#else
# define DEPRECATED__mesh_msgs__msg__MeshVertexCostsSparse __declspec(deprecated)
#endif

namespace mesh_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MeshVertexCostsSparse_
{
  using Type = MeshVertexCostsSparse_<ContainerAllocator>;

  explicit MeshVertexCostsSparse_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->default_value = 0.0f;
    }
  }

  explicit MeshVertexCostsSparse_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->default_value = 0.0f;
    }
  }

  // field types and members
  using _vertices_type =
    std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>>;
  _vertices_type vertices;
  using _costs_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _costs_type costs;
  using _default_value_type =
    float;
  _default_value_type default_value;

  // setters for named parameter idiom
  Type & set__vertices(
    const std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>> & _arg)
  {
    this->vertices = _arg;
    return *this;
  }
  Type & set__costs(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->costs = _arg;
    return *this;
  }
  Type & set__default_value(
    const float & _arg)
  {
    this->default_value = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mesh_msgs::msg::MeshVertexCostsSparse_<ContainerAllocator> *;
  using ConstRawPtr =
    const mesh_msgs::msg::MeshVertexCostsSparse_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mesh_msgs::msg::MeshVertexCostsSparse_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mesh_msgs::msg::MeshVertexCostsSparse_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::msg::MeshVertexCostsSparse_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::msg::MeshVertexCostsSparse_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::msg::MeshVertexCostsSparse_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::msg::MeshVertexCostsSparse_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mesh_msgs::msg::MeshVertexCostsSparse_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mesh_msgs::msg::MeshVertexCostsSparse_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mesh_msgs__msg__MeshVertexCostsSparse
    std::shared_ptr<mesh_msgs::msg::MeshVertexCostsSparse_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mesh_msgs__msg__MeshVertexCostsSparse
    std::shared_ptr<mesh_msgs::msg::MeshVertexCostsSparse_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MeshVertexCostsSparse_ & other) const
  {
    if (this->vertices != other.vertices) {
      return false;
    }
    if (this->costs != other.costs) {
      return false;
    }
    if (this->default_value != other.default_value) {
      return false;
    }
    return true;
  }
  bool operator!=(const MeshVertexCostsSparse_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MeshVertexCostsSparse_

// alias to use template instance with default allocator
using MeshVertexCostsSparse =
  mesh_msgs::msg::MeshVertexCostsSparse_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__MESH_VERTEX_COSTS_SPARSE__STRUCT_HPP_
