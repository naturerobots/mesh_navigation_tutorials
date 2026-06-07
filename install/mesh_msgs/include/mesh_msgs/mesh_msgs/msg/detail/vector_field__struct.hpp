// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mesh_msgs:msg/VectorField.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/vector_field.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__VECTOR_FIELD__STRUCT_HPP_
#define MESH_MSGS__MSG__DETAIL__VECTOR_FIELD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'positions'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'vectors'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mesh_msgs__msg__VectorField __attribute__((deprecated))
#else
# define DEPRECATED__mesh_msgs__msg__VectorField __declspec(deprecated)
#endif

namespace mesh_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VectorField_
{
  using Type = VectorField_<ContainerAllocator>;

  explicit VectorField_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit VectorField_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _positions_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>>;
  _positions_type positions;
  using _vectors_type =
    std::vector<geometry_msgs::msg::Vector3_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Vector3_<ContainerAllocator>>>;
  _vectors_type vectors;

  // setters for named parameter idiom
  Type & set__positions(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->positions = _arg;
    return *this;
  }
  Type & set__vectors(
    const std::vector<geometry_msgs::msg::Vector3_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Vector3_<ContainerAllocator>>> & _arg)
  {
    this->vectors = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mesh_msgs::msg::VectorField_<ContainerAllocator> *;
  using ConstRawPtr =
    const mesh_msgs::msg::VectorField_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mesh_msgs::msg::VectorField_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mesh_msgs::msg::VectorField_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::msg::VectorField_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::msg::VectorField_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::msg::VectorField_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::msg::VectorField_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mesh_msgs::msg::VectorField_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mesh_msgs::msg::VectorField_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mesh_msgs__msg__VectorField
    std::shared_ptr<mesh_msgs::msg::VectorField_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mesh_msgs__msg__VectorField
    std::shared_ptr<mesh_msgs::msg::VectorField_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VectorField_ & other) const
  {
    if (this->positions != other.positions) {
      return false;
    }
    if (this->vectors != other.vectors) {
      return false;
    }
    return true;
  }
  bool operator!=(const VectorField_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VectorField_

// alias to use template instance with default allocator
using VectorField =
  mesh_msgs::msg::VectorField_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__VECTOR_FIELD__STRUCT_HPP_
