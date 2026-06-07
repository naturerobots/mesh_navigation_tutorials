// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mesh_msgs:msg/VectorFieldStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/vector_field_stamped.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__VECTOR_FIELD_STAMPED__STRUCT_HPP_
#define MESH_MSGS__MSG__DETAIL__VECTOR_FIELD_STAMPED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'vector_field'
#include "mesh_msgs/msg/detail/vector_field__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mesh_msgs__msg__VectorFieldStamped __attribute__((deprecated))
#else
# define DEPRECATED__mesh_msgs__msg__VectorFieldStamped __declspec(deprecated)
#endif

namespace mesh_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VectorFieldStamped_
{
  using Type = VectorFieldStamped_<ContainerAllocator>;

  explicit VectorFieldStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    vector_field(_init)
  {
    (void)_init;
  }

  explicit VectorFieldStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    vector_field(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _vector_field_type =
    mesh_msgs::msg::VectorField_<ContainerAllocator>;
  _vector_field_type vector_field;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__vector_field(
    const mesh_msgs::msg::VectorField_<ContainerAllocator> & _arg)
  {
    this->vector_field = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mesh_msgs::msg::VectorFieldStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const mesh_msgs::msg::VectorFieldStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mesh_msgs::msg::VectorFieldStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mesh_msgs::msg::VectorFieldStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::msg::VectorFieldStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::msg::VectorFieldStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::msg::VectorFieldStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::msg::VectorFieldStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mesh_msgs::msg::VectorFieldStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mesh_msgs::msg::VectorFieldStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mesh_msgs__msg__VectorFieldStamped
    std::shared_ptr<mesh_msgs::msg::VectorFieldStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mesh_msgs__msg__VectorFieldStamped
    std::shared_ptr<mesh_msgs::msg::VectorFieldStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VectorFieldStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->vector_field != other.vector_field) {
      return false;
    }
    return true;
  }
  bool operator!=(const VectorFieldStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VectorFieldStamped_

// alias to use template instance with default allocator
using VectorFieldStamped =
  mesh_msgs::msg::VectorFieldStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__VECTOR_FIELD_STAMPED__STRUCT_HPP_
