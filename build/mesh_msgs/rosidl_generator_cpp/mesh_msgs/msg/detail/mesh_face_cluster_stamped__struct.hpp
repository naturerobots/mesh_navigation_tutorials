// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mesh_msgs:msg/MeshFaceClusterStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_face_cluster_stamped.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_FACE_CLUSTER_STAMPED__STRUCT_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_FACE_CLUSTER_STAMPED__STRUCT_HPP_

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
// Member 'cluster'
#include "mesh_msgs/msg/detail/mesh_face_cluster__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mesh_msgs__msg__MeshFaceClusterStamped __attribute__((deprecated))
#else
# define DEPRECATED__mesh_msgs__msg__MeshFaceClusterStamped __declspec(deprecated)
#endif

namespace mesh_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MeshFaceClusterStamped_
{
  using Type = MeshFaceClusterStamped_<ContainerAllocator>;

  explicit MeshFaceClusterStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    cluster(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->uuid = "";
      this->override = false;
    }
  }

  explicit MeshFaceClusterStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    uuid(_alloc),
    cluster(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->uuid = "";
      this->override = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _uuid_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _uuid_type uuid;
  using _cluster_type =
    mesh_msgs::msg::MeshFaceCluster_<ContainerAllocator>;
  _cluster_type cluster;
  using _override_type =
    bool;
  _override_type override;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__uuid(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->uuid = _arg;
    return *this;
  }
  Type & set__cluster(
    const mesh_msgs::msg::MeshFaceCluster_<ContainerAllocator> & _arg)
  {
    this->cluster = _arg;
    return *this;
  }
  Type & set__override(
    const bool & _arg)
  {
    this->override = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mesh_msgs::msg::MeshFaceClusterStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const mesh_msgs::msg::MeshFaceClusterStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mesh_msgs::msg::MeshFaceClusterStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mesh_msgs::msg::MeshFaceClusterStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::msg::MeshFaceClusterStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::msg::MeshFaceClusterStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::msg::MeshFaceClusterStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::msg::MeshFaceClusterStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mesh_msgs::msg::MeshFaceClusterStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mesh_msgs::msg::MeshFaceClusterStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mesh_msgs__msg__MeshFaceClusterStamped
    std::shared_ptr<mesh_msgs::msg::MeshFaceClusterStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mesh_msgs__msg__MeshFaceClusterStamped
    std::shared_ptr<mesh_msgs::msg::MeshFaceClusterStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MeshFaceClusterStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->uuid != other.uuid) {
      return false;
    }
    if (this->cluster != other.cluster) {
      return false;
    }
    if (this->override != other.override) {
      return false;
    }
    return true;
  }
  bool operator!=(const MeshFaceClusterStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MeshFaceClusterStamped_

// alias to use template instance with default allocator
using MeshFaceClusterStamped =
  mesh_msgs::msg::MeshFaceClusterStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__MESH_FACE_CLUSTER_STAMPED__STRUCT_HPP_
