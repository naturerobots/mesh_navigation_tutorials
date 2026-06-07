// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mesh_msgs:srv/GetMaterials.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/srv/get_materials.hpp"


#ifndef MESH_MSGS__SRV__DETAIL__GET_MATERIALS__STRUCT_HPP_
#define MESH_MSGS__SRV__DETAIL__GET_MATERIALS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mesh_msgs__srv__GetMaterials_Request __attribute__((deprecated))
#else
# define DEPRECATED__mesh_msgs__srv__GetMaterials_Request __declspec(deprecated)
#endif

namespace mesh_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetMaterials_Request_
{
  using Type = GetMaterials_Request_<ContainerAllocator>;

  explicit GetMaterials_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->uuid = "";
    }
  }

  explicit GetMaterials_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : uuid(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->uuid = "";
    }
  }

  // field types and members
  using _uuid_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _uuid_type uuid;

  // setters for named parameter idiom
  Type & set__uuid(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->uuid = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mesh_msgs::srv::GetMaterials_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mesh_msgs::srv::GetMaterials_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mesh_msgs::srv::GetMaterials_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mesh_msgs::srv::GetMaterials_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::srv::GetMaterials_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::srv::GetMaterials_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::srv::GetMaterials_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::srv::GetMaterials_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mesh_msgs::srv::GetMaterials_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mesh_msgs::srv::GetMaterials_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mesh_msgs__srv__GetMaterials_Request
    std::shared_ptr<mesh_msgs::srv::GetMaterials_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mesh_msgs__srv__GetMaterials_Request
    std::shared_ptr<mesh_msgs::srv::GetMaterials_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetMaterials_Request_ & other) const
  {
    if (this->uuid != other.uuid) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetMaterials_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetMaterials_Request_

// alias to use template instance with default allocator
using GetMaterials_Request =
  mesh_msgs::srv::GetMaterials_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mesh_msgs


// Include directives for member types
// Member 'mesh_materials_stamped'
#include "mesh_msgs/msg/detail/mesh_materials_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mesh_msgs__srv__GetMaterials_Response __attribute__((deprecated))
#else
# define DEPRECATED__mesh_msgs__srv__GetMaterials_Response __declspec(deprecated)
#endif

namespace mesh_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetMaterials_Response_
{
  using Type = GetMaterials_Response_<ContainerAllocator>;

  explicit GetMaterials_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : mesh_materials_stamped(_init)
  {
    (void)_init;
  }

  explicit GetMaterials_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : mesh_materials_stamped(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _mesh_materials_stamped_type =
    mesh_msgs::msg::MeshMaterialsStamped_<ContainerAllocator>;
  _mesh_materials_stamped_type mesh_materials_stamped;

  // setters for named parameter idiom
  Type & set__mesh_materials_stamped(
    const mesh_msgs::msg::MeshMaterialsStamped_<ContainerAllocator> & _arg)
  {
    this->mesh_materials_stamped = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mesh_msgs::srv::GetMaterials_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mesh_msgs::srv::GetMaterials_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mesh_msgs::srv::GetMaterials_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mesh_msgs::srv::GetMaterials_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::srv::GetMaterials_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::srv::GetMaterials_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::srv::GetMaterials_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::srv::GetMaterials_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mesh_msgs::srv::GetMaterials_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mesh_msgs::srv::GetMaterials_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mesh_msgs__srv__GetMaterials_Response
    std::shared_ptr<mesh_msgs::srv::GetMaterials_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mesh_msgs__srv__GetMaterials_Response
    std::shared_ptr<mesh_msgs::srv::GetMaterials_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetMaterials_Response_ & other) const
  {
    if (this->mesh_materials_stamped != other.mesh_materials_stamped) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetMaterials_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetMaterials_Response_

// alias to use template instance with default allocator
using GetMaterials_Response =
  mesh_msgs::srv::GetMaterials_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mesh_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mesh_msgs__srv__GetMaterials_Event __attribute__((deprecated))
#else
# define DEPRECATED__mesh_msgs__srv__GetMaterials_Event __declspec(deprecated)
#endif

namespace mesh_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetMaterials_Event_
{
  using Type = GetMaterials_Event_<ContainerAllocator>;

  explicit GetMaterials_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit GetMaterials_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<mesh_msgs::srv::GetMaterials_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mesh_msgs::srv::GetMaterials_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<mesh_msgs::srv::GetMaterials_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mesh_msgs::srv::GetMaterials_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<mesh_msgs::srv::GetMaterials_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mesh_msgs::srv::GetMaterials_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<mesh_msgs::srv::GetMaterials_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mesh_msgs::srv::GetMaterials_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mesh_msgs::srv::GetMaterials_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const mesh_msgs::srv::GetMaterials_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mesh_msgs::srv::GetMaterials_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mesh_msgs::srv::GetMaterials_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::srv::GetMaterials_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::srv::GetMaterials_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::srv::GetMaterials_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::srv::GetMaterials_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mesh_msgs::srv::GetMaterials_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mesh_msgs::srv::GetMaterials_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mesh_msgs__srv__GetMaterials_Event
    std::shared_ptr<mesh_msgs::srv::GetMaterials_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mesh_msgs__srv__GetMaterials_Event
    std::shared_ptr<mesh_msgs::srv::GetMaterials_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetMaterials_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetMaterials_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetMaterials_Event_

// alias to use template instance with default allocator
using GetMaterials_Event =
  mesh_msgs::srv::GetMaterials_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mesh_msgs

namespace mesh_msgs
{

namespace srv
{

struct GetMaterials
{
  using Request = mesh_msgs::srv::GetMaterials_Request;
  using Response = mesh_msgs::srv::GetMaterials_Response;
  using Event = mesh_msgs::srv::GetMaterials_Event;
};

}  // namespace srv

}  // namespace mesh_msgs

#endif  // MESH_MSGS__SRV__DETAIL__GET_MATERIALS__STRUCT_HPP_
