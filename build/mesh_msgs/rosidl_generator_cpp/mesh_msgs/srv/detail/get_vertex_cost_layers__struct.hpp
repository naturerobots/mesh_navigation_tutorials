// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mesh_msgs:srv/GetVertexCostLayers.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/srv/get_vertex_cost_layers.hpp"


#ifndef MESH_MSGS__SRV__DETAIL__GET_VERTEX_COST_LAYERS__STRUCT_HPP_
#define MESH_MSGS__SRV__DETAIL__GET_VERTEX_COST_LAYERS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mesh_msgs__srv__GetVertexCostLayers_Request __attribute__((deprecated))
#else
# define DEPRECATED__mesh_msgs__srv__GetVertexCostLayers_Request __declspec(deprecated)
#endif

namespace mesh_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetVertexCostLayers_Request_
{
  using Type = GetVertexCostLayers_Request_<ContainerAllocator>;

  explicit GetVertexCostLayers_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->uuid = "";
    }
  }

  explicit GetVertexCostLayers_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    mesh_msgs::srv::GetVertexCostLayers_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mesh_msgs::srv::GetVertexCostLayers_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mesh_msgs::srv::GetVertexCostLayers_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mesh_msgs::srv::GetVertexCostLayers_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::srv::GetVertexCostLayers_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::srv::GetVertexCostLayers_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::srv::GetVertexCostLayers_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::srv::GetVertexCostLayers_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mesh_msgs::srv::GetVertexCostLayers_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mesh_msgs::srv::GetVertexCostLayers_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mesh_msgs__srv__GetVertexCostLayers_Request
    std::shared_ptr<mesh_msgs::srv::GetVertexCostLayers_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mesh_msgs__srv__GetVertexCostLayers_Request
    std::shared_ptr<mesh_msgs::srv::GetVertexCostLayers_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetVertexCostLayers_Request_ & other) const
  {
    if (this->uuid != other.uuid) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetVertexCostLayers_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetVertexCostLayers_Request_

// alias to use template instance with default allocator
using GetVertexCostLayers_Request =
  mesh_msgs::srv::GetVertexCostLayers_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mesh_msgs


#ifndef _WIN32
# define DEPRECATED__mesh_msgs__srv__GetVertexCostLayers_Response __attribute__((deprecated))
#else
# define DEPRECATED__mesh_msgs__srv__GetVertexCostLayers_Response __declspec(deprecated)
#endif

namespace mesh_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetVertexCostLayers_Response_
{
  using Type = GetVertexCostLayers_Response_<ContainerAllocator>;

  explicit GetVertexCostLayers_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit GetVertexCostLayers_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _layers_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _layers_type layers;

  // setters for named parameter idiom
  Type & set__layers(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->layers = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mesh_msgs::srv::GetVertexCostLayers_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mesh_msgs::srv::GetVertexCostLayers_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mesh_msgs::srv::GetVertexCostLayers_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mesh_msgs::srv::GetVertexCostLayers_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::srv::GetVertexCostLayers_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::srv::GetVertexCostLayers_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::srv::GetVertexCostLayers_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::srv::GetVertexCostLayers_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mesh_msgs::srv::GetVertexCostLayers_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mesh_msgs::srv::GetVertexCostLayers_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mesh_msgs__srv__GetVertexCostLayers_Response
    std::shared_ptr<mesh_msgs::srv::GetVertexCostLayers_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mesh_msgs__srv__GetVertexCostLayers_Response
    std::shared_ptr<mesh_msgs::srv::GetVertexCostLayers_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetVertexCostLayers_Response_ & other) const
  {
    if (this->layers != other.layers) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetVertexCostLayers_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetVertexCostLayers_Response_

// alias to use template instance with default allocator
using GetVertexCostLayers_Response =
  mesh_msgs::srv::GetVertexCostLayers_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mesh_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mesh_msgs__srv__GetVertexCostLayers_Event __attribute__((deprecated))
#else
# define DEPRECATED__mesh_msgs__srv__GetVertexCostLayers_Event __declspec(deprecated)
#endif

namespace mesh_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetVertexCostLayers_Event_
{
  using Type = GetVertexCostLayers_Event_<ContainerAllocator>;

  explicit GetVertexCostLayers_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit GetVertexCostLayers_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<mesh_msgs::srv::GetVertexCostLayers_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mesh_msgs::srv::GetVertexCostLayers_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<mesh_msgs::srv::GetVertexCostLayers_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mesh_msgs::srv::GetVertexCostLayers_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<mesh_msgs::srv::GetVertexCostLayers_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mesh_msgs::srv::GetVertexCostLayers_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<mesh_msgs::srv::GetVertexCostLayers_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mesh_msgs::srv::GetVertexCostLayers_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mesh_msgs::srv::GetVertexCostLayers_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const mesh_msgs::srv::GetVertexCostLayers_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mesh_msgs::srv::GetVertexCostLayers_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mesh_msgs::srv::GetVertexCostLayers_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::srv::GetVertexCostLayers_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::srv::GetVertexCostLayers_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mesh_msgs::srv::GetVertexCostLayers_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mesh_msgs::srv::GetVertexCostLayers_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mesh_msgs::srv::GetVertexCostLayers_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mesh_msgs::srv::GetVertexCostLayers_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mesh_msgs__srv__GetVertexCostLayers_Event
    std::shared_ptr<mesh_msgs::srv::GetVertexCostLayers_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mesh_msgs__srv__GetVertexCostLayers_Event
    std::shared_ptr<mesh_msgs::srv::GetVertexCostLayers_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetVertexCostLayers_Event_ & other) const
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
  bool operator!=(const GetVertexCostLayers_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetVertexCostLayers_Event_

// alias to use template instance with default allocator
using GetVertexCostLayers_Event =
  mesh_msgs::srv::GetVertexCostLayers_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mesh_msgs

namespace mesh_msgs
{

namespace srv
{

struct GetVertexCostLayers
{
  using Request = mesh_msgs::srv::GetVertexCostLayers_Request;
  using Response = mesh_msgs::srv::GetVertexCostLayers_Response;
  using Event = mesh_msgs::srv::GetVertexCostLayers_Event;
};

}  // namespace srv

}  // namespace mesh_msgs

#endif  // MESH_MSGS__SRV__DETAIL__GET_VERTEX_COST_LAYERS__STRUCT_HPP_
