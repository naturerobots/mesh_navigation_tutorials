// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mbf_msgs:srv/CheckPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/srv/check_point.hpp"


#ifndef MBF_MSGS__SRV__DETAIL__CHECK_POINT__STRUCT_HPP_
#define MBF_MSGS__SRV__DETAIL__CHECK_POINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'point'
#include "geometry_msgs/msg/detail/point_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__srv__CheckPoint_Request __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__srv__CheckPoint_Request __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CheckPoint_Request_
{
  using Type = CheckPoint_Request_<ContainerAllocator>;

  explicit CheckPoint_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : point(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->costmap = 0;
    }
  }

  explicit CheckPoint_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : point(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->costmap = 0;
    }
  }

  // field types and members
  using _point_type =
    geometry_msgs::msg::PointStamped_<ContainerAllocator>;
  _point_type point;
  using _costmap_type =
    uint8_t;
  _costmap_type costmap;

  // setters for named parameter idiom
  Type & set__point(
    const geometry_msgs::msg::PointStamped_<ContainerAllocator> & _arg)
  {
    this->point = _arg;
    return *this;
  }
  Type & set__costmap(
    const uint8_t & _arg)
  {
    this->costmap = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t LOCAL_COSTMAP =
    1u;
  static constexpr uint8_t GLOBAL_COSTMAP =
    2u;

  // pointer types
  using RawPtr =
    mbf_msgs::srv::CheckPoint_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::srv::CheckPoint_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::srv::CheckPoint_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::srv::CheckPoint_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::CheckPoint_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::CheckPoint_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::CheckPoint_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::CheckPoint_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::srv::CheckPoint_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::srv::CheckPoint_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__srv__CheckPoint_Request
    std::shared_ptr<mbf_msgs::srv::CheckPoint_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__srv__CheckPoint_Request
    std::shared_ptr<mbf_msgs::srv::CheckPoint_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CheckPoint_Request_ & other) const
  {
    if (this->point != other.point) {
      return false;
    }
    if (this->costmap != other.costmap) {
      return false;
    }
    return true;
  }
  bool operator!=(const CheckPoint_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CheckPoint_Request_

// alias to use template instance with default allocator
using CheckPoint_Request =
  mbf_msgs::srv::CheckPoint_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CheckPoint_Request_<ContainerAllocator>::LOCAL_COSTMAP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CheckPoint_Request_<ContainerAllocator>::GLOBAL_COSTMAP;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace mbf_msgs


#ifndef _WIN32
# define DEPRECATED__mbf_msgs__srv__CheckPoint_Response __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__srv__CheckPoint_Response __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CheckPoint_Response_
{
  using Type = CheckPoint_Response_<ContainerAllocator>;

  explicit CheckPoint_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
      this->cost = 0ul;
    }
  }

  explicit CheckPoint_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
      this->cost = 0ul;
    }
  }

  // field types and members
  using _state_type =
    uint8_t;
  _state_type state;
  using _cost_type =
    uint32_t;
  _cost_type cost;

  // setters for named parameter idiom
  Type & set__state(
    const uint8_t & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__cost(
    const uint32_t & _arg)
  {
    this->cost = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t FREE =
    0u;
  static constexpr uint8_t INSCRIBED =
    1u;
  static constexpr uint8_t LETHAL =
    2u;
  static constexpr uint8_t UNKNOWN =
    3u;
  static constexpr uint8_t OUTSIDE =
    4u;

  // pointer types
  using RawPtr =
    mbf_msgs::srv::CheckPoint_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::srv::CheckPoint_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::srv::CheckPoint_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::srv::CheckPoint_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::CheckPoint_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::CheckPoint_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::CheckPoint_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::CheckPoint_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::srv::CheckPoint_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::srv::CheckPoint_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__srv__CheckPoint_Response
    std::shared_ptr<mbf_msgs::srv::CheckPoint_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__srv__CheckPoint_Response
    std::shared_ptr<mbf_msgs::srv::CheckPoint_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CheckPoint_Response_ & other) const
  {
    if (this->state != other.state) {
      return false;
    }
    if (this->cost != other.cost) {
      return false;
    }
    return true;
  }
  bool operator!=(const CheckPoint_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CheckPoint_Response_

// alias to use template instance with default allocator
using CheckPoint_Response =
  mbf_msgs::srv::CheckPoint_Response_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CheckPoint_Response_<ContainerAllocator>::FREE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CheckPoint_Response_<ContainerAllocator>::INSCRIBED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CheckPoint_Response_<ContainerAllocator>::LETHAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CheckPoint_Response_<ContainerAllocator>::UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CheckPoint_Response_<ContainerAllocator>::OUTSIDE;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace mbf_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__srv__CheckPoint_Event __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__srv__CheckPoint_Event __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CheckPoint_Event_
{
  using Type = CheckPoint_Event_<ContainerAllocator>;

  explicit CheckPoint_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit CheckPoint_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<mbf_msgs::srv::CheckPoint_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::srv::CheckPoint_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<mbf_msgs::srv::CheckPoint_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::srv::CheckPoint_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<mbf_msgs::srv::CheckPoint_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::srv::CheckPoint_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<mbf_msgs::srv::CheckPoint_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::srv::CheckPoint_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::srv::CheckPoint_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::srv::CheckPoint_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::srv::CheckPoint_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::srv::CheckPoint_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::CheckPoint_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::CheckPoint_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::CheckPoint_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::CheckPoint_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::srv::CheckPoint_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::srv::CheckPoint_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__srv__CheckPoint_Event
    std::shared_ptr<mbf_msgs::srv::CheckPoint_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__srv__CheckPoint_Event
    std::shared_ptr<mbf_msgs::srv::CheckPoint_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CheckPoint_Event_ & other) const
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
  bool operator!=(const CheckPoint_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CheckPoint_Event_

// alias to use template instance with default allocator
using CheckPoint_Event =
  mbf_msgs::srv::CheckPoint_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mbf_msgs

namespace mbf_msgs
{

namespace srv
{

struct CheckPoint
{
  using Request = mbf_msgs::srv::CheckPoint_Request;
  using Response = mbf_msgs::srv::CheckPoint_Response;
  using Event = mbf_msgs::srv::CheckPoint_Event;
};

}  // namespace srv

}  // namespace mbf_msgs

#endif  // MBF_MSGS__SRV__DETAIL__CHECK_POINT__STRUCT_HPP_
