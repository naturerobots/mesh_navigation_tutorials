// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mbf_msgs:srv/CheckPath.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/srv/check_path.hpp"


#ifndef MBF_MSGS__SRV__DETAIL__CHECK_PATH__STRUCT_HPP_
#define MBF_MSGS__SRV__DETAIL__CHECK_PATH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'path'
#include "nav_msgs/msg/detail/path__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__srv__CheckPath_Request __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__srv__CheckPath_Request __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CheckPath_Request_
{
  using Type = CheckPath_Request_<ContainerAllocator>;

  explicit CheckPath_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : path(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->safety_dist = 0.0f;
      this->lethal_cost_mult = 0.0f;
      this->inscrib_cost_mult = 0.0f;
      this->unknown_cost_mult = 0.0f;
      this->costmap = 0;
      this->return_on = 0;
      this->skip_poses = 0;
      this->use_padded_fp = false;
      this->path_cells_only = false;
    }
  }

  explicit CheckPath_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : path(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->safety_dist = 0.0f;
      this->lethal_cost_mult = 0.0f;
      this->inscrib_cost_mult = 0.0f;
      this->unknown_cost_mult = 0.0f;
      this->costmap = 0;
      this->return_on = 0;
      this->skip_poses = 0;
      this->use_padded_fp = false;
      this->path_cells_only = false;
    }
  }

  // field types and members
  using _path_type =
    nav_msgs::msg::Path_<ContainerAllocator>;
  _path_type path;
  using _safety_dist_type =
    float;
  _safety_dist_type safety_dist;
  using _lethal_cost_mult_type =
    float;
  _lethal_cost_mult_type lethal_cost_mult;
  using _inscrib_cost_mult_type =
    float;
  _inscrib_cost_mult_type inscrib_cost_mult;
  using _unknown_cost_mult_type =
    float;
  _unknown_cost_mult_type unknown_cost_mult;
  using _costmap_type =
    uint8_t;
  _costmap_type costmap;
  using _return_on_type =
    uint8_t;
  _return_on_type return_on;
  using _skip_poses_type =
    uint8_t;
  _skip_poses_type skip_poses;
  using _use_padded_fp_type =
    bool;
  _use_padded_fp_type use_padded_fp;
  using _path_cells_only_type =
    bool;
  _path_cells_only_type path_cells_only;

  // setters for named parameter idiom
  Type & set__path(
    const nav_msgs::msg::Path_<ContainerAllocator> & _arg)
  {
    this->path = _arg;
    return *this;
  }
  Type & set__safety_dist(
    const float & _arg)
  {
    this->safety_dist = _arg;
    return *this;
  }
  Type & set__lethal_cost_mult(
    const float & _arg)
  {
    this->lethal_cost_mult = _arg;
    return *this;
  }
  Type & set__inscrib_cost_mult(
    const float & _arg)
  {
    this->inscrib_cost_mult = _arg;
    return *this;
  }
  Type & set__unknown_cost_mult(
    const float & _arg)
  {
    this->unknown_cost_mult = _arg;
    return *this;
  }
  Type & set__costmap(
    const uint8_t & _arg)
  {
    this->costmap = _arg;
    return *this;
  }
  Type & set__return_on(
    const uint8_t & _arg)
  {
    this->return_on = _arg;
    return *this;
  }
  Type & set__skip_poses(
    const uint8_t & _arg)
  {
    this->skip_poses = _arg;
    return *this;
  }
  Type & set__use_padded_fp(
    const bool & _arg)
  {
    this->use_padded_fp = _arg;
    return *this;
  }
  Type & set__path_cells_only(
    const bool & _arg)
  {
    this->path_cells_only = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t LOCAL_COSTMAP =
    1u;
  static constexpr uint8_t GLOBAL_COSTMAP =
    2u;

  // pointer types
  using RawPtr =
    mbf_msgs::srv::CheckPath_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::srv::CheckPath_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::srv::CheckPath_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::srv::CheckPath_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::CheckPath_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::CheckPath_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::CheckPath_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::CheckPath_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::srv::CheckPath_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::srv::CheckPath_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__srv__CheckPath_Request
    std::shared_ptr<mbf_msgs::srv::CheckPath_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__srv__CheckPath_Request
    std::shared_ptr<mbf_msgs::srv::CheckPath_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CheckPath_Request_ & other) const
  {
    if (this->path != other.path) {
      return false;
    }
    if (this->safety_dist != other.safety_dist) {
      return false;
    }
    if (this->lethal_cost_mult != other.lethal_cost_mult) {
      return false;
    }
    if (this->inscrib_cost_mult != other.inscrib_cost_mult) {
      return false;
    }
    if (this->unknown_cost_mult != other.unknown_cost_mult) {
      return false;
    }
    if (this->costmap != other.costmap) {
      return false;
    }
    if (this->return_on != other.return_on) {
      return false;
    }
    if (this->skip_poses != other.skip_poses) {
      return false;
    }
    if (this->use_padded_fp != other.use_padded_fp) {
      return false;
    }
    if (this->path_cells_only != other.path_cells_only) {
      return false;
    }
    return true;
  }
  bool operator!=(const CheckPath_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CheckPath_Request_

// alias to use template instance with default allocator
using CheckPath_Request =
  mbf_msgs::srv::CheckPath_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CheckPath_Request_<ContainerAllocator>::LOCAL_COSTMAP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CheckPath_Request_<ContainerAllocator>::GLOBAL_COSTMAP;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace mbf_msgs


#ifndef _WIN32
# define DEPRECATED__mbf_msgs__srv__CheckPath_Response __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__srv__CheckPath_Response __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CheckPath_Response_
{
  using Type = CheckPath_Response_<ContainerAllocator>;

  explicit CheckPath_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->last_checked = 0ul;
      this->state = 0;
      this->cost = 0ul;
    }
  }

  explicit CheckPath_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->last_checked = 0ul;
      this->state = 0;
      this->cost = 0ul;
    }
  }

  // field types and members
  using _last_checked_type =
    uint32_t;
  _last_checked_type last_checked;
  using _state_type =
    uint8_t;
  _state_type state;
  using _cost_type =
    uint32_t;
  _cost_type cost;

  // setters for named parameter idiom
  Type & set__last_checked(
    const uint32_t & _arg)
  {
    this->last_checked = _arg;
    return *this;
  }
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
    mbf_msgs::srv::CheckPath_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::srv::CheckPath_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::srv::CheckPath_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::srv::CheckPath_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::CheckPath_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::CheckPath_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::CheckPath_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::CheckPath_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::srv::CheckPath_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::srv::CheckPath_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__srv__CheckPath_Response
    std::shared_ptr<mbf_msgs::srv::CheckPath_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__srv__CheckPath_Response
    std::shared_ptr<mbf_msgs::srv::CheckPath_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CheckPath_Response_ & other) const
  {
    if (this->last_checked != other.last_checked) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->cost != other.cost) {
      return false;
    }
    return true;
  }
  bool operator!=(const CheckPath_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CheckPath_Response_

// alias to use template instance with default allocator
using CheckPath_Response =
  mbf_msgs::srv::CheckPath_Response_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CheckPath_Response_<ContainerAllocator>::FREE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CheckPath_Response_<ContainerAllocator>::INSCRIBED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CheckPath_Response_<ContainerAllocator>::LETHAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CheckPath_Response_<ContainerAllocator>::UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CheckPath_Response_<ContainerAllocator>::OUTSIDE;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace mbf_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__srv__CheckPath_Event __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__srv__CheckPath_Event __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CheckPath_Event_
{
  using Type = CheckPath_Event_<ContainerAllocator>;

  explicit CheckPath_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit CheckPath_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<mbf_msgs::srv::CheckPath_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::srv::CheckPath_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<mbf_msgs::srv::CheckPath_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::srv::CheckPath_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<mbf_msgs::srv::CheckPath_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::srv::CheckPath_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<mbf_msgs::srv::CheckPath_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::srv::CheckPath_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::srv::CheckPath_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::srv::CheckPath_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::srv::CheckPath_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::srv::CheckPath_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::CheckPath_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::CheckPath_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::CheckPath_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::CheckPath_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::srv::CheckPath_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::srv::CheckPath_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__srv__CheckPath_Event
    std::shared_ptr<mbf_msgs::srv::CheckPath_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__srv__CheckPath_Event
    std::shared_ptr<mbf_msgs::srv::CheckPath_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CheckPath_Event_ & other) const
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
  bool operator!=(const CheckPath_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CheckPath_Event_

// alias to use template instance with default allocator
using CheckPath_Event =
  mbf_msgs::srv::CheckPath_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mbf_msgs

namespace mbf_msgs
{

namespace srv
{

struct CheckPath
{
  using Request = mbf_msgs::srv::CheckPath_Request;
  using Response = mbf_msgs::srv::CheckPath_Response;
  using Event = mbf_msgs::srv::CheckPath_Event;
};

}  // namespace srv

}  // namespace mbf_msgs

#endif  // MBF_MSGS__SRV__DETAIL__CHECK_PATH__STRUCT_HPP_
