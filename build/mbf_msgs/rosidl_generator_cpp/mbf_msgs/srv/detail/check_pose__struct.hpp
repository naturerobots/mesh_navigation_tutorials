// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mbf_msgs:srv/CheckPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/srv/check_pose.hpp"


#ifndef MBF_MSGS__SRV__DETAIL__CHECK_POSE__STRUCT_HPP_
#define MBF_MSGS__SRV__DETAIL__CHECK_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__srv__CheckPose_Request __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__srv__CheckPose_Request __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CheckPose_Request_
{
  using Type = CheckPose_Request_<ContainerAllocator>;

  explicit CheckPose_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->safety_dist = 0.0f;
      this->lethal_cost_mult = 0.0f;
      this->inscrib_cost_mult = 0.0f;
      this->unknown_cost_mult = 0.0f;
      this->costmap = 0;
      this->current_pose = false;
      this->use_padded_fp = false;
    }
  }

  explicit CheckPose_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->safety_dist = 0.0f;
      this->lethal_cost_mult = 0.0f;
      this->inscrib_cost_mult = 0.0f;
      this->unknown_cost_mult = 0.0f;
      this->costmap = 0;
      this->current_pose = false;
      this->use_padded_fp = false;
    }
  }

  // field types and members
  using _pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _pose_type pose;
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
  using _current_pose_type =
    bool;
  _current_pose_type current_pose;
  using _use_padded_fp_type =
    bool;
  _use_padded_fp_type use_padded_fp;

  // setters for named parameter idiom
  Type & set__pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
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
  Type & set__current_pose(
    const bool & _arg)
  {
    this->current_pose = _arg;
    return *this;
  }
  Type & set__use_padded_fp(
    const bool & _arg)
  {
    this->use_padded_fp = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t LOCAL_COSTMAP =
    1u;
  static constexpr uint8_t GLOBAL_COSTMAP =
    2u;

  // pointer types
  using RawPtr =
    mbf_msgs::srv::CheckPose_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::srv::CheckPose_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::srv::CheckPose_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::srv::CheckPose_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::CheckPose_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::CheckPose_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::CheckPose_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::CheckPose_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::srv::CheckPose_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::srv::CheckPose_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__srv__CheckPose_Request
    std::shared_ptr<mbf_msgs::srv::CheckPose_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__srv__CheckPose_Request
    std::shared_ptr<mbf_msgs::srv::CheckPose_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CheckPose_Request_ & other) const
  {
    if (this->pose != other.pose) {
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
    if (this->current_pose != other.current_pose) {
      return false;
    }
    if (this->use_padded_fp != other.use_padded_fp) {
      return false;
    }
    return true;
  }
  bool operator!=(const CheckPose_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CheckPose_Request_

// alias to use template instance with default allocator
using CheckPose_Request =
  mbf_msgs::srv::CheckPose_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CheckPose_Request_<ContainerAllocator>::LOCAL_COSTMAP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CheckPose_Request_<ContainerAllocator>::GLOBAL_COSTMAP;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace mbf_msgs


#ifndef _WIN32
# define DEPRECATED__mbf_msgs__srv__CheckPose_Response __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__srv__CheckPose_Response __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CheckPose_Response_
{
  using Type = CheckPose_Response_<ContainerAllocator>;

  explicit CheckPose_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
      this->cost = 0ul;
    }
  }

  explicit CheckPose_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    mbf_msgs::srv::CheckPose_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::srv::CheckPose_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::srv::CheckPose_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::srv::CheckPose_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::CheckPose_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::CheckPose_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::CheckPose_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::CheckPose_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::srv::CheckPose_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::srv::CheckPose_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__srv__CheckPose_Response
    std::shared_ptr<mbf_msgs::srv::CheckPose_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__srv__CheckPose_Response
    std::shared_ptr<mbf_msgs::srv::CheckPose_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CheckPose_Response_ & other) const
  {
    if (this->state != other.state) {
      return false;
    }
    if (this->cost != other.cost) {
      return false;
    }
    return true;
  }
  bool operator!=(const CheckPose_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CheckPose_Response_

// alias to use template instance with default allocator
using CheckPose_Response =
  mbf_msgs::srv::CheckPose_Response_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CheckPose_Response_<ContainerAllocator>::FREE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CheckPose_Response_<ContainerAllocator>::INSCRIBED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CheckPose_Response_<ContainerAllocator>::LETHAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CheckPose_Response_<ContainerAllocator>::UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t CheckPose_Response_<ContainerAllocator>::OUTSIDE;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace mbf_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__srv__CheckPose_Event __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__srv__CheckPose_Event __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CheckPose_Event_
{
  using Type = CheckPose_Event_<ContainerAllocator>;

  explicit CheckPose_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit CheckPose_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<mbf_msgs::srv::CheckPose_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::srv::CheckPose_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<mbf_msgs::srv::CheckPose_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::srv::CheckPose_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<mbf_msgs::srv::CheckPose_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::srv::CheckPose_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<mbf_msgs::srv::CheckPose_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::srv::CheckPose_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::srv::CheckPose_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::srv::CheckPose_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::srv::CheckPose_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::srv::CheckPose_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::CheckPose_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::CheckPose_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::CheckPose_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::CheckPose_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::srv::CheckPose_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::srv::CheckPose_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__srv__CheckPose_Event
    std::shared_ptr<mbf_msgs::srv::CheckPose_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__srv__CheckPose_Event
    std::shared_ptr<mbf_msgs::srv::CheckPose_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CheckPose_Event_ & other) const
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
  bool operator!=(const CheckPose_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CheckPose_Event_

// alias to use template instance with default allocator
using CheckPose_Event =
  mbf_msgs::srv::CheckPose_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mbf_msgs

namespace mbf_msgs
{

namespace srv
{

struct CheckPose
{
  using Request = mbf_msgs::srv::CheckPose_Request;
  using Response = mbf_msgs::srv::CheckPose_Response;
  using Event = mbf_msgs::srv::CheckPose_Event;
};

}  // namespace srv

}  // namespace mbf_msgs

#endif  // MBF_MSGS__SRV__DETAIL__CHECK_POSE__STRUCT_HPP_
