// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mbf_msgs:srv/SetTestRobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/srv/set_test_robot_state.hpp"


#ifndef MBF_MSGS__SRV__DETAIL__SET_TEST_ROBOT_STATE__STRUCT_HPP_
#define MBF_MSGS__SRV__DETAIL__SET_TEST_ROBOT_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'transform'
#include "geometry_msgs/msg/detail/transform_stamped__struct.hpp"
// Member 'velocity_robot'
#include "geometry_msgs/msg/detail/twist_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__srv__SetTestRobotState_Request __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__srv__SetTestRobotState_Request __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetTestRobotState_Request_
{
  using Type = SetTestRobotState_Request_<ContainerAllocator>;

  explicit SetTestRobotState_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : transform(_init),
    velocity_robot(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->set_velocity = false;
    }
  }

  explicit SetTestRobotState_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : transform(_alloc, _init),
    velocity_robot(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->set_velocity = false;
    }
  }

  // field types and members
  using _transform_type =
    geometry_msgs::msg::TransformStamped_<ContainerAllocator>;
  _transform_type transform;
  using _set_velocity_type =
    bool;
  _set_velocity_type set_velocity;
  using _velocity_robot_type =
    geometry_msgs::msg::TwistStamped_<ContainerAllocator>;
  _velocity_robot_type velocity_robot;

  // setters for named parameter idiom
  Type & set__transform(
    const geometry_msgs::msg::TransformStamped_<ContainerAllocator> & _arg)
  {
    this->transform = _arg;
    return *this;
  }
  Type & set__set_velocity(
    const bool & _arg)
  {
    this->set_velocity = _arg;
    return *this;
  }
  Type & set__velocity_robot(
    const geometry_msgs::msg::TwistStamped_<ContainerAllocator> & _arg)
  {
    this->velocity_robot = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::srv::SetTestRobotState_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::srv::SetTestRobotState_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::srv::SetTestRobotState_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::srv::SetTestRobotState_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::SetTestRobotState_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::SetTestRobotState_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::SetTestRobotState_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::SetTestRobotState_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::srv::SetTestRobotState_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::srv::SetTestRobotState_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__srv__SetTestRobotState_Request
    std::shared_ptr<mbf_msgs::srv::SetTestRobotState_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__srv__SetTestRobotState_Request
    std::shared_ptr<mbf_msgs::srv::SetTestRobotState_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetTestRobotState_Request_ & other) const
  {
    if (this->transform != other.transform) {
      return false;
    }
    if (this->set_velocity != other.set_velocity) {
      return false;
    }
    if (this->velocity_robot != other.velocity_robot) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetTestRobotState_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetTestRobotState_Request_

// alias to use template instance with default allocator
using SetTestRobotState_Request =
  mbf_msgs::srv::SetTestRobotState_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mbf_msgs


#ifndef _WIN32
# define DEPRECATED__mbf_msgs__srv__SetTestRobotState_Response __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__srv__SetTestRobotState_Response __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetTestRobotState_Response_
{
  using Type = SetTestRobotState_Response_<ContainerAllocator>;

  explicit SetTestRobotState_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit SetTestRobotState_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::srv::SetTestRobotState_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::srv::SetTestRobotState_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::srv::SetTestRobotState_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::srv::SetTestRobotState_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::SetTestRobotState_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::SetTestRobotState_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::SetTestRobotState_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::SetTestRobotState_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::srv::SetTestRobotState_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::srv::SetTestRobotState_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__srv__SetTestRobotState_Response
    std::shared_ptr<mbf_msgs::srv::SetTestRobotState_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__srv__SetTestRobotState_Response
    std::shared_ptr<mbf_msgs::srv::SetTestRobotState_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetTestRobotState_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetTestRobotState_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetTestRobotState_Response_

// alias to use template instance with default allocator
using SetTestRobotState_Response =
  mbf_msgs::srv::SetTestRobotState_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mbf_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__srv__SetTestRobotState_Event __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__srv__SetTestRobotState_Event __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetTestRobotState_Event_
{
  using Type = SetTestRobotState_Event_<ContainerAllocator>;

  explicit SetTestRobotState_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit SetTestRobotState_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<mbf_msgs::srv::SetTestRobotState_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::srv::SetTestRobotState_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<mbf_msgs::srv::SetTestRobotState_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::srv::SetTestRobotState_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<mbf_msgs::srv::SetTestRobotState_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::srv::SetTestRobotState_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<mbf_msgs::srv::SetTestRobotState_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::srv::SetTestRobotState_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::srv::SetTestRobotState_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::srv::SetTestRobotState_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::srv::SetTestRobotState_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::srv::SetTestRobotState_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::SetTestRobotState_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::SetTestRobotState_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::srv::SetTestRobotState_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::srv::SetTestRobotState_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::srv::SetTestRobotState_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::srv::SetTestRobotState_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__srv__SetTestRobotState_Event
    std::shared_ptr<mbf_msgs::srv::SetTestRobotState_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__srv__SetTestRobotState_Event
    std::shared_ptr<mbf_msgs::srv::SetTestRobotState_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetTestRobotState_Event_ & other) const
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
  bool operator!=(const SetTestRobotState_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetTestRobotState_Event_

// alias to use template instance with default allocator
using SetTestRobotState_Event =
  mbf_msgs::srv::SetTestRobotState_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mbf_msgs

namespace mbf_msgs
{

namespace srv
{

struct SetTestRobotState
{
  using Request = mbf_msgs::srv::SetTestRobotState_Request;
  using Response = mbf_msgs::srv::SetTestRobotState_Response;
  using Event = mbf_msgs::srv::SetTestRobotState_Event;
};

}  // namespace srv

}  // namespace mbf_msgs

#endif  // MBF_MSGS__SRV__DETAIL__SET_TEST_ROBOT_STATE__STRUCT_HPP_
