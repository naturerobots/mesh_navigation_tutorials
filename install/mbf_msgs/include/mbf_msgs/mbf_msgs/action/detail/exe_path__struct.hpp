// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mbf_msgs:action/ExePath.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/action/exe_path.hpp"


#ifndef MBF_MSGS__ACTION__DETAIL__EXE_PATH__STRUCT_HPP_
#define MBF_MSGS__ACTION__DETAIL__EXE_PATH__STRUCT_HPP_

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
# define DEPRECATED__mbf_msgs__action__ExePath_Goal __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__ExePath_Goal __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExePath_Goal_
{
  using Type = ExePath_Goal_<ContainerAllocator>;

  explicit ExePath_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : path(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->controller = "";
      this->concurrency_slot = 0;
      this->tolerance_from_action = false;
      this->dist_tolerance = 0.0f;
      this->angle_tolerance = 0.0f;
    }
  }

  explicit ExePath_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : path(_alloc, _init),
    controller(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->controller = "";
      this->concurrency_slot = 0;
      this->tolerance_from_action = false;
      this->dist_tolerance = 0.0f;
      this->angle_tolerance = 0.0f;
    }
  }

  // field types and members
  using _path_type =
    nav_msgs::msg::Path_<ContainerAllocator>;
  _path_type path;
  using _controller_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _controller_type controller;
  using _concurrency_slot_type =
    uint8_t;
  _concurrency_slot_type concurrency_slot;
  using _tolerance_from_action_type =
    bool;
  _tolerance_from_action_type tolerance_from_action;
  using _dist_tolerance_type =
    float;
  _dist_tolerance_type dist_tolerance;
  using _angle_tolerance_type =
    float;
  _angle_tolerance_type angle_tolerance;

  // setters for named parameter idiom
  Type & set__path(
    const nav_msgs::msg::Path_<ContainerAllocator> & _arg)
  {
    this->path = _arg;
    return *this;
  }
  Type & set__controller(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->controller = _arg;
    return *this;
  }
  Type & set__concurrency_slot(
    const uint8_t & _arg)
  {
    this->concurrency_slot = _arg;
    return *this;
  }
  Type & set__tolerance_from_action(
    const bool & _arg)
  {
    this->tolerance_from_action = _arg;
    return *this;
  }
  Type & set__dist_tolerance(
    const float & _arg)
  {
    this->dist_tolerance = _arg;
    return *this;
  }
  Type & set__angle_tolerance(
    const float & _arg)
  {
    this->angle_tolerance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::action::ExePath_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::ExePath_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::ExePath_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::ExePath_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::ExePath_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::ExePath_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::ExePath_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::ExePath_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::ExePath_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::ExePath_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__ExePath_Goal
    std::shared_ptr<mbf_msgs::action::ExePath_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__ExePath_Goal
    std::shared_ptr<mbf_msgs::action::ExePath_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExePath_Goal_ & other) const
  {
    if (this->path != other.path) {
      return false;
    }
    if (this->controller != other.controller) {
      return false;
    }
    if (this->concurrency_slot != other.concurrency_slot) {
      return false;
    }
    if (this->tolerance_from_action != other.tolerance_from_action) {
      return false;
    }
    if (this->dist_tolerance != other.dist_tolerance) {
      return false;
    }
    if (this->angle_tolerance != other.angle_tolerance) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExePath_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExePath_Goal_

// alias to use template instance with default allocator
using ExePath_Goal =
  mbf_msgs::action::ExePath_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'final_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__ExePath_Result __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__ExePath_Result __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExePath_Result_
{
  using Type = ExePath_Result_<ContainerAllocator>;

  explicit ExePath_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : final_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->outcome = 0ul;
      this->message = "";
      this->dist_to_goal = 0.0f;
      this->angle_to_goal = 0.0f;
    }
  }

  explicit ExePath_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    final_pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->outcome = 0ul;
      this->message = "";
      this->dist_to_goal = 0.0f;
      this->angle_to_goal = 0.0f;
    }
  }

  // field types and members
  using _outcome_type =
    uint32_t;
  _outcome_type outcome;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _final_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _final_pose_type final_pose;
  using _dist_to_goal_type =
    float;
  _dist_to_goal_type dist_to_goal;
  using _angle_to_goal_type =
    float;
  _angle_to_goal_type angle_to_goal;

  // setters for named parameter idiom
  Type & set__outcome(
    const uint32_t & _arg)
  {
    this->outcome = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__final_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->final_pose = _arg;
    return *this;
  }
  Type & set__dist_to_goal(
    const float & _arg)
  {
    this->dist_to_goal = _arg;
    return *this;
  }
  Type & set__angle_to_goal(
    const float & _arg)
  {
    this->angle_to_goal = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t SUCCESS =
    0u;
  static constexpr uint8_t FAILURE =
    100u;
  static constexpr uint8_t CANCELED =
    101u;
  static constexpr uint8_t NO_VALID_CMD =
    102u;
  static constexpr uint8_t PAT_EXCEEDED =
    103u;
  static constexpr uint8_t COLLISION =
    104u;
  static constexpr uint8_t OSCILLATION =
    105u;
  static constexpr uint8_t ROBOT_STUCK =
    106u;
  static constexpr uint8_t MISSED_GOAL =
    107u;
  static constexpr uint8_t MISSED_PATH =
    108u;
  static constexpr uint8_t BLOCKED_GOAL =
    109u;
  static constexpr uint8_t BLOCKED_PATH =
    110u;
  static constexpr uint8_t INVALID_PATH =
    111u;
  static constexpr uint8_t TF_ERROR =
    112u;
  static constexpr uint8_t NOT_INITIALIZED =
    113u;
  static constexpr uint8_t INVALID_PLUGIN =
    114u;
  static constexpr uint8_t INTERNAL_ERROR =
    115u;
  static constexpr uint8_t OUT_OF_MAP =
    116u;
  static constexpr uint8_t MAP_ERROR =
    117u;
  static constexpr uint8_t STOPPED =
    118u;
  static constexpr uint8_t ERROR_RANGE_START =
    100u;
  static constexpr uint8_t ERROR_RANGE_END =
    149u;
  static constexpr uint8_t PLUGIN_ERROR_RANGE_START =
    121u;
  static constexpr uint8_t PLUGIN_ERROR_RANGE_END =
    149u;

  // pointer types
  using RawPtr =
    mbf_msgs::action::ExePath_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::ExePath_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::ExePath_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::ExePath_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::ExePath_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::ExePath_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::ExePath_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::ExePath_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::ExePath_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::ExePath_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__ExePath_Result
    std::shared_ptr<mbf_msgs::action::ExePath_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__ExePath_Result
    std::shared_ptr<mbf_msgs::action::ExePath_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExePath_Result_ & other) const
  {
    if (this->outcome != other.outcome) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->final_pose != other.final_pose) {
      return false;
    }
    if (this->dist_to_goal != other.dist_to_goal) {
      return false;
    }
    if (this->angle_to_goal != other.angle_to_goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExePath_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExePath_Result_

// alias to use template instance with default allocator
using ExePath_Result =
  mbf_msgs::action::ExePath_Result_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::SUCCESS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::FAILURE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::CANCELED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::NO_VALID_CMD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::PAT_EXCEEDED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::COLLISION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::OSCILLATION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::ROBOT_STUCK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::MISSED_GOAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::MISSED_PATH;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::BLOCKED_GOAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::BLOCKED_PATH;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::INVALID_PATH;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::TF_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::NOT_INITIALIZED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::INVALID_PLUGIN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::INTERNAL_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::OUT_OF_MAP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::MAP_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::STOPPED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::ERROR_RANGE_START;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::ERROR_RANGE_END;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::PLUGIN_ERROR_RANGE_START;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ExePath_Result_<ContainerAllocator>::PLUGIN_ERROR_RANGE_END;
#endif  // __cplusplus < 201703L

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'current_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
// Member 'last_cmd_vel'
#include "geometry_msgs/msg/detail/twist_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__ExePath_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__ExePath_Feedback __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExePath_Feedback_
{
  using Type = ExePath_Feedback_<ContainerAllocator>;

  explicit ExePath_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : current_pose(_init),
    last_cmd_vel(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->outcome = 0ul;
      this->message = "";
      this->dist_to_goal = 0.0f;
      this->angle_to_goal = 0.0f;
    }
  }

  explicit ExePath_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    current_pose(_alloc, _init),
    last_cmd_vel(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->outcome = 0ul;
      this->message = "";
      this->dist_to_goal = 0.0f;
      this->angle_to_goal = 0.0f;
    }
  }

  // field types and members
  using _outcome_type =
    uint32_t;
  _outcome_type outcome;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _dist_to_goal_type =
    float;
  _dist_to_goal_type dist_to_goal;
  using _angle_to_goal_type =
    float;
  _angle_to_goal_type angle_to_goal;
  using _current_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _current_pose_type current_pose;
  using _last_cmd_vel_type =
    geometry_msgs::msg::TwistStamped_<ContainerAllocator>;
  _last_cmd_vel_type last_cmd_vel;

  // setters for named parameter idiom
  Type & set__outcome(
    const uint32_t & _arg)
  {
    this->outcome = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__dist_to_goal(
    const float & _arg)
  {
    this->dist_to_goal = _arg;
    return *this;
  }
  Type & set__angle_to_goal(
    const float & _arg)
  {
    this->angle_to_goal = _arg;
    return *this;
  }
  Type & set__current_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->current_pose = _arg;
    return *this;
  }
  Type & set__last_cmd_vel(
    const geometry_msgs::msg::TwistStamped_<ContainerAllocator> & _arg)
  {
    this->last_cmd_vel = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::action::ExePath_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::ExePath_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::ExePath_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::ExePath_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::ExePath_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::ExePath_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::ExePath_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::ExePath_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::ExePath_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::ExePath_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__ExePath_Feedback
    std::shared_ptr<mbf_msgs::action::ExePath_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__ExePath_Feedback
    std::shared_ptr<mbf_msgs::action::ExePath_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExePath_Feedback_ & other) const
  {
    if (this->outcome != other.outcome) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->dist_to_goal != other.dist_to_goal) {
      return false;
    }
    if (this->angle_to_goal != other.angle_to_goal) {
      return false;
    }
    if (this->current_pose != other.current_pose) {
      return false;
    }
    if (this->last_cmd_vel != other.last_cmd_vel) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExePath_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExePath_Feedback_

// alias to use template instance with default allocator
using ExePath_Feedback =
  mbf_msgs::action::ExePath_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "mbf_msgs/action/detail/exe_path__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__ExePath_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__ExePath_SendGoal_Request __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExePath_SendGoal_Request_
{
  using Type = ExePath_SendGoal_Request_<ContainerAllocator>;

  explicit ExePath_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit ExePath_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    mbf_msgs::action::ExePath_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const mbf_msgs::action::ExePath_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::action::ExePath_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::ExePath_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::ExePath_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::ExePath_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::ExePath_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::ExePath_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::ExePath_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::ExePath_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::ExePath_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::ExePath_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__ExePath_SendGoal_Request
    std::shared_ptr<mbf_msgs::action::ExePath_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__ExePath_SendGoal_Request
    std::shared_ptr<mbf_msgs::action::ExePath_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExePath_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExePath_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExePath_SendGoal_Request_

// alias to use template instance with default allocator
using ExePath_SendGoal_Request =
  mbf_msgs::action::ExePath_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__ExePath_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__ExePath_SendGoal_Response __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExePath_SendGoal_Response_
{
  using Type = ExePath_SendGoal_Response_<ContainerAllocator>;

  explicit ExePath_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit ExePath_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::action::ExePath_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::ExePath_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::ExePath_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::ExePath_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::ExePath_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::ExePath_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::ExePath_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::ExePath_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::ExePath_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::ExePath_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__ExePath_SendGoal_Response
    std::shared_ptr<mbf_msgs::action::ExePath_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__ExePath_SendGoal_Response
    std::shared_ptr<mbf_msgs::action::ExePath_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExePath_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExePath_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExePath_SendGoal_Response_

// alias to use template instance with default allocator
using ExePath_SendGoal_Response =
  mbf_msgs::action::ExePath_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__ExePath_SendGoal_Event __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__ExePath_SendGoal_Event __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExePath_SendGoal_Event_
{
  using Type = ExePath_SendGoal_Event_<ContainerAllocator>;

  explicit ExePath_SendGoal_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit ExePath_SendGoal_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::ExePath_SendGoal_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::ExePath_SendGoal_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::ExePath_SendGoal_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::ExePath_SendGoal_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::ExePath_SendGoal_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::ExePath_SendGoal_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::ExePath_SendGoal_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::ExePath_SendGoal_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::action::ExePath_SendGoal_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::ExePath_SendGoal_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::ExePath_SendGoal_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::ExePath_SendGoal_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::ExePath_SendGoal_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::ExePath_SendGoal_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::ExePath_SendGoal_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::ExePath_SendGoal_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::ExePath_SendGoal_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::ExePath_SendGoal_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__ExePath_SendGoal_Event
    std::shared_ptr<mbf_msgs::action::ExePath_SendGoal_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__ExePath_SendGoal_Event
    std::shared_ptr<mbf_msgs::action::ExePath_SendGoal_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExePath_SendGoal_Event_ & other) const
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
  bool operator!=(const ExePath_SendGoal_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExePath_SendGoal_Event_

// alias to use template instance with default allocator
using ExePath_SendGoal_Event =
  mbf_msgs::action::ExePath_SendGoal_Event_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs

namespace mbf_msgs
{

namespace action
{

struct ExePath_SendGoal
{
  using Request = mbf_msgs::action::ExePath_SendGoal_Request;
  using Response = mbf_msgs::action::ExePath_SendGoal_Response;
  using Event = mbf_msgs::action::ExePath_SendGoal_Event;
};

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__ExePath_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__ExePath_GetResult_Request __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExePath_GetResult_Request_
{
  using Type = ExePath_GetResult_Request_<ContainerAllocator>;

  explicit ExePath_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit ExePath_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::action::ExePath_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::ExePath_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::ExePath_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::ExePath_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::ExePath_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::ExePath_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::ExePath_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::ExePath_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::ExePath_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::ExePath_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__ExePath_GetResult_Request
    std::shared_ptr<mbf_msgs::action::ExePath_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__ExePath_GetResult_Request
    std::shared_ptr<mbf_msgs::action::ExePath_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExePath_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExePath_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExePath_GetResult_Request_

// alias to use template instance with default allocator
using ExePath_GetResult_Request =
  mbf_msgs::action::ExePath_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'result'
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__ExePath_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__ExePath_GetResult_Response __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExePath_GetResult_Response_
{
  using Type = ExePath_GetResult_Response_<ContainerAllocator>;

  explicit ExePath_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit ExePath_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    mbf_msgs::action::ExePath_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const mbf_msgs::action::ExePath_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::action::ExePath_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::ExePath_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::ExePath_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::ExePath_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::ExePath_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::ExePath_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::ExePath_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::ExePath_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::ExePath_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::ExePath_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__ExePath_GetResult_Response
    std::shared_ptr<mbf_msgs::action::ExePath_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__ExePath_GetResult_Response
    std::shared_ptr<mbf_msgs::action::ExePath_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExePath_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExePath_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExePath_GetResult_Response_

// alias to use template instance with default allocator
using ExePath_GetResult_Response =
  mbf_msgs::action::ExePath_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__ExePath_GetResult_Event __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__ExePath_GetResult_Event __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExePath_GetResult_Event_
{
  using Type = ExePath_GetResult_Event_<ContainerAllocator>;

  explicit ExePath_GetResult_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit ExePath_GetResult_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::ExePath_GetResult_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::ExePath_GetResult_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::ExePath_GetResult_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::ExePath_GetResult_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::ExePath_GetResult_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::ExePath_GetResult_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::ExePath_GetResult_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::ExePath_GetResult_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::action::ExePath_GetResult_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::ExePath_GetResult_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::ExePath_GetResult_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::ExePath_GetResult_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::ExePath_GetResult_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::ExePath_GetResult_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::ExePath_GetResult_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::ExePath_GetResult_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::ExePath_GetResult_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::ExePath_GetResult_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__ExePath_GetResult_Event
    std::shared_ptr<mbf_msgs::action::ExePath_GetResult_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__ExePath_GetResult_Event
    std::shared_ptr<mbf_msgs::action::ExePath_GetResult_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExePath_GetResult_Event_ & other) const
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
  bool operator!=(const ExePath_GetResult_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExePath_GetResult_Event_

// alias to use template instance with default allocator
using ExePath_GetResult_Event =
  mbf_msgs::action::ExePath_GetResult_Event_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs

namespace mbf_msgs
{

namespace action
{

struct ExePath_GetResult
{
  using Request = mbf_msgs::action::ExePath_GetResult_Request;
  using Response = mbf_msgs::action::ExePath_GetResult_Response;
  using Event = mbf_msgs::action::ExePath_GetResult_Event;
};

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__ExePath_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__ExePath_FeedbackMessage __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExePath_FeedbackMessage_
{
  using Type = ExePath_FeedbackMessage_<ContainerAllocator>;

  explicit ExePath_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit ExePath_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    mbf_msgs::action::ExePath_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const mbf_msgs::action::ExePath_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::action::ExePath_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::ExePath_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::ExePath_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::ExePath_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::ExePath_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::ExePath_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::ExePath_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::ExePath_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::ExePath_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::ExePath_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__ExePath_FeedbackMessage
    std::shared_ptr<mbf_msgs::action::ExePath_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__ExePath_FeedbackMessage
    std::shared_ptr<mbf_msgs::action::ExePath_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExePath_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExePath_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExePath_FeedbackMessage_

// alias to use template instance with default allocator
using ExePath_FeedbackMessage =
  mbf_msgs::action::ExePath_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace mbf_msgs
{

namespace action
{

struct ExePath
{
  /// The goal message defined in the action definition.
  using Goal = mbf_msgs::action::ExePath_Goal;
  /// The result message defined in the action definition.
  using Result = mbf_msgs::action::ExePath_Result;
  /// The feedback message defined in the action definition.
  using Feedback = mbf_msgs::action::ExePath_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = mbf_msgs::action::ExePath_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = mbf_msgs::action::ExePath_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = mbf_msgs::action::ExePath_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct ExePath ExePath;

}  // namespace action

}  // namespace mbf_msgs

#endif  // MBF_MSGS__ACTION__DETAIL__EXE_PATH__STRUCT_HPP_
