// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mbf_msgs:action/MoveBase.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/action/move_base.hpp"


#ifndef MBF_MSGS__ACTION__DETAIL__MOVE_BASE__STRUCT_HPP_
#define MBF_MSGS__ACTION__DETAIL__MOVE_BASE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__MoveBase_Goal __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__MoveBase_Goal __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveBase_Goal_
{
  using Type = MoveBase_Goal_<ContainerAllocator>;

  explicit MoveBase_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->controller = "";
      this->planner = "";
    }
  }

  explicit MoveBase_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_pose(_alloc, _init),
    controller(_alloc),
    planner(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->controller = "";
      this->planner = "";
    }
  }

  // field types and members
  using _target_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _target_pose_type target_pose;
  using _controller_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _controller_type controller;
  using _planner_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _planner_type planner;
  using _recovery_behaviors_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _recovery_behaviors_type recovery_behaviors;

  // setters for named parameter idiom
  Type & set__target_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->target_pose = _arg;
    return *this;
  }
  Type & set__controller(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->controller = _arg;
    return *this;
  }
  Type & set__planner(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->planner = _arg;
    return *this;
  }
  Type & set__recovery_behaviors(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->recovery_behaviors = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::action::MoveBase_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::MoveBase_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::MoveBase_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::MoveBase_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::MoveBase_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::MoveBase_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::MoveBase_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::MoveBase_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::MoveBase_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::MoveBase_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__MoveBase_Goal
    std::shared_ptr<mbf_msgs::action::MoveBase_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__MoveBase_Goal
    std::shared_ptr<mbf_msgs::action::MoveBase_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveBase_Goal_ & other) const
  {
    if (this->target_pose != other.target_pose) {
      return false;
    }
    if (this->controller != other.controller) {
      return false;
    }
    if (this->planner != other.planner) {
      return false;
    }
    if (this->recovery_behaviors != other.recovery_behaviors) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveBase_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveBase_Goal_

// alias to use template instance with default allocator
using MoveBase_Goal =
  mbf_msgs::action::MoveBase_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'final_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__MoveBase_Result __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__MoveBase_Result __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveBase_Result_
{
  using Type = MoveBase_Result_<ContainerAllocator>;

  explicit MoveBase_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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

  explicit MoveBase_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
  using _dist_to_goal_type =
    float;
  _dist_to_goal_type dist_to_goal;
  using _angle_to_goal_type =
    float;
  _angle_to_goal_type angle_to_goal;
  using _final_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _final_pose_type final_pose;

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
  Type & set__final_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->final_pose = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t SUCCESS =
    0u;
  static constexpr uint8_t FAILURE =
    10u;
  static constexpr uint8_t CANCELED =
    11u;
  static constexpr uint8_t COLLISION =
    12u;
  static constexpr uint8_t OSCILLATION =
    13u;
  static constexpr uint8_t START_BLOCKED =
    14u;
  static constexpr uint8_t GOAL_BLOCKED =
    15u;
  static constexpr uint8_t TF_ERROR =
    16u;
  static constexpr uint8_t INTERNAL_ERROR =
    17u;
  static constexpr uint8_t ERROR_RANGE_START =
    10u;
  static constexpr uint8_t ERROR_RANGE_END =
    49u;
  static constexpr uint8_t PLAN_FAILURE =
    50u;
  static constexpr uint8_t CTRL_FAILURE =
    100u;

  // pointer types
  using RawPtr =
    mbf_msgs::action::MoveBase_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::MoveBase_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::MoveBase_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::MoveBase_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::MoveBase_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::MoveBase_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::MoveBase_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::MoveBase_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::MoveBase_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::MoveBase_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__MoveBase_Result
    std::shared_ptr<mbf_msgs::action::MoveBase_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__MoveBase_Result
    std::shared_ptr<mbf_msgs::action::MoveBase_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveBase_Result_ & other) const
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
    if (this->final_pose != other.final_pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveBase_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveBase_Result_

// alias to use template instance with default allocator
using MoveBase_Result =
  mbf_msgs::action::MoveBase_Result_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MoveBase_Result_<ContainerAllocator>::SUCCESS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MoveBase_Result_<ContainerAllocator>::FAILURE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MoveBase_Result_<ContainerAllocator>::CANCELED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MoveBase_Result_<ContainerAllocator>::COLLISION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MoveBase_Result_<ContainerAllocator>::OSCILLATION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MoveBase_Result_<ContainerAllocator>::START_BLOCKED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MoveBase_Result_<ContainerAllocator>::GOAL_BLOCKED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MoveBase_Result_<ContainerAllocator>::TF_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MoveBase_Result_<ContainerAllocator>::INTERNAL_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MoveBase_Result_<ContainerAllocator>::ERROR_RANGE_START;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MoveBase_Result_<ContainerAllocator>::ERROR_RANGE_END;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MoveBase_Result_<ContainerAllocator>::PLAN_FAILURE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MoveBase_Result_<ContainerAllocator>::CTRL_FAILURE;
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
# define DEPRECATED__mbf_msgs__action__MoveBase_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__MoveBase_Feedback __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveBase_Feedback_
{
  using Type = MoveBase_Feedback_<ContainerAllocator>;

  explicit MoveBase_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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

  explicit MoveBase_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    mbf_msgs::action::MoveBase_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::MoveBase_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::MoveBase_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::MoveBase_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::MoveBase_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::MoveBase_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::MoveBase_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::MoveBase_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::MoveBase_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::MoveBase_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__MoveBase_Feedback
    std::shared_ptr<mbf_msgs::action::MoveBase_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__MoveBase_Feedback
    std::shared_ptr<mbf_msgs::action::MoveBase_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveBase_Feedback_ & other) const
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
  bool operator!=(const MoveBase_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveBase_Feedback_

// alias to use template instance with default allocator
using MoveBase_Feedback =
  mbf_msgs::action::MoveBase_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "mbf_msgs/action/detail/move_base__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__MoveBase_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__MoveBase_SendGoal_Request __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveBase_SendGoal_Request_
{
  using Type = MoveBase_SendGoal_Request_<ContainerAllocator>;

  explicit MoveBase_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit MoveBase_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    mbf_msgs::action::MoveBase_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const mbf_msgs::action::MoveBase_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::action::MoveBase_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::MoveBase_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::MoveBase_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::MoveBase_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::MoveBase_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::MoveBase_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::MoveBase_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::MoveBase_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::MoveBase_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::MoveBase_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__MoveBase_SendGoal_Request
    std::shared_ptr<mbf_msgs::action::MoveBase_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__MoveBase_SendGoal_Request
    std::shared_ptr<mbf_msgs::action::MoveBase_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveBase_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveBase_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveBase_SendGoal_Request_

// alias to use template instance with default allocator
using MoveBase_SendGoal_Request =
  mbf_msgs::action::MoveBase_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__MoveBase_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__MoveBase_SendGoal_Response __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveBase_SendGoal_Response_
{
  using Type = MoveBase_SendGoal_Response_<ContainerAllocator>;

  explicit MoveBase_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit MoveBase_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    mbf_msgs::action::MoveBase_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::MoveBase_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::MoveBase_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::MoveBase_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::MoveBase_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::MoveBase_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::MoveBase_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::MoveBase_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::MoveBase_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::MoveBase_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__MoveBase_SendGoal_Response
    std::shared_ptr<mbf_msgs::action::MoveBase_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__MoveBase_SendGoal_Response
    std::shared_ptr<mbf_msgs::action::MoveBase_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveBase_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveBase_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveBase_SendGoal_Response_

// alias to use template instance with default allocator
using MoveBase_SendGoal_Response =
  mbf_msgs::action::MoveBase_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__MoveBase_SendGoal_Event __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__MoveBase_SendGoal_Event __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveBase_SendGoal_Event_
{
  using Type = MoveBase_SendGoal_Event_<ContainerAllocator>;

  explicit MoveBase_SendGoal_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit MoveBase_SendGoal_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::MoveBase_SendGoal_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::MoveBase_SendGoal_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::MoveBase_SendGoal_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::MoveBase_SendGoal_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::MoveBase_SendGoal_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::MoveBase_SendGoal_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::MoveBase_SendGoal_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::MoveBase_SendGoal_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::action::MoveBase_SendGoal_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::MoveBase_SendGoal_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::MoveBase_SendGoal_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::MoveBase_SendGoal_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::MoveBase_SendGoal_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::MoveBase_SendGoal_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::MoveBase_SendGoal_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::MoveBase_SendGoal_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::MoveBase_SendGoal_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::MoveBase_SendGoal_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__MoveBase_SendGoal_Event
    std::shared_ptr<mbf_msgs::action::MoveBase_SendGoal_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__MoveBase_SendGoal_Event
    std::shared_ptr<mbf_msgs::action::MoveBase_SendGoal_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveBase_SendGoal_Event_ & other) const
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
  bool operator!=(const MoveBase_SendGoal_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveBase_SendGoal_Event_

// alias to use template instance with default allocator
using MoveBase_SendGoal_Event =
  mbf_msgs::action::MoveBase_SendGoal_Event_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs

namespace mbf_msgs
{

namespace action
{

struct MoveBase_SendGoal
{
  using Request = mbf_msgs::action::MoveBase_SendGoal_Request;
  using Response = mbf_msgs::action::MoveBase_SendGoal_Response;
  using Event = mbf_msgs::action::MoveBase_SendGoal_Event;
};

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__MoveBase_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__MoveBase_GetResult_Request __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveBase_GetResult_Request_
{
  using Type = MoveBase_GetResult_Request_<ContainerAllocator>;

  explicit MoveBase_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit MoveBase_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    mbf_msgs::action::MoveBase_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::MoveBase_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::MoveBase_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::MoveBase_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::MoveBase_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::MoveBase_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::MoveBase_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::MoveBase_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::MoveBase_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::MoveBase_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__MoveBase_GetResult_Request
    std::shared_ptr<mbf_msgs::action::MoveBase_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__MoveBase_GetResult_Request
    std::shared_ptr<mbf_msgs::action::MoveBase_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveBase_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveBase_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveBase_GetResult_Request_

// alias to use template instance with default allocator
using MoveBase_GetResult_Request =
  mbf_msgs::action::MoveBase_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'result'
// already included above
// #include "mbf_msgs/action/detail/move_base__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__MoveBase_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__MoveBase_GetResult_Response __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveBase_GetResult_Response_
{
  using Type = MoveBase_GetResult_Response_<ContainerAllocator>;

  explicit MoveBase_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit MoveBase_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    mbf_msgs::action::MoveBase_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const mbf_msgs::action::MoveBase_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::action::MoveBase_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::MoveBase_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::MoveBase_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::MoveBase_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::MoveBase_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::MoveBase_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::MoveBase_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::MoveBase_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::MoveBase_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::MoveBase_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__MoveBase_GetResult_Response
    std::shared_ptr<mbf_msgs::action::MoveBase_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__MoveBase_GetResult_Response
    std::shared_ptr<mbf_msgs::action::MoveBase_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveBase_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveBase_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveBase_GetResult_Response_

// alias to use template instance with default allocator
using MoveBase_GetResult_Response =
  mbf_msgs::action::MoveBase_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__MoveBase_GetResult_Event __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__MoveBase_GetResult_Event __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveBase_GetResult_Event_
{
  using Type = MoveBase_GetResult_Event_<ContainerAllocator>;

  explicit MoveBase_GetResult_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit MoveBase_GetResult_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::MoveBase_GetResult_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::MoveBase_GetResult_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::MoveBase_GetResult_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::MoveBase_GetResult_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::MoveBase_GetResult_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::MoveBase_GetResult_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::MoveBase_GetResult_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::MoveBase_GetResult_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::action::MoveBase_GetResult_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::MoveBase_GetResult_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::MoveBase_GetResult_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::MoveBase_GetResult_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::MoveBase_GetResult_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::MoveBase_GetResult_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::MoveBase_GetResult_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::MoveBase_GetResult_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::MoveBase_GetResult_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::MoveBase_GetResult_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__MoveBase_GetResult_Event
    std::shared_ptr<mbf_msgs::action::MoveBase_GetResult_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__MoveBase_GetResult_Event
    std::shared_ptr<mbf_msgs::action::MoveBase_GetResult_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveBase_GetResult_Event_ & other) const
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
  bool operator!=(const MoveBase_GetResult_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveBase_GetResult_Event_

// alias to use template instance with default allocator
using MoveBase_GetResult_Event =
  mbf_msgs::action::MoveBase_GetResult_Event_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs

namespace mbf_msgs
{

namespace action
{

struct MoveBase_GetResult
{
  using Request = mbf_msgs::action::MoveBase_GetResult_Request;
  using Response = mbf_msgs::action::MoveBase_GetResult_Response;
  using Event = mbf_msgs::action::MoveBase_GetResult_Event;
};

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "mbf_msgs/action/detail/move_base__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__MoveBase_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__MoveBase_FeedbackMessage __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveBase_FeedbackMessage_
{
  using Type = MoveBase_FeedbackMessage_<ContainerAllocator>;

  explicit MoveBase_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit MoveBase_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    mbf_msgs::action::MoveBase_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const mbf_msgs::action::MoveBase_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::action::MoveBase_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::MoveBase_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::MoveBase_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::MoveBase_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::MoveBase_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::MoveBase_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::MoveBase_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::MoveBase_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::MoveBase_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::MoveBase_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__MoveBase_FeedbackMessage
    std::shared_ptr<mbf_msgs::action::MoveBase_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__MoveBase_FeedbackMessage
    std::shared_ptr<mbf_msgs::action::MoveBase_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveBase_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveBase_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveBase_FeedbackMessage_

// alias to use template instance with default allocator
using MoveBase_FeedbackMessage =
  mbf_msgs::action::MoveBase_FeedbackMessage_<std::allocator<void>>;

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

struct MoveBase
{
  /// The goal message defined in the action definition.
  using Goal = mbf_msgs::action::MoveBase_Goal;
  /// The result message defined in the action definition.
  using Result = mbf_msgs::action::MoveBase_Result;
  /// The feedback message defined in the action definition.
  using Feedback = mbf_msgs::action::MoveBase_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = mbf_msgs::action::MoveBase_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = mbf_msgs::action::MoveBase_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = mbf_msgs::action::MoveBase_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct MoveBase MoveBase;

}  // namespace action

}  // namespace mbf_msgs

#endif  // MBF_MSGS__ACTION__DETAIL__MOVE_BASE__STRUCT_HPP_
