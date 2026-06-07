// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mbf_msgs:action/Recovery.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/action/recovery.hpp"


#ifndef MBF_MSGS__ACTION__DETAIL__RECOVERY__STRUCT_HPP_
#define MBF_MSGS__ACTION__DETAIL__RECOVERY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__Recovery_Goal __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__Recovery_Goal __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Recovery_Goal_
{
  using Type = Recovery_Goal_<ContainerAllocator>;

  explicit Recovery_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->behavior = "";
      this->concurrency_slot = 0;
    }
  }

  explicit Recovery_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : behavior(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->behavior = "";
      this->concurrency_slot = 0;
    }
  }

  // field types and members
  using _behavior_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _behavior_type behavior;
  using _concurrency_slot_type =
    uint8_t;
  _concurrency_slot_type concurrency_slot;

  // setters for named parameter idiom
  Type & set__behavior(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->behavior = _arg;
    return *this;
  }
  Type & set__concurrency_slot(
    const uint8_t & _arg)
  {
    this->concurrency_slot = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::action::Recovery_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::Recovery_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::Recovery_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::Recovery_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::Recovery_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::Recovery_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::Recovery_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::Recovery_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::Recovery_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::Recovery_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__Recovery_Goal
    std::shared_ptr<mbf_msgs::action::Recovery_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__Recovery_Goal
    std::shared_ptr<mbf_msgs::action::Recovery_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Recovery_Goal_ & other) const
  {
    if (this->behavior != other.behavior) {
      return false;
    }
    if (this->concurrency_slot != other.concurrency_slot) {
      return false;
    }
    return true;
  }
  bool operator!=(const Recovery_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Recovery_Goal_

// alias to use template instance with default allocator
using Recovery_Goal =
  mbf_msgs::action::Recovery_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs


#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__Recovery_Result __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__Recovery_Result __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Recovery_Result_
{
  using Type = Recovery_Result_<ContainerAllocator>;

  explicit Recovery_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->outcome = 0ul;
      this->message = "";
      this->used_plugin = "";
    }
  }

  explicit Recovery_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    used_plugin(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->outcome = 0ul;
      this->message = "";
      this->used_plugin = "";
    }
  }

  // field types and members
  using _outcome_type =
    uint32_t;
  _outcome_type outcome;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _used_plugin_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _used_plugin_type used_plugin;

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
  Type & set__used_plugin(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->used_plugin = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t SUCCESS =
    0u;
  static constexpr uint8_t FAILURE =
    150u;
  static constexpr uint8_t CANCELED =
    151u;
  static constexpr uint8_t PAT_EXCEEDED =
    152u;
  static constexpr uint8_t TF_ERROR =
    153u;
  static constexpr uint8_t NOT_INITIALIZED =
    154u;
  static constexpr uint8_t INVALID_PLUGIN =
    155u;
  static constexpr uint8_t INTERNAL_ERROR =
    156u;
  static constexpr uint8_t STOPPED =
    157u;
  static constexpr uint8_t IMPASSABLE =
    158u;
  static constexpr uint8_t ERROR_RANGE_START =
    150u;
  static constexpr uint8_t ERROR_RANGE_END =
    199u;
  static constexpr uint8_t PLUGIN_ERROR_RANGE_START =
    171u;
  static constexpr uint8_t PLUGIN_ERROR_RANGE_END =
    199u;

  // pointer types
  using RawPtr =
    mbf_msgs::action::Recovery_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::Recovery_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::Recovery_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::Recovery_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::Recovery_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::Recovery_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::Recovery_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::Recovery_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::Recovery_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::Recovery_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__Recovery_Result
    std::shared_ptr<mbf_msgs::action::Recovery_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__Recovery_Result
    std::shared_ptr<mbf_msgs::action::Recovery_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Recovery_Result_ & other) const
  {
    if (this->outcome != other.outcome) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->used_plugin != other.used_plugin) {
      return false;
    }
    return true;
  }
  bool operator!=(const Recovery_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Recovery_Result_

// alias to use template instance with default allocator
using Recovery_Result =
  mbf_msgs::action::Recovery_Result_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Recovery_Result_<ContainerAllocator>::SUCCESS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Recovery_Result_<ContainerAllocator>::FAILURE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Recovery_Result_<ContainerAllocator>::CANCELED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Recovery_Result_<ContainerAllocator>::PAT_EXCEEDED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Recovery_Result_<ContainerAllocator>::TF_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Recovery_Result_<ContainerAllocator>::NOT_INITIALIZED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Recovery_Result_<ContainerAllocator>::INVALID_PLUGIN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Recovery_Result_<ContainerAllocator>::INTERNAL_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Recovery_Result_<ContainerAllocator>::STOPPED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Recovery_Result_<ContainerAllocator>::IMPASSABLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Recovery_Result_<ContainerAllocator>::ERROR_RANGE_START;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Recovery_Result_<ContainerAllocator>::ERROR_RANGE_END;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Recovery_Result_<ContainerAllocator>::PLUGIN_ERROR_RANGE_START;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Recovery_Result_<ContainerAllocator>::PLUGIN_ERROR_RANGE_END;
#endif  // __cplusplus < 201703L

}  // namespace action

}  // namespace mbf_msgs


#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__Recovery_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__Recovery_Feedback __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Recovery_Feedback_
{
  using Type = Recovery_Feedback_<ContainerAllocator>;

  explicit Recovery_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit Recovery_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::action::Recovery_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::Recovery_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::Recovery_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::Recovery_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::Recovery_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::Recovery_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::Recovery_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::Recovery_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::Recovery_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::Recovery_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__Recovery_Feedback
    std::shared_ptr<mbf_msgs::action::Recovery_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__Recovery_Feedback
    std::shared_ptr<mbf_msgs::action::Recovery_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Recovery_Feedback_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const Recovery_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Recovery_Feedback_

// alias to use template instance with default allocator
using Recovery_Feedback =
  mbf_msgs::action::Recovery_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "mbf_msgs/action/detail/recovery__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__Recovery_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__Recovery_SendGoal_Request __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Recovery_SendGoal_Request_
{
  using Type = Recovery_SendGoal_Request_<ContainerAllocator>;

  explicit Recovery_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit Recovery_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    mbf_msgs::action::Recovery_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const mbf_msgs::action::Recovery_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::action::Recovery_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::Recovery_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::Recovery_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::Recovery_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::Recovery_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::Recovery_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::Recovery_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::Recovery_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::Recovery_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::Recovery_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__Recovery_SendGoal_Request
    std::shared_ptr<mbf_msgs::action::Recovery_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__Recovery_SendGoal_Request
    std::shared_ptr<mbf_msgs::action::Recovery_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Recovery_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const Recovery_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Recovery_SendGoal_Request_

// alias to use template instance with default allocator
using Recovery_SendGoal_Request =
  mbf_msgs::action::Recovery_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__Recovery_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__Recovery_SendGoal_Response __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Recovery_SendGoal_Response_
{
  using Type = Recovery_SendGoal_Response_<ContainerAllocator>;

  explicit Recovery_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit Recovery_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    mbf_msgs::action::Recovery_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::Recovery_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::Recovery_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::Recovery_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::Recovery_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::Recovery_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::Recovery_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::Recovery_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::Recovery_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::Recovery_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__Recovery_SendGoal_Response
    std::shared_ptr<mbf_msgs::action::Recovery_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__Recovery_SendGoal_Response
    std::shared_ptr<mbf_msgs::action::Recovery_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Recovery_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const Recovery_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Recovery_SendGoal_Response_

// alias to use template instance with default allocator
using Recovery_SendGoal_Response =
  mbf_msgs::action::Recovery_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__Recovery_SendGoal_Event __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__Recovery_SendGoal_Event __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Recovery_SendGoal_Event_
{
  using Type = Recovery_SendGoal_Event_<ContainerAllocator>;

  explicit Recovery_SendGoal_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit Recovery_SendGoal_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::Recovery_SendGoal_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::Recovery_SendGoal_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::Recovery_SendGoal_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::Recovery_SendGoal_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::Recovery_SendGoal_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::Recovery_SendGoal_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::Recovery_SendGoal_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::Recovery_SendGoal_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::action::Recovery_SendGoal_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::Recovery_SendGoal_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::Recovery_SendGoal_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::Recovery_SendGoal_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::Recovery_SendGoal_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::Recovery_SendGoal_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::Recovery_SendGoal_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::Recovery_SendGoal_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::Recovery_SendGoal_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::Recovery_SendGoal_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__Recovery_SendGoal_Event
    std::shared_ptr<mbf_msgs::action::Recovery_SendGoal_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__Recovery_SendGoal_Event
    std::shared_ptr<mbf_msgs::action::Recovery_SendGoal_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Recovery_SendGoal_Event_ & other) const
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
  bool operator!=(const Recovery_SendGoal_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Recovery_SendGoal_Event_

// alias to use template instance with default allocator
using Recovery_SendGoal_Event =
  mbf_msgs::action::Recovery_SendGoal_Event_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs

namespace mbf_msgs
{

namespace action
{

struct Recovery_SendGoal
{
  using Request = mbf_msgs::action::Recovery_SendGoal_Request;
  using Response = mbf_msgs::action::Recovery_SendGoal_Response;
  using Event = mbf_msgs::action::Recovery_SendGoal_Event;
};

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__Recovery_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__Recovery_GetResult_Request __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Recovery_GetResult_Request_
{
  using Type = Recovery_GetResult_Request_<ContainerAllocator>;

  explicit Recovery_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit Recovery_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    mbf_msgs::action::Recovery_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::Recovery_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::Recovery_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::Recovery_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::Recovery_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::Recovery_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::Recovery_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::Recovery_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::Recovery_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::Recovery_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__Recovery_GetResult_Request
    std::shared_ptr<mbf_msgs::action::Recovery_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__Recovery_GetResult_Request
    std::shared_ptr<mbf_msgs::action::Recovery_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Recovery_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const Recovery_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Recovery_GetResult_Request_

// alias to use template instance with default allocator
using Recovery_GetResult_Request =
  mbf_msgs::action::Recovery_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'result'
// already included above
// #include "mbf_msgs/action/detail/recovery__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__Recovery_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__Recovery_GetResult_Response __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Recovery_GetResult_Response_
{
  using Type = Recovery_GetResult_Response_<ContainerAllocator>;

  explicit Recovery_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit Recovery_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    mbf_msgs::action::Recovery_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const mbf_msgs::action::Recovery_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::action::Recovery_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::Recovery_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::Recovery_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::Recovery_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::Recovery_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::Recovery_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::Recovery_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::Recovery_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::Recovery_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::Recovery_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__Recovery_GetResult_Response
    std::shared_ptr<mbf_msgs::action::Recovery_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__Recovery_GetResult_Response
    std::shared_ptr<mbf_msgs::action::Recovery_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Recovery_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const Recovery_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Recovery_GetResult_Response_

// alias to use template instance with default allocator
using Recovery_GetResult_Response =
  mbf_msgs::action::Recovery_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__Recovery_GetResult_Event __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__Recovery_GetResult_Event __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Recovery_GetResult_Event_
{
  using Type = Recovery_GetResult_Event_<ContainerAllocator>;

  explicit Recovery_GetResult_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit Recovery_GetResult_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::Recovery_GetResult_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::Recovery_GetResult_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::Recovery_GetResult_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::Recovery_GetResult_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::Recovery_GetResult_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::Recovery_GetResult_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<mbf_msgs::action::Recovery_GetResult_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mbf_msgs::action::Recovery_GetResult_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::action::Recovery_GetResult_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::Recovery_GetResult_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::Recovery_GetResult_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::Recovery_GetResult_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::Recovery_GetResult_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::Recovery_GetResult_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::Recovery_GetResult_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::Recovery_GetResult_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::Recovery_GetResult_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::Recovery_GetResult_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__Recovery_GetResult_Event
    std::shared_ptr<mbf_msgs::action::Recovery_GetResult_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__Recovery_GetResult_Event
    std::shared_ptr<mbf_msgs::action::Recovery_GetResult_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Recovery_GetResult_Event_ & other) const
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
  bool operator!=(const Recovery_GetResult_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Recovery_GetResult_Event_

// alias to use template instance with default allocator
using Recovery_GetResult_Event =
  mbf_msgs::action::Recovery_GetResult_Event_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace mbf_msgs

namespace mbf_msgs
{

namespace action
{

struct Recovery_GetResult
{
  using Request = mbf_msgs::action::Recovery_GetResult_Request;
  using Response = mbf_msgs::action::Recovery_GetResult_Response;
  using Event = mbf_msgs::action::Recovery_GetResult_Event;
};

}  // namespace action

}  // namespace mbf_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "mbf_msgs/action/detail/recovery__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mbf_msgs__action__Recovery_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__mbf_msgs__action__Recovery_FeedbackMessage __declspec(deprecated)
#endif

namespace mbf_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Recovery_FeedbackMessage_
{
  using Type = Recovery_FeedbackMessage_<ContainerAllocator>;

  explicit Recovery_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit Recovery_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    mbf_msgs::action::Recovery_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const mbf_msgs::action::Recovery_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mbf_msgs::action::Recovery_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const mbf_msgs::action::Recovery_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mbf_msgs::action::Recovery_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mbf_msgs::action::Recovery_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::Recovery_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::Recovery_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mbf_msgs::action::Recovery_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mbf_msgs::action::Recovery_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mbf_msgs::action::Recovery_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mbf_msgs::action::Recovery_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mbf_msgs__action__Recovery_FeedbackMessage
    std::shared_ptr<mbf_msgs::action::Recovery_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mbf_msgs__action__Recovery_FeedbackMessage
    std::shared_ptr<mbf_msgs::action::Recovery_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Recovery_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const Recovery_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Recovery_FeedbackMessage_

// alias to use template instance with default allocator
using Recovery_FeedbackMessage =
  mbf_msgs::action::Recovery_FeedbackMessage_<std::allocator<void>>;

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

struct Recovery
{
  /// The goal message defined in the action definition.
  using Goal = mbf_msgs::action::Recovery_Goal;
  /// The result message defined in the action definition.
  using Result = mbf_msgs::action::Recovery_Result;
  /// The feedback message defined in the action definition.
  using Feedback = mbf_msgs::action::Recovery_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = mbf_msgs::action::Recovery_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = mbf_msgs::action::Recovery_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = mbf_msgs::action::Recovery_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct Recovery Recovery;

}  // namespace action

}  // namespace mbf_msgs

#endif  // MBF_MSGS__ACTION__DETAIL__RECOVERY__STRUCT_HPP_
