// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mbf_msgs:action/MoveBase.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/action/move_base.hpp"


#ifndef MBF_MSGS__ACTION__DETAIL__MOVE_BASE__TRAITS_HPP_
#define MBF_MSGS__ACTION__DETAIL__MOVE_BASE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mbf_msgs/action/detail/move_base__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace mbf_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveBase_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: target_pose
  {
    out << "target_pose: ";
    to_flow_style_yaml(msg.target_pose, out);
    out << ", ";
  }

  // member: controller
  {
    out << "controller: ";
    rosidl_generator_traits::value_to_yaml(msg.controller, out);
    out << ", ";
  }

  // member: planner
  {
    out << "planner: ";
    rosidl_generator_traits::value_to_yaml(msg.planner, out);
    out << ", ";
  }

  // member: recovery_behaviors
  {
    if (msg.recovery_behaviors.size() == 0) {
      out << "recovery_behaviors: []";
    } else {
      out << "recovery_behaviors: [";
      size_t pending_items = msg.recovery_behaviors.size();
      for (auto item : msg.recovery_behaviors) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveBase_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: target_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_pose:\n";
    to_block_style_yaml(msg.target_pose, out, indentation + 2);
  }

  // member: controller
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "controller: ";
    rosidl_generator_traits::value_to_yaml(msg.controller, out);
    out << "\n";
  }

  // member: planner
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "planner: ";
    rosidl_generator_traits::value_to_yaml(msg.planner, out);
    out << "\n";
  }

  // member: recovery_behaviors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.recovery_behaviors.size() == 0) {
      out << "recovery_behaviors: []\n";
    } else {
      out << "recovery_behaviors:\n";
      for (auto item : msg.recovery_behaviors) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveBase_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mbf_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mbf_msgs::action::MoveBase_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::action::MoveBase_Goal & msg)
{
  return mbf_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::action::MoveBase_Goal>()
{
  return "mbf_msgs::action::MoveBase_Goal";
}

template<>
inline const char * name<mbf_msgs::action::MoveBase_Goal>()
{
  return "mbf_msgs/action/MoveBase_Goal";
}

template<>
struct has_fixed_size<mbf_msgs::action::MoveBase_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mbf_msgs::action::MoveBase_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mbf_msgs::action::MoveBase_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'final_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace mbf_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveBase_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: outcome
  {
    out << "outcome: ";
    rosidl_generator_traits::value_to_yaml(msg.outcome, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << ", ";
  }

  // member: dist_to_goal
  {
    out << "dist_to_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.dist_to_goal, out);
    out << ", ";
  }

  // member: angle_to_goal
  {
    out << "angle_to_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_to_goal, out);
    out << ", ";
  }

  // member: final_pose
  {
    out << "final_pose: ";
    to_flow_style_yaml(msg.final_pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveBase_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: outcome
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "outcome: ";
    rosidl_generator_traits::value_to_yaml(msg.outcome, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }

  // member: dist_to_goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dist_to_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.dist_to_goal, out);
    out << "\n";
  }

  // member: angle_to_goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle_to_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_to_goal, out);
    out << "\n";
  }

  // member: final_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_pose:\n";
    to_block_style_yaml(msg.final_pose, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveBase_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mbf_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mbf_msgs::action::MoveBase_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::action::MoveBase_Result & msg)
{
  return mbf_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::action::MoveBase_Result>()
{
  return "mbf_msgs::action::MoveBase_Result";
}

template<>
inline const char * name<mbf_msgs::action::MoveBase_Result>()
{
  return "mbf_msgs/action/MoveBase_Result";
}

template<>
struct has_fixed_size<mbf_msgs::action::MoveBase_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mbf_msgs::action::MoveBase_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mbf_msgs::action::MoveBase_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'current_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"
// Member 'last_cmd_vel'
#include "geometry_msgs/msg/detail/twist_stamped__traits.hpp"

namespace mbf_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveBase_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: outcome
  {
    out << "outcome: ";
    rosidl_generator_traits::value_to_yaml(msg.outcome, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << ", ";
  }

  // member: dist_to_goal
  {
    out << "dist_to_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.dist_to_goal, out);
    out << ", ";
  }

  // member: angle_to_goal
  {
    out << "angle_to_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_to_goal, out);
    out << ", ";
  }

  // member: current_pose
  {
    out << "current_pose: ";
    to_flow_style_yaml(msg.current_pose, out);
    out << ", ";
  }

  // member: last_cmd_vel
  {
    out << "last_cmd_vel: ";
    to_flow_style_yaml(msg.last_cmd_vel, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveBase_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: outcome
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "outcome: ";
    rosidl_generator_traits::value_to_yaml(msg.outcome, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }

  // member: dist_to_goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dist_to_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.dist_to_goal, out);
    out << "\n";
  }

  // member: angle_to_goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle_to_goal: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_to_goal, out);
    out << "\n";
  }

  // member: current_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_pose:\n";
    to_block_style_yaml(msg.current_pose, out, indentation + 2);
  }

  // member: last_cmd_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "last_cmd_vel:\n";
    to_block_style_yaml(msg.last_cmd_vel, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveBase_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mbf_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mbf_msgs::action::MoveBase_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::action::MoveBase_Feedback & msg)
{
  return mbf_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::action::MoveBase_Feedback>()
{
  return "mbf_msgs::action::MoveBase_Feedback";
}

template<>
inline const char * name<mbf_msgs::action::MoveBase_Feedback>()
{
  return "mbf_msgs/action/MoveBase_Feedback";
}

template<>
struct has_fixed_size<mbf_msgs::action::MoveBase_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mbf_msgs::action::MoveBase_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mbf_msgs::action::MoveBase_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "mbf_msgs/action/detail/move_base__traits.hpp"

namespace mbf_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveBase_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveBase_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveBase_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mbf_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mbf_msgs::action::MoveBase_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::action::MoveBase_SendGoal_Request & msg)
{
  return mbf_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::action::MoveBase_SendGoal_Request>()
{
  return "mbf_msgs::action::MoveBase_SendGoal_Request";
}

template<>
inline const char * name<mbf_msgs::action::MoveBase_SendGoal_Request>()
{
  return "mbf_msgs/action/MoveBase_SendGoal_Request";
}

template<>
struct has_fixed_size<mbf_msgs::action::MoveBase_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<mbf_msgs::action::MoveBase_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<mbf_msgs::action::MoveBase_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<mbf_msgs::action::MoveBase_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<mbf_msgs::action::MoveBase_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace mbf_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveBase_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveBase_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveBase_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mbf_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mbf_msgs::action::MoveBase_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::action::MoveBase_SendGoal_Response & msg)
{
  return mbf_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::action::MoveBase_SendGoal_Response>()
{
  return "mbf_msgs::action::MoveBase_SendGoal_Response";
}

template<>
inline const char * name<mbf_msgs::action::MoveBase_SendGoal_Response>()
{
  return "mbf_msgs/action/MoveBase_SendGoal_Response";
}

template<>
struct has_fixed_size<mbf_msgs::action::MoveBase_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<mbf_msgs::action::MoveBase_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<mbf_msgs::action::MoveBase_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace mbf_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveBase_SendGoal_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveBase_SendGoal_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveBase_SendGoal_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mbf_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mbf_msgs::action::MoveBase_SendGoal_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::action::MoveBase_SendGoal_Event & msg)
{
  return mbf_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::action::MoveBase_SendGoal_Event>()
{
  return "mbf_msgs::action::MoveBase_SendGoal_Event";
}

template<>
inline const char * name<mbf_msgs::action::MoveBase_SendGoal_Event>()
{
  return "mbf_msgs/action/MoveBase_SendGoal_Event";
}

template<>
struct has_fixed_size<mbf_msgs::action::MoveBase_SendGoal_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mbf_msgs::action::MoveBase_SendGoal_Event>
  : std::integral_constant<bool, has_bounded_size<mbf_msgs::action::MoveBase_SendGoal_Request>::value && has_bounded_size<mbf_msgs::action::MoveBase_SendGoal_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<mbf_msgs::action::MoveBase_SendGoal_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mbf_msgs::action::MoveBase_SendGoal>()
{
  return "mbf_msgs::action::MoveBase_SendGoal";
}

template<>
inline const char * name<mbf_msgs::action::MoveBase_SendGoal>()
{
  return "mbf_msgs/action/MoveBase_SendGoal";
}

template<>
struct has_fixed_size<mbf_msgs::action::MoveBase_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<mbf_msgs::action::MoveBase_SendGoal_Request>::value &&
    has_fixed_size<mbf_msgs::action::MoveBase_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<mbf_msgs::action::MoveBase_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<mbf_msgs::action::MoveBase_SendGoal_Request>::value &&
    has_bounded_size<mbf_msgs::action::MoveBase_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<mbf_msgs::action::MoveBase_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<mbf_msgs::action::MoveBase_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mbf_msgs::action::MoveBase_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace mbf_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveBase_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveBase_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveBase_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mbf_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mbf_msgs::action::MoveBase_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::action::MoveBase_GetResult_Request & msg)
{
  return mbf_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::action::MoveBase_GetResult_Request>()
{
  return "mbf_msgs::action::MoveBase_GetResult_Request";
}

template<>
inline const char * name<mbf_msgs::action::MoveBase_GetResult_Request>()
{
  return "mbf_msgs/action/MoveBase_GetResult_Request";
}

template<>
struct has_fixed_size<mbf_msgs::action::MoveBase_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<mbf_msgs::action::MoveBase_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<mbf_msgs::action::MoveBase_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "mbf_msgs/action/detail/move_base__traits.hpp"

namespace mbf_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveBase_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveBase_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveBase_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mbf_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mbf_msgs::action::MoveBase_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::action::MoveBase_GetResult_Response & msg)
{
  return mbf_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::action::MoveBase_GetResult_Response>()
{
  return "mbf_msgs::action::MoveBase_GetResult_Response";
}

template<>
inline const char * name<mbf_msgs::action::MoveBase_GetResult_Response>()
{
  return "mbf_msgs/action/MoveBase_GetResult_Response";
}

template<>
struct has_fixed_size<mbf_msgs::action::MoveBase_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<mbf_msgs::action::MoveBase_Result>::value> {};

template<>
struct has_bounded_size<mbf_msgs::action::MoveBase_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<mbf_msgs::action::MoveBase_Result>::value> {};

template<>
struct is_message<mbf_msgs::action::MoveBase_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace mbf_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveBase_GetResult_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveBase_GetResult_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveBase_GetResult_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mbf_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mbf_msgs::action::MoveBase_GetResult_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::action::MoveBase_GetResult_Event & msg)
{
  return mbf_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::action::MoveBase_GetResult_Event>()
{
  return "mbf_msgs::action::MoveBase_GetResult_Event";
}

template<>
inline const char * name<mbf_msgs::action::MoveBase_GetResult_Event>()
{
  return "mbf_msgs/action/MoveBase_GetResult_Event";
}

template<>
struct has_fixed_size<mbf_msgs::action::MoveBase_GetResult_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mbf_msgs::action::MoveBase_GetResult_Event>
  : std::integral_constant<bool, has_bounded_size<mbf_msgs::action::MoveBase_GetResult_Request>::value && has_bounded_size<mbf_msgs::action::MoveBase_GetResult_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<mbf_msgs::action::MoveBase_GetResult_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mbf_msgs::action::MoveBase_GetResult>()
{
  return "mbf_msgs::action::MoveBase_GetResult";
}

template<>
inline const char * name<mbf_msgs::action::MoveBase_GetResult>()
{
  return "mbf_msgs/action/MoveBase_GetResult";
}

template<>
struct has_fixed_size<mbf_msgs::action::MoveBase_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<mbf_msgs::action::MoveBase_GetResult_Request>::value &&
    has_fixed_size<mbf_msgs::action::MoveBase_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<mbf_msgs::action::MoveBase_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<mbf_msgs::action::MoveBase_GetResult_Request>::value &&
    has_bounded_size<mbf_msgs::action::MoveBase_GetResult_Response>::value
  >
{
};

template<>
struct is_service<mbf_msgs::action::MoveBase_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<mbf_msgs::action::MoveBase_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mbf_msgs::action::MoveBase_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "mbf_msgs/action/detail/move_base__traits.hpp"

namespace mbf_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const MoveBase_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveBase_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveBase_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mbf_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mbf_msgs::action::MoveBase_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::action::MoveBase_FeedbackMessage & msg)
{
  return mbf_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::action::MoveBase_FeedbackMessage>()
{
  return "mbf_msgs::action::MoveBase_FeedbackMessage";
}

template<>
inline const char * name<mbf_msgs::action::MoveBase_FeedbackMessage>()
{
  return "mbf_msgs/action/MoveBase_FeedbackMessage";
}

template<>
struct has_fixed_size<mbf_msgs::action::MoveBase_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<mbf_msgs::action::MoveBase_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<mbf_msgs::action::MoveBase_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<mbf_msgs::action::MoveBase_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<mbf_msgs::action::MoveBase_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<mbf_msgs::action::MoveBase>
  : std::true_type
{
};

template<>
struct is_action_goal<mbf_msgs::action::MoveBase_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<mbf_msgs::action::MoveBase_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<mbf_msgs::action::MoveBase_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // MBF_MSGS__ACTION__DETAIL__MOVE_BASE__TRAITS_HPP_
