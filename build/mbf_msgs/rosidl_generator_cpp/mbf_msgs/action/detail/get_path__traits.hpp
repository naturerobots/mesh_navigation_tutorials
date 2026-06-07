// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mbf_msgs:action/GetPath.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/action/get_path.hpp"


#ifndef MBF_MSGS__ACTION__DETAIL__GET_PATH__TRAITS_HPP_
#define MBF_MSGS__ACTION__DETAIL__GET_PATH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mbf_msgs/action/detail/get_path__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'start_pose'
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace mbf_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const GetPath_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: use_start_pose
  {
    out << "use_start_pose: ";
    rosidl_generator_traits::value_to_yaml(msg.use_start_pose, out);
    out << ", ";
  }

  // member: start_pose
  {
    out << "start_pose: ";
    to_flow_style_yaml(msg.start_pose, out);
    out << ", ";
  }

  // member: target_pose
  {
    out << "target_pose: ";
    to_flow_style_yaml(msg.target_pose, out);
    out << ", ";
  }

  // member: tolerance
  {
    out << "tolerance: ";
    rosidl_generator_traits::value_to_yaml(msg.tolerance, out);
    out << ", ";
  }

  // member: planner
  {
    out << "planner: ";
    rosidl_generator_traits::value_to_yaml(msg.planner, out);
    out << ", ";
  }

  // member: concurrency_slot
  {
    out << "concurrency_slot: ";
    rosidl_generator_traits::value_to_yaml(msg.concurrency_slot, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetPath_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: use_start_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "use_start_pose: ";
    rosidl_generator_traits::value_to_yaml(msg.use_start_pose, out);
    out << "\n";
  }

  // member: start_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_pose:\n";
    to_block_style_yaml(msg.start_pose, out, indentation + 2);
  }

  // member: target_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_pose:\n";
    to_block_style_yaml(msg.target_pose, out, indentation + 2);
  }

  // member: tolerance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tolerance: ";
    rosidl_generator_traits::value_to_yaml(msg.tolerance, out);
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

  // member: concurrency_slot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "concurrency_slot: ";
    rosidl_generator_traits::value_to_yaml(msg.concurrency_slot, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetPath_Goal & msg, bool use_flow_style = false)
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
  const mbf_msgs::action::GetPath_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::action::GetPath_Goal & msg)
{
  return mbf_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::action::GetPath_Goal>()
{
  return "mbf_msgs::action::GetPath_Goal";
}

template<>
inline const char * name<mbf_msgs::action::GetPath_Goal>()
{
  return "mbf_msgs/action/GetPath_Goal";
}

template<>
struct has_fixed_size<mbf_msgs::action::GetPath_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mbf_msgs::action::GetPath_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mbf_msgs::action::GetPath_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'path'
#include "nav_msgs/msg/detail/path__traits.hpp"

namespace mbf_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const GetPath_Result & msg,
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

  // member: path
  {
    out << "path: ";
    to_flow_style_yaml(msg.path, out);
    out << ", ";
  }

  // member: cost
  {
    out << "cost: ";
    rosidl_generator_traits::value_to_yaml(msg.cost, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetPath_Result & msg,
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

  // member: path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "path:\n";
    to_block_style_yaml(msg.path, out, indentation + 2);
  }

  // member: cost
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cost: ";
    rosidl_generator_traits::value_to_yaml(msg.cost, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetPath_Result & msg, bool use_flow_style = false)
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
  const mbf_msgs::action::GetPath_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::action::GetPath_Result & msg)
{
  return mbf_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::action::GetPath_Result>()
{
  return "mbf_msgs::action::GetPath_Result";
}

template<>
inline const char * name<mbf_msgs::action::GetPath_Result>()
{
  return "mbf_msgs/action/GetPath_Result";
}

template<>
struct has_fixed_size<mbf_msgs::action::GetPath_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mbf_msgs::action::GetPath_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mbf_msgs::action::GetPath_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace mbf_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const GetPath_Feedback & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetPath_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetPath_Feedback & msg, bool use_flow_style = false)
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
  const mbf_msgs::action::GetPath_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::action::GetPath_Feedback & msg)
{
  return mbf_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::action::GetPath_Feedback>()
{
  return "mbf_msgs::action::GetPath_Feedback";
}

template<>
inline const char * name<mbf_msgs::action::GetPath_Feedback>()
{
  return "mbf_msgs/action/GetPath_Feedback";
}

template<>
struct has_fixed_size<mbf_msgs::action::GetPath_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mbf_msgs::action::GetPath_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mbf_msgs::action::GetPath_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "mbf_msgs/action/detail/get_path__traits.hpp"

namespace mbf_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const GetPath_SendGoal_Request & msg,
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
  const GetPath_SendGoal_Request & msg,
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

inline std::string to_yaml(const GetPath_SendGoal_Request & msg, bool use_flow_style = false)
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
  const mbf_msgs::action::GetPath_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::action::GetPath_SendGoal_Request & msg)
{
  return mbf_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::action::GetPath_SendGoal_Request>()
{
  return "mbf_msgs::action::GetPath_SendGoal_Request";
}

template<>
inline const char * name<mbf_msgs::action::GetPath_SendGoal_Request>()
{
  return "mbf_msgs/action/GetPath_SendGoal_Request";
}

template<>
struct has_fixed_size<mbf_msgs::action::GetPath_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<mbf_msgs::action::GetPath_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<mbf_msgs::action::GetPath_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<mbf_msgs::action::GetPath_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<mbf_msgs::action::GetPath_SendGoal_Request>
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
  const GetPath_SendGoal_Response & msg,
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
  const GetPath_SendGoal_Response & msg,
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

inline std::string to_yaml(const GetPath_SendGoal_Response & msg, bool use_flow_style = false)
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
  const mbf_msgs::action::GetPath_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::action::GetPath_SendGoal_Response & msg)
{
  return mbf_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::action::GetPath_SendGoal_Response>()
{
  return "mbf_msgs::action::GetPath_SendGoal_Response";
}

template<>
inline const char * name<mbf_msgs::action::GetPath_SendGoal_Response>()
{
  return "mbf_msgs/action/GetPath_SendGoal_Response";
}

template<>
struct has_fixed_size<mbf_msgs::action::GetPath_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<mbf_msgs::action::GetPath_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<mbf_msgs::action::GetPath_SendGoal_Response>
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
  const GetPath_SendGoal_Event & msg,
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
  const GetPath_SendGoal_Event & msg,
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

inline std::string to_yaml(const GetPath_SendGoal_Event & msg, bool use_flow_style = false)
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
  const mbf_msgs::action::GetPath_SendGoal_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::action::GetPath_SendGoal_Event & msg)
{
  return mbf_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::action::GetPath_SendGoal_Event>()
{
  return "mbf_msgs::action::GetPath_SendGoal_Event";
}

template<>
inline const char * name<mbf_msgs::action::GetPath_SendGoal_Event>()
{
  return "mbf_msgs/action/GetPath_SendGoal_Event";
}

template<>
struct has_fixed_size<mbf_msgs::action::GetPath_SendGoal_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mbf_msgs::action::GetPath_SendGoal_Event>
  : std::integral_constant<bool, has_bounded_size<mbf_msgs::action::GetPath_SendGoal_Request>::value && has_bounded_size<mbf_msgs::action::GetPath_SendGoal_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<mbf_msgs::action::GetPath_SendGoal_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mbf_msgs::action::GetPath_SendGoal>()
{
  return "mbf_msgs::action::GetPath_SendGoal";
}

template<>
inline const char * name<mbf_msgs::action::GetPath_SendGoal>()
{
  return "mbf_msgs/action/GetPath_SendGoal";
}

template<>
struct has_fixed_size<mbf_msgs::action::GetPath_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<mbf_msgs::action::GetPath_SendGoal_Request>::value &&
    has_fixed_size<mbf_msgs::action::GetPath_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<mbf_msgs::action::GetPath_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<mbf_msgs::action::GetPath_SendGoal_Request>::value &&
    has_bounded_size<mbf_msgs::action::GetPath_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<mbf_msgs::action::GetPath_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<mbf_msgs::action::GetPath_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mbf_msgs::action::GetPath_SendGoal_Response>
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
  const GetPath_GetResult_Request & msg,
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
  const GetPath_GetResult_Request & msg,
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

inline std::string to_yaml(const GetPath_GetResult_Request & msg, bool use_flow_style = false)
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
  const mbf_msgs::action::GetPath_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::action::GetPath_GetResult_Request & msg)
{
  return mbf_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::action::GetPath_GetResult_Request>()
{
  return "mbf_msgs::action::GetPath_GetResult_Request";
}

template<>
inline const char * name<mbf_msgs::action::GetPath_GetResult_Request>()
{
  return "mbf_msgs/action/GetPath_GetResult_Request";
}

template<>
struct has_fixed_size<mbf_msgs::action::GetPath_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<mbf_msgs::action::GetPath_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<mbf_msgs::action::GetPath_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "mbf_msgs/action/detail/get_path__traits.hpp"

namespace mbf_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const GetPath_GetResult_Response & msg,
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
  const GetPath_GetResult_Response & msg,
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

inline std::string to_yaml(const GetPath_GetResult_Response & msg, bool use_flow_style = false)
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
  const mbf_msgs::action::GetPath_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::action::GetPath_GetResult_Response & msg)
{
  return mbf_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::action::GetPath_GetResult_Response>()
{
  return "mbf_msgs::action::GetPath_GetResult_Response";
}

template<>
inline const char * name<mbf_msgs::action::GetPath_GetResult_Response>()
{
  return "mbf_msgs/action/GetPath_GetResult_Response";
}

template<>
struct has_fixed_size<mbf_msgs::action::GetPath_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<mbf_msgs::action::GetPath_Result>::value> {};

template<>
struct has_bounded_size<mbf_msgs::action::GetPath_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<mbf_msgs::action::GetPath_Result>::value> {};

template<>
struct is_message<mbf_msgs::action::GetPath_GetResult_Response>
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
  const GetPath_GetResult_Event & msg,
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
  const GetPath_GetResult_Event & msg,
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

inline std::string to_yaml(const GetPath_GetResult_Event & msg, bool use_flow_style = false)
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
  const mbf_msgs::action::GetPath_GetResult_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::action::GetPath_GetResult_Event & msg)
{
  return mbf_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::action::GetPath_GetResult_Event>()
{
  return "mbf_msgs::action::GetPath_GetResult_Event";
}

template<>
inline const char * name<mbf_msgs::action::GetPath_GetResult_Event>()
{
  return "mbf_msgs/action/GetPath_GetResult_Event";
}

template<>
struct has_fixed_size<mbf_msgs::action::GetPath_GetResult_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mbf_msgs::action::GetPath_GetResult_Event>
  : std::integral_constant<bool, has_bounded_size<mbf_msgs::action::GetPath_GetResult_Request>::value && has_bounded_size<mbf_msgs::action::GetPath_GetResult_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<mbf_msgs::action::GetPath_GetResult_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mbf_msgs::action::GetPath_GetResult>()
{
  return "mbf_msgs::action::GetPath_GetResult";
}

template<>
inline const char * name<mbf_msgs::action::GetPath_GetResult>()
{
  return "mbf_msgs/action/GetPath_GetResult";
}

template<>
struct has_fixed_size<mbf_msgs::action::GetPath_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<mbf_msgs::action::GetPath_GetResult_Request>::value &&
    has_fixed_size<mbf_msgs::action::GetPath_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<mbf_msgs::action::GetPath_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<mbf_msgs::action::GetPath_GetResult_Request>::value &&
    has_bounded_size<mbf_msgs::action::GetPath_GetResult_Response>::value
  >
{
};

template<>
struct is_service<mbf_msgs::action::GetPath_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<mbf_msgs::action::GetPath_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mbf_msgs::action::GetPath_GetResult_Response>
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
// #include "mbf_msgs/action/detail/get_path__traits.hpp"

namespace mbf_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const GetPath_FeedbackMessage & msg,
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
  const GetPath_FeedbackMessage & msg,
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

inline std::string to_yaml(const GetPath_FeedbackMessage & msg, bool use_flow_style = false)
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
  const mbf_msgs::action::GetPath_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::action::GetPath_FeedbackMessage & msg)
{
  return mbf_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::action::GetPath_FeedbackMessage>()
{
  return "mbf_msgs::action::GetPath_FeedbackMessage";
}

template<>
inline const char * name<mbf_msgs::action::GetPath_FeedbackMessage>()
{
  return "mbf_msgs/action/GetPath_FeedbackMessage";
}

template<>
struct has_fixed_size<mbf_msgs::action::GetPath_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<mbf_msgs::action::GetPath_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<mbf_msgs::action::GetPath_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<mbf_msgs::action::GetPath_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<mbf_msgs::action::GetPath_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<mbf_msgs::action::GetPath>
  : std::true_type
{
};

template<>
struct is_action_goal<mbf_msgs::action::GetPath_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<mbf_msgs::action::GetPath_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<mbf_msgs::action::GetPath_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // MBF_MSGS__ACTION__DETAIL__GET_PATH__TRAITS_HPP_
