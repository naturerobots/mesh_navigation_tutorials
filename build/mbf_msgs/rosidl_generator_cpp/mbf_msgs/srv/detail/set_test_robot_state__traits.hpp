// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mbf_msgs:srv/SetTestRobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/srv/set_test_robot_state.hpp"


#ifndef MBF_MSGS__SRV__DETAIL__SET_TEST_ROBOT_STATE__TRAITS_HPP_
#define MBF_MSGS__SRV__DETAIL__SET_TEST_ROBOT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mbf_msgs/srv/detail/set_test_robot_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'transform'
#include "geometry_msgs/msg/detail/transform_stamped__traits.hpp"
// Member 'velocity_robot'
#include "geometry_msgs/msg/detail/twist_stamped__traits.hpp"

namespace mbf_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetTestRobotState_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: transform
  {
    out << "transform: ";
    to_flow_style_yaml(msg.transform, out);
    out << ", ";
  }

  // member: set_velocity
  {
    out << "set_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.set_velocity, out);
    out << ", ";
  }

  // member: velocity_robot
  {
    out << "velocity_robot: ";
    to_flow_style_yaml(msg.velocity_robot, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetTestRobotState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: transform
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "transform:\n";
    to_block_style_yaml(msg.transform, out, indentation + 2);
  }

  // member: set_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "set_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.set_velocity, out);
    out << "\n";
  }

  // member: velocity_robot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_robot:\n";
    to_block_style_yaml(msg.velocity_robot, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetTestRobotState_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace mbf_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mbf_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mbf_msgs::srv::SetTestRobotState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::srv::SetTestRobotState_Request & msg)
{
  return mbf_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::srv::SetTestRobotState_Request>()
{
  return "mbf_msgs::srv::SetTestRobotState_Request";
}

template<>
inline const char * name<mbf_msgs::srv::SetTestRobotState_Request>()
{
  return "mbf_msgs/srv/SetTestRobotState_Request";
}

template<>
struct has_fixed_size<mbf_msgs::srv::SetTestRobotState_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::TransformStamped>::value && has_fixed_size<geometry_msgs::msg::TwistStamped>::value> {};

template<>
struct has_bounded_size<mbf_msgs::srv::SetTestRobotState_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::TransformStamped>::value && has_bounded_size<geometry_msgs::msg::TwistStamped>::value> {};

template<>
struct is_message<mbf_msgs::srv::SetTestRobotState_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace mbf_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetTestRobotState_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetTestRobotState_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetTestRobotState_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace mbf_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mbf_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mbf_msgs::srv::SetTestRobotState_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::srv::SetTestRobotState_Response & msg)
{
  return mbf_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::srv::SetTestRobotState_Response>()
{
  return "mbf_msgs::srv::SetTestRobotState_Response";
}

template<>
inline const char * name<mbf_msgs::srv::SetTestRobotState_Response>()
{
  return "mbf_msgs/srv/SetTestRobotState_Response";
}

template<>
struct has_fixed_size<mbf_msgs::srv::SetTestRobotState_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mbf_msgs::srv::SetTestRobotState_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mbf_msgs::srv::SetTestRobotState_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace mbf_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetTestRobotState_Event & msg,
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
  const SetTestRobotState_Event & msg,
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

inline std::string to_yaml(const SetTestRobotState_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace mbf_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mbf_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mbf_msgs::srv::SetTestRobotState_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::srv::SetTestRobotState_Event & msg)
{
  return mbf_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::srv::SetTestRobotState_Event>()
{
  return "mbf_msgs::srv::SetTestRobotState_Event";
}

template<>
inline const char * name<mbf_msgs::srv::SetTestRobotState_Event>()
{
  return "mbf_msgs/srv/SetTestRobotState_Event";
}

template<>
struct has_fixed_size<mbf_msgs::srv::SetTestRobotState_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mbf_msgs::srv::SetTestRobotState_Event>
  : std::integral_constant<bool, has_bounded_size<mbf_msgs::srv::SetTestRobotState_Request>::value && has_bounded_size<mbf_msgs::srv::SetTestRobotState_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<mbf_msgs::srv::SetTestRobotState_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mbf_msgs::srv::SetTestRobotState>()
{
  return "mbf_msgs::srv::SetTestRobotState";
}

template<>
inline const char * name<mbf_msgs::srv::SetTestRobotState>()
{
  return "mbf_msgs/srv/SetTestRobotState";
}

template<>
struct has_fixed_size<mbf_msgs::srv::SetTestRobotState>
  : std::integral_constant<
    bool,
    has_fixed_size<mbf_msgs::srv::SetTestRobotState_Request>::value &&
    has_fixed_size<mbf_msgs::srv::SetTestRobotState_Response>::value
  >
{
};

template<>
struct has_bounded_size<mbf_msgs::srv::SetTestRobotState>
  : std::integral_constant<
    bool,
    has_bounded_size<mbf_msgs::srv::SetTestRobotState_Request>::value &&
    has_bounded_size<mbf_msgs::srv::SetTestRobotState_Response>::value
  >
{
};

template<>
struct is_service<mbf_msgs::srv::SetTestRobotState>
  : std::true_type
{
};

template<>
struct is_service_request<mbf_msgs::srv::SetTestRobotState_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mbf_msgs::srv::SetTestRobotState_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MBF_MSGS__SRV__DETAIL__SET_TEST_ROBOT_STATE__TRAITS_HPP_
