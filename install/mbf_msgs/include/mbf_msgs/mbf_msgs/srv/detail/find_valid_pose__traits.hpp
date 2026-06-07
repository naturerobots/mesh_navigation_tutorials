// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mbf_msgs:srv/FindValidPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/srv/find_valid_pose.hpp"


#ifndef MBF_MSGS__SRV__DETAIL__FIND_VALID_POSE__TRAITS_HPP_
#define MBF_MSGS__SRV__DETAIL__FIND_VALID_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mbf_msgs/srv/detail/find_valid_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace mbf_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const FindValidPose_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: safety_dist
  {
    out << "safety_dist: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_dist, out);
    out << ", ";
  }

  // member: dist_tolerance
  {
    out << "dist_tolerance: ";
    rosidl_generator_traits::value_to_yaml(msg.dist_tolerance, out);
    out << ", ";
  }

  // member: angle_tolerance
  {
    out << "angle_tolerance: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_tolerance, out);
    out << ", ";
  }

  // member: costmap
  {
    out << "costmap: ";
    rosidl_generator_traits::value_to_yaml(msg.costmap, out);
    out << ", ";
  }

  // member: current_pose
  {
    out << "current_pose: ";
    rosidl_generator_traits::value_to_yaml(msg.current_pose, out);
    out << ", ";
  }

  // member: use_padded_fp
  {
    out << "use_padded_fp: ";
    rosidl_generator_traits::value_to_yaml(msg.use_padded_fp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FindValidPose_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: safety_dist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safety_dist: ";
    rosidl_generator_traits::value_to_yaml(msg.safety_dist, out);
    out << "\n";
  }

  // member: dist_tolerance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dist_tolerance: ";
    rosidl_generator_traits::value_to_yaml(msg.dist_tolerance, out);
    out << "\n";
  }

  // member: angle_tolerance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle_tolerance: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_tolerance, out);
    out << "\n";
  }

  // member: costmap
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "costmap: ";
    rosidl_generator_traits::value_to_yaml(msg.costmap, out);
    out << "\n";
  }

  // member: current_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_pose: ";
    rosidl_generator_traits::value_to_yaml(msg.current_pose, out);
    out << "\n";
  }

  // member: use_padded_fp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "use_padded_fp: ";
    rosidl_generator_traits::value_to_yaml(msg.use_padded_fp, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FindValidPose_Request & msg, bool use_flow_style = false)
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
  const mbf_msgs::srv::FindValidPose_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::srv::FindValidPose_Request & msg)
{
  return mbf_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::srv::FindValidPose_Request>()
{
  return "mbf_msgs::srv::FindValidPose_Request";
}

template<>
inline const char * name<mbf_msgs::srv::FindValidPose_Request>()
{
  return "mbf_msgs/srv/FindValidPose_Request";
}

template<>
struct has_fixed_size<mbf_msgs::srv::FindValidPose_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct has_bounded_size<mbf_msgs::srv::FindValidPose_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct is_message<mbf_msgs::srv::FindValidPose_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace mbf_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const FindValidPose_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: cost
  {
    out << "cost: ";
    rosidl_generator_traits::value_to_yaml(msg.cost, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FindValidPose_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
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

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FindValidPose_Response & msg, bool use_flow_style = false)
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
  const mbf_msgs::srv::FindValidPose_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::srv::FindValidPose_Response & msg)
{
  return mbf_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::srv::FindValidPose_Response>()
{
  return "mbf_msgs::srv::FindValidPose_Response";
}

template<>
inline const char * name<mbf_msgs::srv::FindValidPose_Response>()
{
  return "mbf_msgs/srv/FindValidPose_Response";
}

template<>
struct has_fixed_size<mbf_msgs::srv::FindValidPose_Response>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct has_bounded_size<mbf_msgs::srv::FindValidPose_Response>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct is_message<mbf_msgs::srv::FindValidPose_Response>
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
  const FindValidPose_Event & msg,
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
  const FindValidPose_Event & msg,
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

inline std::string to_yaml(const FindValidPose_Event & msg, bool use_flow_style = false)
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
  const mbf_msgs::srv::FindValidPose_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbf_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbf_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mbf_msgs::srv::FindValidPose_Event & msg)
{
  return mbf_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mbf_msgs::srv::FindValidPose_Event>()
{
  return "mbf_msgs::srv::FindValidPose_Event";
}

template<>
inline const char * name<mbf_msgs::srv::FindValidPose_Event>()
{
  return "mbf_msgs/srv/FindValidPose_Event";
}

template<>
struct has_fixed_size<mbf_msgs::srv::FindValidPose_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mbf_msgs::srv::FindValidPose_Event>
  : std::integral_constant<bool, has_bounded_size<mbf_msgs::srv::FindValidPose_Request>::value && has_bounded_size<mbf_msgs::srv::FindValidPose_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<mbf_msgs::srv::FindValidPose_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mbf_msgs::srv::FindValidPose>()
{
  return "mbf_msgs::srv::FindValidPose";
}

template<>
inline const char * name<mbf_msgs::srv::FindValidPose>()
{
  return "mbf_msgs/srv/FindValidPose";
}

template<>
struct has_fixed_size<mbf_msgs::srv::FindValidPose>
  : std::integral_constant<
    bool,
    has_fixed_size<mbf_msgs::srv::FindValidPose_Request>::value &&
    has_fixed_size<mbf_msgs::srv::FindValidPose_Response>::value
  >
{
};

template<>
struct has_bounded_size<mbf_msgs::srv::FindValidPose>
  : std::integral_constant<
    bool,
    has_bounded_size<mbf_msgs::srv::FindValidPose_Request>::value &&
    has_bounded_size<mbf_msgs::srv::FindValidPose_Response>::value
  >
{
};

template<>
struct is_service<mbf_msgs::srv::FindValidPose>
  : std::true_type
{
};

template<>
struct is_service_request<mbf_msgs::srv::FindValidPose_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mbf_msgs::srv::FindValidPose_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MBF_MSGS__SRV__DETAIL__FIND_VALID_POSE__TRAITS_HPP_
