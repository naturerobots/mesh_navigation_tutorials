// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mesh_msgs:srv/GetMaterials.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/srv/get_materials.hpp"


#ifndef MESH_MSGS__SRV__DETAIL__GET_MATERIALS__TRAITS_HPP_
#define MESH_MSGS__SRV__DETAIL__GET_MATERIALS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mesh_msgs/srv/detail/get_materials__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mesh_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetMaterials_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: uuid
  {
    out << "uuid: ";
    rosidl_generator_traits::value_to_yaml(msg.uuid, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetMaterials_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: uuid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "uuid: ";
    rosidl_generator_traits::value_to_yaml(msg.uuid, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetMaterials_Request & msg, bool use_flow_style = false)
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

}  // namespace mesh_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mesh_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mesh_msgs::srv::GetMaterials_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mesh_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mesh_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mesh_msgs::srv::GetMaterials_Request & msg)
{
  return mesh_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mesh_msgs::srv::GetMaterials_Request>()
{
  return "mesh_msgs::srv::GetMaterials_Request";
}

template<>
inline const char * name<mesh_msgs::srv::GetMaterials_Request>()
{
  return "mesh_msgs/srv/GetMaterials_Request";
}

template<>
struct has_fixed_size<mesh_msgs::srv::GetMaterials_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mesh_msgs::srv::GetMaterials_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mesh_msgs::srv::GetMaterials_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'mesh_materials_stamped'
#include "mesh_msgs/msg/detail/mesh_materials_stamped__traits.hpp"

namespace mesh_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetMaterials_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: mesh_materials_stamped
  {
    out << "mesh_materials_stamped: ";
    to_flow_style_yaml(msg.mesh_materials_stamped, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetMaterials_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: mesh_materials_stamped
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mesh_materials_stamped:\n";
    to_block_style_yaml(msg.mesh_materials_stamped, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetMaterials_Response & msg, bool use_flow_style = false)
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

}  // namespace mesh_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mesh_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mesh_msgs::srv::GetMaterials_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mesh_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mesh_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mesh_msgs::srv::GetMaterials_Response & msg)
{
  return mesh_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mesh_msgs::srv::GetMaterials_Response>()
{
  return "mesh_msgs::srv::GetMaterials_Response";
}

template<>
inline const char * name<mesh_msgs::srv::GetMaterials_Response>()
{
  return "mesh_msgs/srv/GetMaterials_Response";
}

template<>
struct has_fixed_size<mesh_msgs::srv::GetMaterials_Response>
  : std::integral_constant<bool, has_fixed_size<mesh_msgs::msg::MeshMaterialsStamped>::value> {};

template<>
struct has_bounded_size<mesh_msgs::srv::GetMaterials_Response>
  : std::integral_constant<bool, has_bounded_size<mesh_msgs::msg::MeshMaterialsStamped>::value> {};

template<>
struct is_message<mesh_msgs::srv::GetMaterials_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace mesh_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetMaterials_Event & msg,
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
  const GetMaterials_Event & msg,
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

inline std::string to_yaml(const GetMaterials_Event & msg, bool use_flow_style = false)
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

}  // namespace mesh_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mesh_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mesh_msgs::srv::GetMaterials_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  mesh_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mesh_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mesh_msgs::srv::GetMaterials_Event & msg)
{
  return mesh_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mesh_msgs::srv::GetMaterials_Event>()
{
  return "mesh_msgs::srv::GetMaterials_Event";
}

template<>
inline const char * name<mesh_msgs::srv::GetMaterials_Event>()
{
  return "mesh_msgs/srv/GetMaterials_Event";
}

template<>
struct has_fixed_size<mesh_msgs::srv::GetMaterials_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mesh_msgs::srv::GetMaterials_Event>
  : std::integral_constant<bool, has_bounded_size<mesh_msgs::srv::GetMaterials_Request>::value && has_bounded_size<mesh_msgs::srv::GetMaterials_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<mesh_msgs::srv::GetMaterials_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mesh_msgs::srv::GetMaterials>()
{
  return "mesh_msgs::srv::GetMaterials";
}

template<>
inline const char * name<mesh_msgs::srv::GetMaterials>()
{
  return "mesh_msgs/srv/GetMaterials";
}

template<>
struct has_fixed_size<mesh_msgs::srv::GetMaterials>
  : std::integral_constant<
    bool,
    has_fixed_size<mesh_msgs::srv::GetMaterials_Request>::value &&
    has_fixed_size<mesh_msgs::srv::GetMaterials_Response>::value
  >
{
};

template<>
struct has_bounded_size<mesh_msgs::srv::GetMaterials>
  : std::integral_constant<
    bool,
    has_bounded_size<mesh_msgs::srv::GetMaterials_Request>::value &&
    has_bounded_size<mesh_msgs::srv::GetMaterials_Response>::value
  >
{
};

template<>
struct is_service<mesh_msgs::srv::GetMaterials>
  : std::true_type
{
};

template<>
struct is_service_request<mesh_msgs::srv::GetMaterials_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mesh_msgs::srv::GetMaterials_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MESH_MSGS__SRV__DETAIL__GET_MATERIALS__TRAITS_HPP_
