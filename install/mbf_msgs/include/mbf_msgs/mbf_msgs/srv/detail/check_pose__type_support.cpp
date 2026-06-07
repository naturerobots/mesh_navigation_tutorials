// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mbf_msgs:srv/CheckPose.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mbf_msgs/srv/detail/check_pose__functions.h"
#include "mbf_msgs/srv/detail/check_pose__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mbf_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void CheckPose_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mbf_msgs::srv::CheckPose_Request(_init);
}

void CheckPose_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mbf_msgs::srv::CheckPose_Request *>(message_memory);
  typed_message->~CheckPose_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CheckPose_Request_message_member_array[8] = {
  {
    "pose",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::PoseStamped>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs::srv::CheckPose_Request, pose),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "safety_dist",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs::srv::CheckPose_Request, safety_dist),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "lethal_cost_mult",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs::srv::CheckPose_Request, lethal_cost_mult),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "inscrib_cost_mult",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs::srv::CheckPose_Request, inscrib_cost_mult),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "unknown_cost_mult",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs::srv::CheckPose_Request, unknown_cost_mult),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "costmap",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs::srv::CheckPose_Request, costmap),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "current_pose",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs::srv::CheckPose_Request, current_pose),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "use_padded_fp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs::srv::CheckPose_Request, use_padded_fp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CheckPose_Request_message_members = {
  "mbf_msgs::srv",  // message namespace
  "CheckPose_Request",  // message name
  8,  // number of fields
  sizeof(mbf_msgs::srv::CheckPose_Request),
  false,  // has_any_key_member_
  CheckPose_Request_message_member_array,  // message members
  CheckPose_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  CheckPose_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CheckPose_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CheckPose_Request_message_members,
  get_message_typesupport_handle_function,
  &mbf_msgs__srv__CheckPose_Request__get_type_hash,
  &mbf_msgs__srv__CheckPose_Request__get_type_description,
  &mbf_msgs__srv__CheckPose_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace mbf_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::srv::CheckPose_Request>()
{
  return &::mbf_msgs::srv::rosidl_typesupport_introspection_cpp::CheckPose_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, srv, CheckPose_Request)() {
  return &::mbf_msgs::srv::rosidl_typesupport_introspection_cpp::CheckPose_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "mbf_msgs/srv/detail/check_pose__functions.h"
// already included above
// #include "mbf_msgs/srv/detail/check_pose__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mbf_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void CheckPose_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mbf_msgs::srv::CheckPose_Response(_init);
}

void CheckPose_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mbf_msgs::srv::CheckPose_Response *>(message_memory);
  typed_message->~CheckPose_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CheckPose_Response_message_member_array[2] = {
  {
    "state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs::srv::CheckPose_Response, state),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "cost",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs::srv::CheckPose_Response, cost),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CheckPose_Response_message_members = {
  "mbf_msgs::srv",  // message namespace
  "CheckPose_Response",  // message name
  2,  // number of fields
  sizeof(mbf_msgs::srv::CheckPose_Response),
  false,  // has_any_key_member_
  CheckPose_Response_message_member_array,  // message members
  CheckPose_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  CheckPose_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CheckPose_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CheckPose_Response_message_members,
  get_message_typesupport_handle_function,
  &mbf_msgs__srv__CheckPose_Response__get_type_hash,
  &mbf_msgs__srv__CheckPose_Response__get_type_description,
  &mbf_msgs__srv__CheckPose_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace mbf_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::srv::CheckPose_Response>()
{
  return &::mbf_msgs::srv::rosidl_typesupport_introspection_cpp::CheckPose_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, srv, CheckPose_Response)() {
  return &::mbf_msgs::srv::rosidl_typesupport_introspection_cpp::CheckPose_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "mbf_msgs/srv/detail/check_pose__functions.h"
// already included above
// #include "mbf_msgs/srv/detail/check_pose__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mbf_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void CheckPose_Event_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mbf_msgs::srv::CheckPose_Event(_init);
}

void CheckPose_Event_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mbf_msgs::srv::CheckPose_Event *>(message_memory);
  typed_message->~CheckPose_Event();
}

size_t size_function__CheckPose_Event__request(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<mbf_msgs::srv::CheckPose_Request> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CheckPose_Event__request(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<mbf_msgs::srv::CheckPose_Request> *>(untyped_member);
  return &member[index];
}

void * get_function__CheckPose_Event__request(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<mbf_msgs::srv::CheckPose_Request> *>(untyped_member);
  return &member[index];
}

void fetch_function__CheckPose_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const mbf_msgs::srv::CheckPose_Request *>(
    get_const_function__CheckPose_Event__request(untyped_member, index));
  auto & value = *reinterpret_cast<mbf_msgs::srv::CheckPose_Request *>(untyped_value);
  value = item;
}

void assign_function__CheckPose_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<mbf_msgs::srv::CheckPose_Request *>(
    get_function__CheckPose_Event__request(untyped_member, index));
  const auto & value = *reinterpret_cast<const mbf_msgs::srv::CheckPose_Request *>(untyped_value);
  item = value;
}

void resize_function__CheckPose_Event__request(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<mbf_msgs::srv::CheckPose_Request> *>(untyped_member);
  member->resize(size);
}

size_t size_function__CheckPose_Event__response(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<mbf_msgs::srv::CheckPose_Response> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CheckPose_Event__response(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<mbf_msgs::srv::CheckPose_Response> *>(untyped_member);
  return &member[index];
}

void * get_function__CheckPose_Event__response(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<mbf_msgs::srv::CheckPose_Response> *>(untyped_member);
  return &member[index];
}

void fetch_function__CheckPose_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const mbf_msgs::srv::CheckPose_Response *>(
    get_const_function__CheckPose_Event__response(untyped_member, index));
  auto & value = *reinterpret_cast<mbf_msgs::srv::CheckPose_Response *>(untyped_value);
  value = item;
}

void assign_function__CheckPose_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<mbf_msgs::srv::CheckPose_Response *>(
    get_function__CheckPose_Event__response(untyped_member, index));
  const auto & value = *reinterpret_cast<const mbf_msgs::srv::CheckPose_Response *>(untyped_value);
  item = value;
}

void resize_function__CheckPose_Event__response(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<mbf_msgs::srv::CheckPose_Response> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CheckPose_Event_message_member_array[3] = {
  {
    "info",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<service_msgs::msg::ServiceEventInfo>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs::srv::CheckPose_Event, info),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "request",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<mbf_msgs::srv::CheckPose_Request>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(mbf_msgs::srv::CheckPose_Event, request),  // bytes offset in struct
    nullptr,  // default value
    size_function__CheckPose_Event__request,  // size() function pointer
    get_const_function__CheckPose_Event__request,  // get_const(index) function pointer
    get_function__CheckPose_Event__request,  // get(index) function pointer
    fetch_function__CheckPose_Event__request,  // fetch(index, &value) function pointer
    assign_function__CheckPose_Event__request,  // assign(index, value) function pointer
    resize_function__CheckPose_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<mbf_msgs::srv::CheckPose_Response>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(mbf_msgs::srv::CheckPose_Event, response),  // bytes offset in struct
    nullptr,  // default value
    size_function__CheckPose_Event__response,  // size() function pointer
    get_const_function__CheckPose_Event__response,  // get_const(index) function pointer
    get_function__CheckPose_Event__response,  // get(index) function pointer
    fetch_function__CheckPose_Event__response,  // fetch(index, &value) function pointer
    assign_function__CheckPose_Event__response,  // assign(index, value) function pointer
    resize_function__CheckPose_Event__response  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CheckPose_Event_message_members = {
  "mbf_msgs::srv",  // message namespace
  "CheckPose_Event",  // message name
  3,  // number of fields
  sizeof(mbf_msgs::srv::CheckPose_Event),
  false,  // has_any_key_member_
  CheckPose_Event_message_member_array,  // message members
  CheckPose_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  CheckPose_Event_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CheckPose_Event_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CheckPose_Event_message_members,
  get_message_typesupport_handle_function,
  &mbf_msgs__srv__CheckPose_Event__get_type_hash,
  &mbf_msgs__srv__CheckPose_Event__get_type_description,
  &mbf_msgs__srv__CheckPose_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace mbf_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::srv::CheckPose_Event>()
{
  return &::mbf_msgs::srv::rosidl_typesupport_introspection_cpp::CheckPose_Event_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, srv, CheckPose_Event)() {
  return &::mbf_msgs::srv::rosidl_typesupport_introspection_cpp::CheckPose_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "mbf_msgs/srv/detail/check_pose__functions.h"
// already included above
// #include "mbf_msgs/srv/detail/check_pose__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace mbf_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers CheckPose_service_members = {
  "mbf_msgs::srv",  // service namespace
  "CheckPose",  // service name
  // the following fields are initialized below on first access
  // see get_service_type_support_handle<mbf_msgs::srv::CheckPose>()
  nullptr,  // request message
  nullptr,  // response message
  nullptr,  // event message
};

static const rosidl_service_type_support_t CheckPose_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CheckPose_service_members,
  get_service_typesupport_handle_function,
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<mbf_msgs::srv::CheckPose_Request>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<mbf_msgs::srv::CheckPose_Response>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<mbf_msgs::srv::CheckPose_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<mbf_msgs::srv::CheckPose>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<mbf_msgs::srv::CheckPose>,
  &mbf_msgs__srv__CheckPose__get_type_hash,
  &mbf_msgs__srv__CheckPose__get_type_description,
  &mbf_msgs__srv__CheckPose__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace mbf_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<mbf_msgs::srv::CheckPose>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::mbf_msgs::srv::rosidl_typesupport_introspection_cpp::CheckPose_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure all of the service_members are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr ||
    service_members->event_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::mbf_msgs::srv::CheckPose_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::mbf_msgs::srv::CheckPose_Response
      >()->data
      );
    // initialize the event_members_ with the static function from the external library
    service_members->event_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::mbf_msgs::srv::CheckPose_Event
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, srv, CheckPose)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<mbf_msgs::srv::CheckPose>();
}

#ifdef __cplusplus
}
#endif
