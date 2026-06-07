// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mbf_msgs:srv/CheckPath.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mbf_msgs/srv/detail/check_path__rosidl_typesupport_introspection_c.h"
#include "mbf_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mbf_msgs/srv/detail/check_path__functions.h"
#include "mbf_msgs/srv/detail/check_path__struct.h"


// Include directives for member types
// Member `path`
#include "nav_msgs/msg/path.h"
// Member `path`
#include "nav_msgs/msg/detail/path__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mbf_msgs__srv__CheckPath_Request__rosidl_typesupport_introspection_c__CheckPath_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mbf_msgs__srv__CheckPath_Request__init(message_memory);
}

void mbf_msgs__srv__CheckPath_Request__rosidl_typesupport_introspection_c__CheckPath_Request_fini_function(void * message_memory)
{
  mbf_msgs__srv__CheckPath_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mbf_msgs__srv__CheckPath_Request__rosidl_typesupport_introspection_c__CheckPath_Request_message_member_array[10] = {
  {
    "path",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__srv__CheckPath_Request, path),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "safety_dist",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__srv__CheckPath_Request, safety_dist),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lethal_cost_mult",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__srv__CheckPath_Request, lethal_cost_mult),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "inscrib_cost_mult",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__srv__CheckPath_Request, inscrib_cost_mult),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "unknown_cost_mult",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__srv__CheckPath_Request, unknown_cost_mult),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "costmap",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__srv__CheckPath_Request, costmap),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "return_on",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__srv__CheckPath_Request, return_on),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "skip_poses",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__srv__CheckPath_Request, skip_poses),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "use_padded_fp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__srv__CheckPath_Request, use_padded_fp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "path_cells_only",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__srv__CheckPath_Request, path_cells_only),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mbf_msgs__srv__CheckPath_Request__rosidl_typesupport_introspection_c__CheckPath_Request_message_members = {
  "mbf_msgs__srv",  // message namespace
  "CheckPath_Request",  // message name
  10,  // number of fields
  sizeof(mbf_msgs__srv__CheckPath_Request),
  false,  // has_any_key_member_
  mbf_msgs__srv__CheckPath_Request__rosidl_typesupport_introspection_c__CheckPath_Request_message_member_array,  // message members
  mbf_msgs__srv__CheckPath_Request__rosidl_typesupport_introspection_c__CheckPath_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  mbf_msgs__srv__CheckPath_Request__rosidl_typesupport_introspection_c__CheckPath_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mbf_msgs__srv__CheckPath_Request__rosidl_typesupport_introspection_c__CheckPath_Request_message_type_support_handle = {
  0,
  &mbf_msgs__srv__CheckPath_Request__rosidl_typesupport_introspection_c__CheckPath_Request_message_members,
  get_message_typesupport_handle_function,
  &mbf_msgs__srv__CheckPath_Request__get_type_hash,
  &mbf_msgs__srv__CheckPath_Request__get_type_description,
  &mbf_msgs__srv__CheckPath_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mbf_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, CheckPath_Request)() {
  mbf_msgs__srv__CheckPath_Request__rosidl_typesupport_introspection_c__CheckPath_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nav_msgs, msg, Path)();
  if (!mbf_msgs__srv__CheckPath_Request__rosidl_typesupport_introspection_c__CheckPath_Request_message_type_support_handle.typesupport_identifier) {
    mbf_msgs__srv__CheckPath_Request__rosidl_typesupport_introspection_c__CheckPath_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mbf_msgs__srv__CheckPath_Request__rosidl_typesupport_introspection_c__CheckPath_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "mbf_msgs/srv/detail/check_path__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mbf_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mbf_msgs/srv/detail/check_path__functions.h"
// already included above
// #include "mbf_msgs/srv/detail/check_path__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void mbf_msgs__srv__CheckPath_Response__rosidl_typesupport_introspection_c__CheckPath_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mbf_msgs__srv__CheckPath_Response__init(message_memory);
}

void mbf_msgs__srv__CheckPath_Response__rosidl_typesupport_introspection_c__CheckPath_Response_fini_function(void * message_memory)
{
  mbf_msgs__srv__CheckPath_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mbf_msgs__srv__CheckPath_Response__rosidl_typesupport_introspection_c__CheckPath_Response_message_member_array[3] = {
  {
    "last_checked",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__srv__CheckPath_Response, last_checked),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__srv__CheckPath_Response, state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cost",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__srv__CheckPath_Response, cost),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mbf_msgs__srv__CheckPath_Response__rosidl_typesupport_introspection_c__CheckPath_Response_message_members = {
  "mbf_msgs__srv",  // message namespace
  "CheckPath_Response",  // message name
  3,  // number of fields
  sizeof(mbf_msgs__srv__CheckPath_Response),
  false,  // has_any_key_member_
  mbf_msgs__srv__CheckPath_Response__rosidl_typesupport_introspection_c__CheckPath_Response_message_member_array,  // message members
  mbf_msgs__srv__CheckPath_Response__rosidl_typesupport_introspection_c__CheckPath_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  mbf_msgs__srv__CheckPath_Response__rosidl_typesupport_introspection_c__CheckPath_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mbf_msgs__srv__CheckPath_Response__rosidl_typesupport_introspection_c__CheckPath_Response_message_type_support_handle = {
  0,
  &mbf_msgs__srv__CheckPath_Response__rosidl_typesupport_introspection_c__CheckPath_Response_message_members,
  get_message_typesupport_handle_function,
  &mbf_msgs__srv__CheckPath_Response__get_type_hash,
  &mbf_msgs__srv__CheckPath_Response__get_type_description,
  &mbf_msgs__srv__CheckPath_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mbf_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, CheckPath_Response)() {
  if (!mbf_msgs__srv__CheckPath_Response__rosidl_typesupport_introspection_c__CheckPath_Response_message_type_support_handle.typesupport_identifier) {
    mbf_msgs__srv__CheckPath_Response__rosidl_typesupport_introspection_c__CheckPath_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mbf_msgs__srv__CheckPath_Response__rosidl_typesupport_introspection_c__CheckPath_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "mbf_msgs/srv/detail/check_path__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mbf_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mbf_msgs/srv/detail/check_path__functions.h"
// already included above
// #include "mbf_msgs/srv/detail/check_path__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "mbf_msgs/srv/check_path.h"
// Member `request`
// Member `response`
// already included above
// #include "mbf_msgs/srv/detail/check_path__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__CheckPath_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mbf_msgs__srv__CheckPath_Event__init(message_memory);
}

void mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__CheckPath_Event_fini_function(void * message_memory)
{
  mbf_msgs__srv__CheckPath_Event__fini(message_memory);
}

size_t mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__size_function__CheckPath_Event__request(
  const void * untyped_member)
{
  const mbf_msgs__srv__CheckPath_Request__Sequence * member =
    (const mbf_msgs__srv__CheckPath_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__get_const_function__CheckPath_Event__request(
  const void * untyped_member, size_t index)
{
  const mbf_msgs__srv__CheckPath_Request__Sequence * member =
    (const mbf_msgs__srv__CheckPath_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__get_function__CheckPath_Event__request(
  void * untyped_member, size_t index)
{
  mbf_msgs__srv__CheckPath_Request__Sequence * member =
    (mbf_msgs__srv__CheckPath_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__fetch_function__CheckPath_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const mbf_msgs__srv__CheckPath_Request * item =
    ((const mbf_msgs__srv__CheckPath_Request *)
    mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__get_const_function__CheckPath_Event__request(untyped_member, index));
  mbf_msgs__srv__CheckPath_Request * value =
    (mbf_msgs__srv__CheckPath_Request *)(untyped_value);
  *value = *item;
}

void mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__assign_function__CheckPath_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  mbf_msgs__srv__CheckPath_Request * item =
    ((mbf_msgs__srv__CheckPath_Request *)
    mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__get_function__CheckPath_Event__request(untyped_member, index));
  const mbf_msgs__srv__CheckPath_Request * value =
    (const mbf_msgs__srv__CheckPath_Request *)(untyped_value);
  *item = *value;
}

bool mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__resize_function__CheckPath_Event__request(
  void * untyped_member, size_t size)
{
  mbf_msgs__srv__CheckPath_Request__Sequence * member =
    (mbf_msgs__srv__CheckPath_Request__Sequence *)(untyped_member);
  mbf_msgs__srv__CheckPath_Request__Sequence__fini(member);
  return mbf_msgs__srv__CheckPath_Request__Sequence__init(member, size);
}

size_t mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__size_function__CheckPath_Event__response(
  const void * untyped_member)
{
  const mbf_msgs__srv__CheckPath_Response__Sequence * member =
    (const mbf_msgs__srv__CheckPath_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__get_const_function__CheckPath_Event__response(
  const void * untyped_member, size_t index)
{
  const mbf_msgs__srv__CheckPath_Response__Sequence * member =
    (const mbf_msgs__srv__CheckPath_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__get_function__CheckPath_Event__response(
  void * untyped_member, size_t index)
{
  mbf_msgs__srv__CheckPath_Response__Sequence * member =
    (mbf_msgs__srv__CheckPath_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__fetch_function__CheckPath_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const mbf_msgs__srv__CheckPath_Response * item =
    ((const mbf_msgs__srv__CheckPath_Response *)
    mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__get_const_function__CheckPath_Event__response(untyped_member, index));
  mbf_msgs__srv__CheckPath_Response * value =
    (mbf_msgs__srv__CheckPath_Response *)(untyped_value);
  *value = *item;
}

void mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__assign_function__CheckPath_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  mbf_msgs__srv__CheckPath_Response * item =
    ((mbf_msgs__srv__CheckPath_Response *)
    mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__get_function__CheckPath_Event__response(untyped_member, index));
  const mbf_msgs__srv__CheckPath_Response * value =
    (const mbf_msgs__srv__CheckPath_Response *)(untyped_value);
  *item = *value;
}

bool mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__resize_function__CheckPath_Event__response(
  void * untyped_member, size_t size)
{
  mbf_msgs__srv__CheckPath_Response__Sequence * member =
    (mbf_msgs__srv__CheckPath_Response__Sequence *)(untyped_member);
  mbf_msgs__srv__CheckPath_Response__Sequence__fini(member);
  return mbf_msgs__srv__CheckPath_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__CheckPath_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__srv__CheckPath_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(mbf_msgs__srv__CheckPath_Event, request),  // bytes offset in struct
    NULL,  // default value
    mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__size_function__CheckPath_Event__request,  // size() function pointer
    mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__get_const_function__CheckPath_Event__request,  // get_const(index) function pointer
    mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__get_function__CheckPath_Event__request,  // get(index) function pointer
    mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__fetch_function__CheckPath_Event__request,  // fetch(index, &value) function pointer
    mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__assign_function__CheckPath_Event__request,  // assign(index, value) function pointer
    mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__resize_function__CheckPath_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(mbf_msgs__srv__CheckPath_Event, response),  // bytes offset in struct
    NULL,  // default value
    mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__size_function__CheckPath_Event__response,  // size() function pointer
    mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__get_const_function__CheckPath_Event__response,  // get_const(index) function pointer
    mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__get_function__CheckPath_Event__response,  // get(index) function pointer
    mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__fetch_function__CheckPath_Event__response,  // fetch(index, &value) function pointer
    mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__assign_function__CheckPath_Event__response,  // assign(index, value) function pointer
    mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__resize_function__CheckPath_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__CheckPath_Event_message_members = {
  "mbf_msgs__srv",  // message namespace
  "CheckPath_Event",  // message name
  3,  // number of fields
  sizeof(mbf_msgs__srv__CheckPath_Event),
  false,  // has_any_key_member_
  mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__CheckPath_Event_message_member_array,  // message members
  mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__CheckPath_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__CheckPath_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__CheckPath_Event_message_type_support_handle = {
  0,
  &mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__CheckPath_Event_message_members,
  get_message_typesupport_handle_function,
  &mbf_msgs__srv__CheckPath_Event__get_type_hash,
  &mbf_msgs__srv__CheckPath_Event__get_type_description,
  &mbf_msgs__srv__CheckPath_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mbf_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, CheckPath_Event)() {
  mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__CheckPath_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__CheckPath_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, CheckPath_Request)();
  mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__CheckPath_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, CheckPath_Response)();
  if (!mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__CheckPath_Event_message_type_support_handle.typesupport_identifier) {
    mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__CheckPath_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__CheckPath_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "mbf_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "mbf_msgs/srv/detail/check_path__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers mbf_msgs__srv__detail__check_path__rosidl_typesupport_introspection_c__CheckPath_service_members = {
  "mbf_msgs__srv",  // service namespace
  "CheckPath",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // mbf_msgs__srv__detail__check_path__rosidl_typesupport_introspection_c__CheckPath_Request_message_type_support_handle,
  NULL,  // response message
  // mbf_msgs__srv__detail__check_path__rosidl_typesupport_introspection_c__CheckPath_Response_message_type_support_handle
  NULL  // event_message
  // mbf_msgs__srv__detail__check_path__rosidl_typesupport_introspection_c__CheckPath_Response_message_type_support_handle
};


static rosidl_service_type_support_t mbf_msgs__srv__detail__check_path__rosidl_typesupport_introspection_c__CheckPath_service_type_support_handle = {
  0,
  &mbf_msgs__srv__detail__check_path__rosidl_typesupport_introspection_c__CheckPath_service_members,
  get_service_typesupport_handle_function,
  &mbf_msgs__srv__CheckPath_Request__rosidl_typesupport_introspection_c__CheckPath_Request_message_type_support_handle,
  &mbf_msgs__srv__CheckPath_Response__rosidl_typesupport_introspection_c__CheckPath_Response_message_type_support_handle,
  &mbf_msgs__srv__CheckPath_Event__rosidl_typesupport_introspection_c__CheckPath_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    mbf_msgs,
    srv,
    CheckPath
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    mbf_msgs,
    srv,
    CheckPath
  ),
  &mbf_msgs__srv__CheckPath__get_type_hash,
  &mbf_msgs__srv__CheckPath__get_type_description,
  &mbf_msgs__srv__CheckPath__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, CheckPath_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, CheckPath_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, CheckPath_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mbf_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, CheckPath)(void) {
  if (!mbf_msgs__srv__detail__check_path__rosidl_typesupport_introspection_c__CheckPath_service_type_support_handle.typesupport_identifier) {
    mbf_msgs__srv__detail__check_path__rosidl_typesupport_introspection_c__CheckPath_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)mbf_msgs__srv__detail__check_path__rosidl_typesupport_introspection_c__CheckPath_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, CheckPath_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, CheckPath_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, CheckPath_Event)()->data;
  }

  return &mbf_msgs__srv__detail__check_path__rosidl_typesupport_introspection_c__CheckPath_service_type_support_handle;
}
