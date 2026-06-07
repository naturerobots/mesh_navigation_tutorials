// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mbf_msgs:srv/SetTestRobotState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mbf_msgs/srv/detail/set_test_robot_state__rosidl_typesupport_introspection_c.h"
#include "mbf_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mbf_msgs/srv/detail/set_test_robot_state__functions.h"
#include "mbf_msgs/srv/detail/set_test_robot_state__struct.h"


// Include directives for member types
// Member `transform`
#include "geometry_msgs/msg/transform_stamped.h"
// Member `transform`
#include "geometry_msgs/msg/detail/transform_stamped__rosidl_typesupport_introspection_c.h"
// Member `velocity_robot`
#include "geometry_msgs/msg/twist_stamped.h"
// Member `velocity_robot`
#include "geometry_msgs/msg/detail/twist_stamped__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mbf_msgs__srv__SetTestRobotState_Request__rosidl_typesupport_introspection_c__SetTestRobotState_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mbf_msgs__srv__SetTestRobotState_Request__init(message_memory);
}

void mbf_msgs__srv__SetTestRobotState_Request__rosidl_typesupport_introspection_c__SetTestRobotState_Request_fini_function(void * message_memory)
{
  mbf_msgs__srv__SetTestRobotState_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mbf_msgs__srv__SetTestRobotState_Request__rosidl_typesupport_introspection_c__SetTestRobotState_Request_message_member_array[3] = {
  {
    "transform",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__srv__SetTestRobotState_Request, transform),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "set_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__srv__SetTestRobotState_Request, set_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity_robot",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__srv__SetTestRobotState_Request, velocity_robot),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mbf_msgs__srv__SetTestRobotState_Request__rosidl_typesupport_introspection_c__SetTestRobotState_Request_message_members = {
  "mbf_msgs__srv",  // message namespace
  "SetTestRobotState_Request",  // message name
  3,  // number of fields
  sizeof(mbf_msgs__srv__SetTestRobotState_Request),
  false,  // has_any_key_member_
  mbf_msgs__srv__SetTestRobotState_Request__rosidl_typesupport_introspection_c__SetTestRobotState_Request_message_member_array,  // message members
  mbf_msgs__srv__SetTestRobotState_Request__rosidl_typesupport_introspection_c__SetTestRobotState_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  mbf_msgs__srv__SetTestRobotState_Request__rosidl_typesupport_introspection_c__SetTestRobotState_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mbf_msgs__srv__SetTestRobotState_Request__rosidl_typesupport_introspection_c__SetTestRobotState_Request_message_type_support_handle = {
  0,
  &mbf_msgs__srv__SetTestRobotState_Request__rosidl_typesupport_introspection_c__SetTestRobotState_Request_message_members,
  get_message_typesupport_handle_function,
  &mbf_msgs__srv__SetTestRobotState_Request__get_type_hash,
  &mbf_msgs__srv__SetTestRobotState_Request__get_type_description,
  &mbf_msgs__srv__SetTestRobotState_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mbf_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, SetTestRobotState_Request)() {
  mbf_msgs__srv__SetTestRobotState_Request__rosidl_typesupport_introspection_c__SetTestRobotState_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, TransformStamped)();
  mbf_msgs__srv__SetTestRobotState_Request__rosidl_typesupport_introspection_c__SetTestRobotState_Request_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, TwistStamped)();
  if (!mbf_msgs__srv__SetTestRobotState_Request__rosidl_typesupport_introspection_c__SetTestRobotState_Request_message_type_support_handle.typesupport_identifier) {
    mbf_msgs__srv__SetTestRobotState_Request__rosidl_typesupport_introspection_c__SetTestRobotState_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mbf_msgs__srv__SetTestRobotState_Request__rosidl_typesupport_introspection_c__SetTestRobotState_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "mbf_msgs/srv/detail/set_test_robot_state__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mbf_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mbf_msgs/srv/detail/set_test_robot_state__functions.h"
// already included above
// #include "mbf_msgs/srv/detail/set_test_robot_state__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void mbf_msgs__srv__SetTestRobotState_Response__rosidl_typesupport_introspection_c__SetTestRobotState_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mbf_msgs__srv__SetTestRobotState_Response__init(message_memory);
}

void mbf_msgs__srv__SetTestRobotState_Response__rosidl_typesupport_introspection_c__SetTestRobotState_Response_fini_function(void * message_memory)
{
  mbf_msgs__srv__SetTestRobotState_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mbf_msgs__srv__SetTestRobotState_Response__rosidl_typesupport_introspection_c__SetTestRobotState_Response_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__srv__SetTestRobotState_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mbf_msgs__srv__SetTestRobotState_Response__rosidl_typesupport_introspection_c__SetTestRobotState_Response_message_members = {
  "mbf_msgs__srv",  // message namespace
  "SetTestRobotState_Response",  // message name
  1,  // number of fields
  sizeof(mbf_msgs__srv__SetTestRobotState_Response),
  false,  // has_any_key_member_
  mbf_msgs__srv__SetTestRobotState_Response__rosidl_typesupport_introspection_c__SetTestRobotState_Response_message_member_array,  // message members
  mbf_msgs__srv__SetTestRobotState_Response__rosidl_typesupport_introspection_c__SetTestRobotState_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  mbf_msgs__srv__SetTestRobotState_Response__rosidl_typesupport_introspection_c__SetTestRobotState_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mbf_msgs__srv__SetTestRobotState_Response__rosidl_typesupport_introspection_c__SetTestRobotState_Response_message_type_support_handle = {
  0,
  &mbf_msgs__srv__SetTestRobotState_Response__rosidl_typesupport_introspection_c__SetTestRobotState_Response_message_members,
  get_message_typesupport_handle_function,
  &mbf_msgs__srv__SetTestRobotState_Response__get_type_hash,
  &mbf_msgs__srv__SetTestRobotState_Response__get_type_description,
  &mbf_msgs__srv__SetTestRobotState_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mbf_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, SetTestRobotState_Response)() {
  if (!mbf_msgs__srv__SetTestRobotState_Response__rosidl_typesupport_introspection_c__SetTestRobotState_Response_message_type_support_handle.typesupport_identifier) {
    mbf_msgs__srv__SetTestRobotState_Response__rosidl_typesupport_introspection_c__SetTestRobotState_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mbf_msgs__srv__SetTestRobotState_Response__rosidl_typesupport_introspection_c__SetTestRobotState_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "mbf_msgs/srv/detail/set_test_robot_state__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mbf_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mbf_msgs/srv/detail/set_test_robot_state__functions.h"
// already included above
// #include "mbf_msgs/srv/detail/set_test_robot_state__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "mbf_msgs/srv/set_test_robot_state.h"
// Member `request`
// Member `response`
// already included above
// #include "mbf_msgs/srv/detail/set_test_robot_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__SetTestRobotState_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mbf_msgs__srv__SetTestRobotState_Event__init(message_memory);
}

void mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__SetTestRobotState_Event_fini_function(void * message_memory)
{
  mbf_msgs__srv__SetTestRobotState_Event__fini(message_memory);
}

size_t mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__size_function__SetTestRobotState_Event__request(
  const void * untyped_member)
{
  const mbf_msgs__srv__SetTestRobotState_Request__Sequence * member =
    (const mbf_msgs__srv__SetTestRobotState_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__get_const_function__SetTestRobotState_Event__request(
  const void * untyped_member, size_t index)
{
  const mbf_msgs__srv__SetTestRobotState_Request__Sequence * member =
    (const mbf_msgs__srv__SetTestRobotState_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__get_function__SetTestRobotState_Event__request(
  void * untyped_member, size_t index)
{
  mbf_msgs__srv__SetTestRobotState_Request__Sequence * member =
    (mbf_msgs__srv__SetTestRobotState_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__fetch_function__SetTestRobotState_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const mbf_msgs__srv__SetTestRobotState_Request * item =
    ((const mbf_msgs__srv__SetTestRobotState_Request *)
    mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__get_const_function__SetTestRobotState_Event__request(untyped_member, index));
  mbf_msgs__srv__SetTestRobotState_Request * value =
    (mbf_msgs__srv__SetTestRobotState_Request *)(untyped_value);
  *value = *item;
}

void mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__assign_function__SetTestRobotState_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  mbf_msgs__srv__SetTestRobotState_Request * item =
    ((mbf_msgs__srv__SetTestRobotState_Request *)
    mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__get_function__SetTestRobotState_Event__request(untyped_member, index));
  const mbf_msgs__srv__SetTestRobotState_Request * value =
    (const mbf_msgs__srv__SetTestRobotState_Request *)(untyped_value);
  *item = *value;
}

bool mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__resize_function__SetTestRobotState_Event__request(
  void * untyped_member, size_t size)
{
  mbf_msgs__srv__SetTestRobotState_Request__Sequence * member =
    (mbf_msgs__srv__SetTestRobotState_Request__Sequence *)(untyped_member);
  mbf_msgs__srv__SetTestRobotState_Request__Sequence__fini(member);
  return mbf_msgs__srv__SetTestRobotState_Request__Sequence__init(member, size);
}

size_t mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__size_function__SetTestRobotState_Event__response(
  const void * untyped_member)
{
  const mbf_msgs__srv__SetTestRobotState_Response__Sequence * member =
    (const mbf_msgs__srv__SetTestRobotState_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__get_const_function__SetTestRobotState_Event__response(
  const void * untyped_member, size_t index)
{
  const mbf_msgs__srv__SetTestRobotState_Response__Sequence * member =
    (const mbf_msgs__srv__SetTestRobotState_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__get_function__SetTestRobotState_Event__response(
  void * untyped_member, size_t index)
{
  mbf_msgs__srv__SetTestRobotState_Response__Sequence * member =
    (mbf_msgs__srv__SetTestRobotState_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__fetch_function__SetTestRobotState_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const mbf_msgs__srv__SetTestRobotState_Response * item =
    ((const mbf_msgs__srv__SetTestRobotState_Response *)
    mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__get_const_function__SetTestRobotState_Event__response(untyped_member, index));
  mbf_msgs__srv__SetTestRobotState_Response * value =
    (mbf_msgs__srv__SetTestRobotState_Response *)(untyped_value);
  *value = *item;
}

void mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__assign_function__SetTestRobotState_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  mbf_msgs__srv__SetTestRobotState_Response * item =
    ((mbf_msgs__srv__SetTestRobotState_Response *)
    mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__get_function__SetTestRobotState_Event__response(untyped_member, index));
  const mbf_msgs__srv__SetTestRobotState_Response * value =
    (const mbf_msgs__srv__SetTestRobotState_Response *)(untyped_value);
  *item = *value;
}

bool mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__resize_function__SetTestRobotState_Event__response(
  void * untyped_member, size_t size)
{
  mbf_msgs__srv__SetTestRobotState_Response__Sequence * member =
    (mbf_msgs__srv__SetTestRobotState_Response__Sequence *)(untyped_member);
  mbf_msgs__srv__SetTestRobotState_Response__Sequence__fini(member);
  return mbf_msgs__srv__SetTestRobotState_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__SetTestRobotState_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__srv__SetTestRobotState_Event, info),  // bytes offset in struct
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
    offsetof(mbf_msgs__srv__SetTestRobotState_Event, request),  // bytes offset in struct
    NULL,  // default value
    mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__size_function__SetTestRobotState_Event__request,  // size() function pointer
    mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__get_const_function__SetTestRobotState_Event__request,  // get_const(index) function pointer
    mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__get_function__SetTestRobotState_Event__request,  // get(index) function pointer
    mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__fetch_function__SetTestRobotState_Event__request,  // fetch(index, &value) function pointer
    mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__assign_function__SetTestRobotState_Event__request,  // assign(index, value) function pointer
    mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__resize_function__SetTestRobotState_Event__request  // resize(index) function pointer
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
    offsetof(mbf_msgs__srv__SetTestRobotState_Event, response),  // bytes offset in struct
    NULL,  // default value
    mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__size_function__SetTestRobotState_Event__response,  // size() function pointer
    mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__get_const_function__SetTestRobotState_Event__response,  // get_const(index) function pointer
    mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__get_function__SetTestRobotState_Event__response,  // get(index) function pointer
    mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__fetch_function__SetTestRobotState_Event__response,  // fetch(index, &value) function pointer
    mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__assign_function__SetTestRobotState_Event__response,  // assign(index, value) function pointer
    mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__resize_function__SetTestRobotState_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__SetTestRobotState_Event_message_members = {
  "mbf_msgs__srv",  // message namespace
  "SetTestRobotState_Event",  // message name
  3,  // number of fields
  sizeof(mbf_msgs__srv__SetTestRobotState_Event),
  false,  // has_any_key_member_
  mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__SetTestRobotState_Event_message_member_array,  // message members
  mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__SetTestRobotState_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__SetTestRobotState_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__SetTestRobotState_Event_message_type_support_handle = {
  0,
  &mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__SetTestRobotState_Event_message_members,
  get_message_typesupport_handle_function,
  &mbf_msgs__srv__SetTestRobotState_Event__get_type_hash,
  &mbf_msgs__srv__SetTestRobotState_Event__get_type_description,
  &mbf_msgs__srv__SetTestRobotState_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mbf_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, SetTestRobotState_Event)() {
  mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__SetTestRobotState_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__SetTestRobotState_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, SetTestRobotState_Request)();
  mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__SetTestRobotState_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, SetTestRobotState_Response)();
  if (!mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__SetTestRobotState_Event_message_type_support_handle.typesupport_identifier) {
    mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__SetTestRobotState_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__SetTestRobotState_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "mbf_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "mbf_msgs/srv/detail/set_test_robot_state__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers mbf_msgs__srv__detail__set_test_robot_state__rosidl_typesupport_introspection_c__SetTestRobotState_service_members = {
  "mbf_msgs__srv",  // service namespace
  "SetTestRobotState",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // mbf_msgs__srv__detail__set_test_robot_state__rosidl_typesupport_introspection_c__SetTestRobotState_Request_message_type_support_handle,
  NULL,  // response message
  // mbf_msgs__srv__detail__set_test_robot_state__rosidl_typesupport_introspection_c__SetTestRobotState_Response_message_type_support_handle
  NULL  // event_message
  // mbf_msgs__srv__detail__set_test_robot_state__rosidl_typesupport_introspection_c__SetTestRobotState_Response_message_type_support_handle
};


static rosidl_service_type_support_t mbf_msgs__srv__detail__set_test_robot_state__rosidl_typesupport_introspection_c__SetTestRobotState_service_type_support_handle = {
  0,
  &mbf_msgs__srv__detail__set_test_robot_state__rosidl_typesupport_introspection_c__SetTestRobotState_service_members,
  get_service_typesupport_handle_function,
  &mbf_msgs__srv__SetTestRobotState_Request__rosidl_typesupport_introspection_c__SetTestRobotState_Request_message_type_support_handle,
  &mbf_msgs__srv__SetTestRobotState_Response__rosidl_typesupport_introspection_c__SetTestRobotState_Response_message_type_support_handle,
  &mbf_msgs__srv__SetTestRobotState_Event__rosidl_typesupport_introspection_c__SetTestRobotState_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    mbf_msgs,
    srv,
    SetTestRobotState
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    mbf_msgs,
    srv,
    SetTestRobotState
  ),
  &mbf_msgs__srv__SetTestRobotState__get_type_hash,
  &mbf_msgs__srv__SetTestRobotState__get_type_description,
  &mbf_msgs__srv__SetTestRobotState__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, SetTestRobotState_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, SetTestRobotState_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, SetTestRobotState_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mbf_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, SetTestRobotState)(void) {
  if (!mbf_msgs__srv__detail__set_test_robot_state__rosidl_typesupport_introspection_c__SetTestRobotState_service_type_support_handle.typesupport_identifier) {
    mbf_msgs__srv__detail__set_test_robot_state__rosidl_typesupport_introspection_c__SetTestRobotState_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)mbf_msgs__srv__detail__set_test_robot_state__rosidl_typesupport_introspection_c__SetTestRobotState_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, SetTestRobotState_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, SetTestRobotState_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, SetTestRobotState_Event)()->data;
  }

  return &mbf_msgs__srv__detail__set_test_robot_state__rosidl_typesupport_introspection_c__SetTestRobotState_service_type_support_handle;
}
