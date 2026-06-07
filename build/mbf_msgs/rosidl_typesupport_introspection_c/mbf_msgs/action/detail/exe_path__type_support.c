// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mbf_msgs:action/ExePath.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mbf_msgs/action/detail/exe_path__rosidl_typesupport_introspection_c.h"
#include "mbf_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mbf_msgs/action/detail/exe_path__functions.h"
#include "mbf_msgs/action/detail/exe_path__struct.h"


// Include directives for member types
// Member `path`
#include "nav_msgs/msg/path.h"
// Member `path`
#include "nav_msgs/msg/detail/path__rosidl_typesupport_introspection_c.h"
// Member `controller`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mbf_msgs__action__ExePath_Goal__rosidl_typesupport_introspection_c__ExePath_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mbf_msgs__action__ExePath_Goal__init(message_memory);
}

void mbf_msgs__action__ExePath_Goal__rosidl_typesupport_introspection_c__ExePath_Goal_fini_function(void * message_memory)
{
  mbf_msgs__action__ExePath_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mbf_msgs__action__ExePath_Goal__rosidl_typesupport_introspection_c__ExePath_Goal_message_member_array[6] = {
  {
    "path",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_Goal, path),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "controller",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_Goal, controller),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "concurrency_slot",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_Goal, concurrency_slot),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tolerance_from_action",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_Goal, tolerance_from_action),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dist_tolerance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_Goal, dist_tolerance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angle_tolerance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_Goal, angle_tolerance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mbf_msgs__action__ExePath_Goal__rosidl_typesupport_introspection_c__ExePath_Goal_message_members = {
  "mbf_msgs__action",  // message namespace
  "ExePath_Goal",  // message name
  6,  // number of fields
  sizeof(mbf_msgs__action__ExePath_Goal),
  false,  // has_any_key_member_
  mbf_msgs__action__ExePath_Goal__rosidl_typesupport_introspection_c__ExePath_Goal_message_member_array,  // message members
  mbf_msgs__action__ExePath_Goal__rosidl_typesupport_introspection_c__ExePath_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  mbf_msgs__action__ExePath_Goal__rosidl_typesupport_introspection_c__ExePath_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mbf_msgs__action__ExePath_Goal__rosidl_typesupport_introspection_c__ExePath_Goal_message_type_support_handle = {
  0,
  &mbf_msgs__action__ExePath_Goal__rosidl_typesupport_introspection_c__ExePath_Goal_message_members,
  get_message_typesupport_handle_function,
  &mbf_msgs__action__ExePath_Goal__get_type_hash,
  &mbf_msgs__action__ExePath_Goal__get_type_description,
  &mbf_msgs__action__ExePath_Goal__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mbf_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_Goal)() {
  mbf_msgs__action__ExePath_Goal__rosidl_typesupport_introspection_c__ExePath_Goal_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nav_msgs, msg, Path)();
  if (!mbf_msgs__action__ExePath_Goal__rosidl_typesupport_introspection_c__ExePath_Goal_message_type_support_handle.typesupport_identifier) {
    mbf_msgs__action__ExePath_Goal__rosidl_typesupport_introspection_c__ExePath_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mbf_msgs__action__ExePath_Goal__rosidl_typesupport_introspection_c__ExePath_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "mbf_msgs/action/detail/exe_path__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mbf_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__functions.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.h"


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `final_pose`
#include "geometry_msgs/msg/pose_stamped.h"
// Member `final_pose`
#include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mbf_msgs__action__ExePath_Result__rosidl_typesupport_introspection_c__ExePath_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mbf_msgs__action__ExePath_Result__init(message_memory);
}

void mbf_msgs__action__ExePath_Result__rosidl_typesupport_introspection_c__ExePath_Result_fini_function(void * message_memory)
{
  mbf_msgs__action__ExePath_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mbf_msgs__action__ExePath_Result__rosidl_typesupport_introspection_c__ExePath_Result_message_member_array[5] = {
  {
    "outcome",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_Result, outcome),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_Result, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "final_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_Result, final_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dist_to_goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_Result, dist_to_goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angle_to_goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_Result, angle_to_goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mbf_msgs__action__ExePath_Result__rosidl_typesupport_introspection_c__ExePath_Result_message_members = {
  "mbf_msgs__action",  // message namespace
  "ExePath_Result",  // message name
  5,  // number of fields
  sizeof(mbf_msgs__action__ExePath_Result),
  false,  // has_any_key_member_
  mbf_msgs__action__ExePath_Result__rosidl_typesupport_introspection_c__ExePath_Result_message_member_array,  // message members
  mbf_msgs__action__ExePath_Result__rosidl_typesupport_introspection_c__ExePath_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  mbf_msgs__action__ExePath_Result__rosidl_typesupport_introspection_c__ExePath_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mbf_msgs__action__ExePath_Result__rosidl_typesupport_introspection_c__ExePath_Result_message_type_support_handle = {
  0,
  &mbf_msgs__action__ExePath_Result__rosidl_typesupport_introspection_c__ExePath_Result_message_members,
  get_message_typesupport_handle_function,
  &mbf_msgs__action__ExePath_Result__get_type_hash,
  &mbf_msgs__action__ExePath_Result__get_type_description,
  &mbf_msgs__action__ExePath_Result__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mbf_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_Result)() {
  mbf_msgs__action__ExePath_Result__rosidl_typesupport_introspection_c__ExePath_Result_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  if (!mbf_msgs__action__ExePath_Result__rosidl_typesupport_introspection_c__ExePath_Result_message_type_support_handle.typesupport_identifier) {
    mbf_msgs__action__ExePath_Result__rosidl_typesupport_introspection_c__ExePath_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mbf_msgs__action__ExePath_Result__rosidl_typesupport_introspection_c__ExePath_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "mbf_msgs/action/detail/exe_path__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mbf_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__functions.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.h"


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `current_pose`
// already included above
// #include "geometry_msgs/msg/pose_stamped.h"
// Member `current_pose`
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"
// Member `last_cmd_vel`
#include "geometry_msgs/msg/twist_stamped.h"
// Member `last_cmd_vel`
#include "geometry_msgs/msg/detail/twist_stamped__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mbf_msgs__action__ExePath_Feedback__rosidl_typesupport_introspection_c__ExePath_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mbf_msgs__action__ExePath_Feedback__init(message_memory);
}

void mbf_msgs__action__ExePath_Feedback__rosidl_typesupport_introspection_c__ExePath_Feedback_fini_function(void * message_memory)
{
  mbf_msgs__action__ExePath_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mbf_msgs__action__ExePath_Feedback__rosidl_typesupport_introspection_c__ExePath_Feedback_message_member_array[6] = {
  {
    "outcome",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_Feedback, outcome),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_Feedback, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dist_to_goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_Feedback, dist_to_goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angle_to_goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_Feedback, angle_to_goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_Feedback, current_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "last_cmd_vel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_Feedback, last_cmd_vel),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mbf_msgs__action__ExePath_Feedback__rosidl_typesupport_introspection_c__ExePath_Feedback_message_members = {
  "mbf_msgs__action",  // message namespace
  "ExePath_Feedback",  // message name
  6,  // number of fields
  sizeof(mbf_msgs__action__ExePath_Feedback),
  false,  // has_any_key_member_
  mbf_msgs__action__ExePath_Feedback__rosidl_typesupport_introspection_c__ExePath_Feedback_message_member_array,  // message members
  mbf_msgs__action__ExePath_Feedback__rosidl_typesupport_introspection_c__ExePath_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  mbf_msgs__action__ExePath_Feedback__rosidl_typesupport_introspection_c__ExePath_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mbf_msgs__action__ExePath_Feedback__rosidl_typesupport_introspection_c__ExePath_Feedback_message_type_support_handle = {
  0,
  &mbf_msgs__action__ExePath_Feedback__rosidl_typesupport_introspection_c__ExePath_Feedback_message_members,
  get_message_typesupport_handle_function,
  &mbf_msgs__action__ExePath_Feedback__get_type_hash,
  &mbf_msgs__action__ExePath_Feedback__get_type_description,
  &mbf_msgs__action__ExePath_Feedback__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mbf_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_Feedback)() {
  mbf_msgs__action__ExePath_Feedback__rosidl_typesupport_introspection_c__ExePath_Feedback_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  mbf_msgs__action__ExePath_Feedback__rosidl_typesupport_introspection_c__ExePath_Feedback_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, TwistStamped)();
  if (!mbf_msgs__action__ExePath_Feedback__rosidl_typesupport_introspection_c__ExePath_Feedback_message_type_support_handle.typesupport_identifier) {
    mbf_msgs__action__ExePath_Feedback__rosidl_typesupport_introspection_c__ExePath_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mbf_msgs__action__ExePath_Feedback__rosidl_typesupport_introspection_c__ExePath_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "mbf_msgs/action/detail/exe_path__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mbf_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__functions.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "mbf_msgs/action/exe_path.h"
// Member `goal`
// already included above
// #include "mbf_msgs/action/detail/exe_path__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mbf_msgs__action__ExePath_SendGoal_Request__rosidl_typesupport_introspection_c__ExePath_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mbf_msgs__action__ExePath_SendGoal_Request__init(message_memory);
}

void mbf_msgs__action__ExePath_SendGoal_Request__rosidl_typesupport_introspection_c__ExePath_SendGoal_Request_fini_function(void * message_memory)
{
  mbf_msgs__action__ExePath_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mbf_msgs__action__ExePath_SendGoal_Request__rosidl_typesupport_introspection_c__ExePath_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mbf_msgs__action__ExePath_SendGoal_Request__rosidl_typesupport_introspection_c__ExePath_SendGoal_Request_message_members = {
  "mbf_msgs__action",  // message namespace
  "ExePath_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(mbf_msgs__action__ExePath_SendGoal_Request),
  false,  // has_any_key_member_
  mbf_msgs__action__ExePath_SendGoal_Request__rosidl_typesupport_introspection_c__ExePath_SendGoal_Request_message_member_array,  // message members
  mbf_msgs__action__ExePath_SendGoal_Request__rosidl_typesupport_introspection_c__ExePath_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  mbf_msgs__action__ExePath_SendGoal_Request__rosidl_typesupport_introspection_c__ExePath_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mbf_msgs__action__ExePath_SendGoal_Request__rosidl_typesupport_introspection_c__ExePath_SendGoal_Request_message_type_support_handle = {
  0,
  &mbf_msgs__action__ExePath_SendGoal_Request__rosidl_typesupport_introspection_c__ExePath_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
  &mbf_msgs__action__ExePath_SendGoal_Request__get_type_hash,
  &mbf_msgs__action__ExePath_SendGoal_Request__get_type_description,
  &mbf_msgs__action__ExePath_SendGoal_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mbf_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_SendGoal_Request)() {
  mbf_msgs__action__ExePath_SendGoal_Request__rosidl_typesupport_introspection_c__ExePath_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  mbf_msgs__action__ExePath_SendGoal_Request__rosidl_typesupport_introspection_c__ExePath_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_Goal)();
  if (!mbf_msgs__action__ExePath_SendGoal_Request__rosidl_typesupport_introspection_c__ExePath_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    mbf_msgs__action__ExePath_SendGoal_Request__rosidl_typesupport_introspection_c__ExePath_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mbf_msgs__action__ExePath_SendGoal_Request__rosidl_typesupport_introspection_c__ExePath_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "mbf_msgs/action/detail/exe_path__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mbf_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__functions.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mbf_msgs__action__ExePath_SendGoal_Response__rosidl_typesupport_introspection_c__ExePath_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mbf_msgs__action__ExePath_SendGoal_Response__init(message_memory);
}

void mbf_msgs__action__ExePath_SendGoal_Response__rosidl_typesupport_introspection_c__ExePath_SendGoal_Response_fini_function(void * message_memory)
{
  mbf_msgs__action__ExePath_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mbf_msgs__action__ExePath_SendGoal_Response__rosidl_typesupport_introspection_c__ExePath_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mbf_msgs__action__ExePath_SendGoal_Response__rosidl_typesupport_introspection_c__ExePath_SendGoal_Response_message_members = {
  "mbf_msgs__action",  // message namespace
  "ExePath_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(mbf_msgs__action__ExePath_SendGoal_Response),
  false,  // has_any_key_member_
  mbf_msgs__action__ExePath_SendGoal_Response__rosidl_typesupport_introspection_c__ExePath_SendGoal_Response_message_member_array,  // message members
  mbf_msgs__action__ExePath_SendGoal_Response__rosidl_typesupport_introspection_c__ExePath_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  mbf_msgs__action__ExePath_SendGoal_Response__rosidl_typesupport_introspection_c__ExePath_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mbf_msgs__action__ExePath_SendGoal_Response__rosidl_typesupport_introspection_c__ExePath_SendGoal_Response_message_type_support_handle = {
  0,
  &mbf_msgs__action__ExePath_SendGoal_Response__rosidl_typesupport_introspection_c__ExePath_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
  &mbf_msgs__action__ExePath_SendGoal_Response__get_type_hash,
  &mbf_msgs__action__ExePath_SendGoal_Response__get_type_description,
  &mbf_msgs__action__ExePath_SendGoal_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mbf_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_SendGoal_Response)() {
  mbf_msgs__action__ExePath_SendGoal_Response__rosidl_typesupport_introspection_c__ExePath_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!mbf_msgs__action__ExePath_SendGoal_Response__rosidl_typesupport_introspection_c__ExePath_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    mbf_msgs__action__ExePath_SendGoal_Response__rosidl_typesupport_introspection_c__ExePath_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mbf_msgs__action__ExePath_SendGoal_Response__rosidl_typesupport_introspection_c__ExePath_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "mbf_msgs/action/detail/exe_path__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mbf_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__functions.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
// already included above
// #include "mbf_msgs/action/exe_path.h"
// Member `request`
// Member `response`
// already included above
// #include "mbf_msgs/action/detail/exe_path__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__ExePath_SendGoal_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mbf_msgs__action__ExePath_SendGoal_Event__init(message_memory);
}

void mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__ExePath_SendGoal_Event_fini_function(void * message_memory)
{
  mbf_msgs__action__ExePath_SendGoal_Event__fini(message_memory);
}

size_t mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__size_function__ExePath_SendGoal_Event__request(
  const void * untyped_member)
{
  const mbf_msgs__action__ExePath_SendGoal_Request__Sequence * member =
    (const mbf_msgs__action__ExePath_SendGoal_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__ExePath_SendGoal_Event__request(
  const void * untyped_member, size_t index)
{
  const mbf_msgs__action__ExePath_SendGoal_Request__Sequence * member =
    (const mbf_msgs__action__ExePath_SendGoal_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__ExePath_SendGoal_Event__request(
  void * untyped_member, size_t index)
{
  mbf_msgs__action__ExePath_SendGoal_Request__Sequence * member =
    (mbf_msgs__action__ExePath_SendGoal_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__fetch_function__ExePath_SendGoal_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const mbf_msgs__action__ExePath_SendGoal_Request * item =
    ((const mbf_msgs__action__ExePath_SendGoal_Request *)
    mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__ExePath_SendGoal_Event__request(untyped_member, index));
  mbf_msgs__action__ExePath_SendGoal_Request * value =
    (mbf_msgs__action__ExePath_SendGoal_Request *)(untyped_value);
  *value = *item;
}

void mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__assign_function__ExePath_SendGoal_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  mbf_msgs__action__ExePath_SendGoal_Request * item =
    ((mbf_msgs__action__ExePath_SendGoal_Request *)
    mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__ExePath_SendGoal_Event__request(untyped_member, index));
  const mbf_msgs__action__ExePath_SendGoal_Request * value =
    (const mbf_msgs__action__ExePath_SendGoal_Request *)(untyped_value);
  *item = *value;
}

bool mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__resize_function__ExePath_SendGoal_Event__request(
  void * untyped_member, size_t size)
{
  mbf_msgs__action__ExePath_SendGoal_Request__Sequence * member =
    (mbf_msgs__action__ExePath_SendGoal_Request__Sequence *)(untyped_member);
  mbf_msgs__action__ExePath_SendGoal_Request__Sequence__fini(member);
  return mbf_msgs__action__ExePath_SendGoal_Request__Sequence__init(member, size);
}

size_t mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__size_function__ExePath_SendGoal_Event__response(
  const void * untyped_member)
{
  const mbf_msgs__action__ExePath_SendGoal_Response__Sequence * member =
    (const mbf_msgs__action__ExePath_SendGoal_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__ExePath_SendGoal_Event__response(
  const void * untyped_member, size_t index)
{
  const mbf_msgs__action__ExePath_SendGoal_Response__Sequence * member =
    (const mbf_msgs__action__ExePath_SendGoal_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__ExePath_SendGoal_Event__response(
  void * untyped_member, size_t index)
{
  mbf_msgs__action__ExePath_SendGoal_Response__Sequence * member =
    (mbf_msgs__action__ExePath_SendGoal_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__fetch_function__ExePath_SendGoal_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const mbf_msgs__action__ExePath_SendGoal_Response * item =
    ((const mbf_msgs__action__ExePath_SendGoal_Response *)
    mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__ExePath_SendGoal_Event__response(untyped_member, index));
  mbf_msgs__action__ExePath_SendGoal_Response * value =
    (mbf_msgs__action__ExePath_SendGoal_Response *)(untyped_value);
  *value = *item;
}

void mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__assign_function__ExePath_SendGoal_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  mbf_msgs__action__ExePath_SendGoal_Response * item =
    ((mbf_msgs__action__ExePath_SendGoal_Response *)
    mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__ExePath_SendGoal_Event__response(untyped_member, index));
  const mbf_msgs__action__ExePath_SendGoal_Response * value =
    (const mbf_msgs__action__ExePath_SendGoal_Response *)(untyped_value);
  *item = *value;
}

bool mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__resize_function__ExePath_SendGoal_Event__response(
  void * untyped_member, size_t size)
{
  mbf_msgs__action__ExePath_SendGoal_Response__Sequence * member =
    (mbf_msgs__action__ExePath_SendGoal_Response__Sequence *)(untyped_member);
  mbf_msgs__action__ExePath_SendGoal_Response__Sequence__fini(member);
  return mbf_msgs__action__ExePath_SendGoal_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__ExePath_SendGoal_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_SendGoal_Event, info),  // bytes offset in struct
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
    offsetof(mbf_msgs__action__ExePath_SendGoal_Event, request),  // bytes offset in struct
    NULL,  // default value
    mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__size_function__ExePath_SendGoal_Event__request,  // size() function pointer
    mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__ExePath_SendGoal_Event__request,  // get_const(index) function pointer
    mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__ExePath_SendGoal_Event__request,  // get(index) function pointer
    mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__fetch_function__ExePath_SendGoal_Event__request,  // fetch(index, &value) function pointer
    mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__assign_function__ExePath_SendGoal_Event__request,  // assign(index, value) function pointer
    mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__resize_function__ExePath_SendGoal_Event__request  // resize(index) function pointer
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
    offsetof(mbf_msgs__action__ExePath_SendGoal_Event, response),  // bytes offset in struct
    NULL,  // default value
    mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__size_function__ExePath_SendGoal_Event__response,  // size() function pointer
    mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__get_const_function__ExePath_SendGoal_Event__response,  // get_const(index) function pointer
    mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__get_function__ExePath_SendGoal_Event__response,  // get(index) function pointer
    mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__fetch_function__ExePath_SendGoal_Event__response,  // fetch(index, &value) function pointer
    mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__assign_function__ExePath_SendGoal_Event__response,  // assign(index, value) function pointer
    mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__resize_function__ExePath_SendGoal_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__ExePath_SendGoal_Event_message_members = {
  "mbf_msgs__action",  // message namespace
  "ExePath_SendGoal_Event",  // message name
  3,  // number of fields
  sizeof(mbf_msgs__action__ExePath_SendGoal_Event),
  false,  // has_any_key_member_
  mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__ExePath_SendGoal_Event_message_member_array,  // message members
  mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__ExePath_SendGoal_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__ExePath_SendGoal_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__ExePath_SendGoal_Event_message_type_support_handle = {
  0,
  &mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__ExePath_SendGoal_Event_message_members,
  get_message_typesupport_handle_function,
  &mbf_msgs__action__ExePath_SendGoal_Event__get_type_hash,
  &mbf_msgs__action__ExePath_SendGoal_Event__get_type_description,
  &mbf_msgs__action__ExePath_SendGoal_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mbf_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_SendGoal_Event)() {
  mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__ExePath_SendGoal_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__ExePath_SendGoal_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_SendGoal_Request)();
  mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__ExePath_SendGoal_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_SendGoal_Response)();
  if (!mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__ExePath_SendGoal_Event_message_type_support_handle.typesupport_identifier) {
    mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__ExePath_SendGoal_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__ExePath_SendGoal_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "mbf_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers mbf_msgs__action__detail__exe_path__rosidl_typesupport_introspection_c__ExePath_SendGoal_service_members = {
  "mbf_msgs__action",  // service namespace
  "ExePath_SendGoal",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // mbf_msgs__action__detail__exe_path__rosidl_typesupport_introspection_c__ExePath_SendGoal_Request_message_type_support_handle,
  NULL,  // response message
  // mbf_msgs__action__detail__exe_path__rosidl_typesupport_introspection_c__ExePath_SendGoal_Response_message_type_support_handle
  NULL  // event_message
  // mbf_msgs__action__detail__exe_path__rosidl_typesupport_introspection_c__ExePath_SendGoal_Response_message_type_support_handle
};


static rosidl_service_type_support_t mbf_msgs__action__detail__exe_path__rosidl_typesupport_introspection_c__ExePath_SendGoal_service_type_support_handle = {
  0,
  &mbf_msgs__action__detail__exe_path__rosidl_typesupport_introspection_c__ExePath_SendGoal_service_members,
  get_service_typesupport_handle_function,
  &mbf_msgs__action__ExePath_SendGoal_Request__rosidl_typesupport_introspection_c__ExePath_SendGoal_Request_message_type_support_handle,
  &mbf_msgs__action__ExePath_SendGoal_Response__rosidl_typesupport_introspection_c__ExePath_SendGoal_Response_message_type_support_handle,
  &mbf_msgs__action__ExePath_SendGoal_Event__rosidl_typesupport_introspection_c__ExePath_SendGoal_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    mbf_msgs,
    action,
    ExePath_SendGoal
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    mbf_msgs,
    action,
    ExePath_SendGoal
  ),
  &mbf_msgs__action__ExePath_SendGoal__get_type_hash,
  &mbf_msgs__action__ExePath_SendGoal__get_type_description,
  &mbf_msgs__action__ExePath_SendGoal__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_SendGoal_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_SendGoal_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_SendGoal_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mbf_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_SendGoal)(void) {
  if (!mbf_msgs__action__detail__exe_path__rosidl_typesupport_introspection_c__ExePath_SendGoal_service_type_support_handle.typesupport_identifier) {
    mbf_msgs__action__detail__exe_path__rosidl_typesupport_introspection_c__ExePath_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)mbf_msgs__action__detail__exe_path__rosidl_typesupport_introspection_c__ExePath_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_SendGoal_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_SendGoal_Event)()->data;
  }

  return &mbf_msgs__action__detail__exe_path__rosidl_typesupport_introspection_c__ExePath_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "mbf_msgs/action/detail/exe_path__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mbf_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__functions.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mbf_msgs__action__ExePath_GetResult_Request__rosidl_typesupport_introspection_c__ExePath_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mbf_msgs__action__ExePath_GetResult_Request__init(message_memory);
}

void mbf_msgs__action__ExePath_GetResult_Request__rosidl_typesupport_introspection_c__ExePath_GetResult_Request_fini_function(void * message_memory)
{
  mbf_msgs__action__ExePath_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mbf_msgs__action__ExePath_GetResult_Request__rosidl_typesupport_introspection_c__ExePath_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mbf_msgs__action__ExePath_GetResult_Request__rosidl_typesupport_introspection_c__ExePath_GetResult_Request_message_members = {
  "mbf_msgs__action",  // message namespace
  "ExePath_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(mbf_msgs__action__ExePath_GetResult_Request),
  false,  // has_any_key_member_
  mbf_msgs__action__ExePath_GetResult_Request__rosidl_typesupport_introspection_c__ExePath_GetResult_Request_message_member_array,  // message members
  mbf_msgs__action__ExePath_GetResult_Request__rosidl_typesupport_introspection_c__ExePath_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  mbf_msgs__action__ExePath_GetResult_Request__rosidl_typesupport_introspection_c__ExePath_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mbf_msgs__action__ExePath_GetResult_Request__rosidl_typesupport_introspection_c__ExePath_GetResult_Request_message_type_support_handle = {
  0,
  &mbf_msgs__action__ExePath_GetResult_Request__rosidl_typesupport_introspection_c__ExePath_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
  &mbf_msgs__action__ExePath_GetResult_Request__get_type_hash,
  &mbf_msgs__action__ExePath_GetResult_Request__get_type_description,
  &mbf_msgs__action__ExePath_GetResult_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mbf_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_GetResult_Request)() {
  mbf_msgs__action__ExePath_GetResult_Request__rosidl_typesupport_introspection_c__ExePath_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!mbf_msgs__action__ExePath_GetResult_Request__rosidl_typesupport_introspection_c__ExePath_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    mbf_msgs__action__ExePath_GetResult_Request__rosidl_typesupport_introspection_c__ExePath_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mbf_msgs__action__ExePath_GetResult_Request__rosidl_typesupport_introspection_c__ExePath_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "mbf_msgs/action/detail/exe_path__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mbf_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__functions.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "mbf_msgs/action/exe_path.h"
// Member `result`
// already included above
// #include "mbf_msgs/action/detail/exe_path__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mbf_msgs__action__ExePath_GetResult_Response__rosidl_typesupport_introspection_c__ExePath_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mbf_msgs__action__ExePath_GetResult_Response__init(message_memory);
}

void mbf_msgs__action__ExePath_GetResult_Response__rosidl_typesupport_introspection_c__ExePath_GetResult_Response_fini_function(void * message_memory)
{
  mbf_msgs__action__ExePath_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mbf_msgs__action__ExePath_GetResult_Response__rosidl_typesupport_introspection_c__ExePath_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mbf_msgs__action__ExePath_GetResult_Response__rosidl_typesupport_introspection_c__ExePath_GetResult_Response_message_members = {
  "mbf_msgs__action",  // message namespace
  "ExePath_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(mbf_msgs__action__ExePath_GetResult_Response),
  false,  // has_any_key_member_
  mbf_msgs__action__ExePath_GetResult_Response__rosidl_typesupport_introspection_c__ExePath_GetResult_Response_message_member_array,  // message members
  mbf_msgs__action__ExePath_GetResult_Response__rosidl_typesupport_introspection_c__ExePath_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  mbf_msgs__action__ExePath_GetResult_Response__rosidl_typesupport_introspection_c__ExePath_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mbf_msgs__action__ExePath_GetResult_Response__rosidl_typesupport_introspection_c__ExePath_GetResult_Response_message_type_support_handle = {
  0,
  &mbf_msgs__action__ExePath_GetResult_Response__rosidl_typesupport_introspection_c__ExePath_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
  &mbf_msgs__action__ExePath_GetResult_Response__get_type_hash,
  &mbf_msgs__action__ExePath_GetResult_Response__get_type_description,
  &mbf_msgs__action__ExePath_GetResult_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mbf_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_GetResult_Response)() {
  mbf_msgs__action__ExePath_GetResult_Response__rosidl_typesupport_introspection_c__ExePath_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_Result)();
  if (!mbf_msgs__action__ExePath_GetResult_Response__rosidl_typesupport_introspection_c__ExePath_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    mbf_msgs__action__ExePath_GetResult_Response__rosidl_typesupport_introspection_c__ExePath_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mbf_msgs__action__ExePath_GetResult_Response__rosidl_typesupport_introspection_c__ExePath_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "mbf_msgs/action/detail/exe_path__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mbf_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__functions.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.h"


// Include directives for member types
// Member `info`
// already included above
// #include "service_msgs/msg/service_event_info.h"
// Member `info`
// already included above
// #include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
// already included above
// #include "mbf_msgs/action/exe_path.h"
// Member `request`
// Member `response`
// already included above
// #include "mbf_msgs/action/detail/exe_path__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__ExePath_GetResult_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mbf_msgs__action__ExePath_GetResult_Event__init(message_memory);
}

void mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__ExePath_GetResult_Event_fini_function(void * message_memory)
{
  mbf_msgs__action__ExePath_GetResult_Event__fini(message_memory);
}

size_t mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__size_function__ExePath_GetResult_Event__request(
  const void * untyped_member)
{
  const mbf_msgs__action__ExePath_GetResult_Request__Sequence * member =
    (const mbf_msgs__action__ExePath_GetResult_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__ExePath_GetResult_Event__request(
  const void * untyped_member, size_t index)
{
  const mbf_msgs__action__ExePath_GetResult_Request__Sequence * member =
    (const mbf_msgs__action__ExePath_GetResult_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__get_function__ExePath_GetResult_Event__request(
  void * untyped_member, size_t index)
{
  mbf_msgs__action__ExePath_GetResult_Request__Sequence * member =
    (mbf_msgs__action__ExePath_GetResult_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__fetch_function__ExePath_GetResult_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const mbf_msgs__action__ExePath_GetResult_Request * item =
    ((const mbf_msgs__action__ExePath_GetResult_Request *)
    mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__ExePath_GetResult_Event__request(untyped_member, index));
  mbf_msgs__action__ExePath_GetResult_Request * value =
    (mbf_msgs__action__ExePath_GetResult_Request *)(untyped_value);
  *value = *item;
}

void mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__assign_function__ExePath_GetResult_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  mbf_msgs__action__ExePath_GetResult_Request * item =
    ((mbf_msgs__action__ExePath_GetResult_Request *)
    mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__get_function__ExePath_GetResult_Event__request(untyped_member, index));
  const mbf_msgs__action__ExePath_GetResult_Request * value =
    (const mbf_msgs__action__ExePath_GetResult_Request *)(untyped_value);
  *item = *value;
}

bool mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__resize_function__ExePath_GetResult_Event__request(
  void * untyped_member, size_t size)
{
  mbf_msgs__action__ExePath_GetResult_Request__Sequence * member =
    (mbf_msgs__action__ExePath_GetResult_Request__Sequence *)(untyped_member);
  mbf_msgs__action__ExePath_GetResult_Request__Sequence__fini(member);
  return mbf_msgs__action__ExePath_GetResult_Request__Sequence__init(member, size);
}

size_t mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__size_function__ExePath_GetResult_Event__response(
  const void * untyped_member)
{
  const mbf_msgs__action__ExePath_GetResult_Response__Sequence * member =
    (const mbf_msgs__action__ExePath_GetResult_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__ExePath_GetResult_Event__response(
  const void * untyped_member, size_t index)
{
  const mbf_msgs__action__ExePath_GetResult_Response__Sequence * member =
    (const mbf_msgs__action__ExePath_GetResult_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__get_function__ExePath_GetResult_Event__response(
  void * untyped_member, size_t index)
{
  mbf_msgs__action__ExePath_GetResult_Response__Sequence * member =
    (mbf_msgs__action__ExePath_GetResult_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__fetch_function__ExePath_GetResult_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const mbf_msgs__action__ExePath_GetResult_Response * item =
    ((const mbf_msgs__action__ExePath_GetResult_Response *)
    mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__ExePath_GetResult_Event__response(untyped_member, index));
  mbf_msgs__action__ExePath_GetResult_Response * value =
    (mbf_msgs__action__ExePath_GetResult_Response *)(untyped_value);
  *value = *item;
}

void mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__assign_function__ExePath_GetResult_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  mbf_msgs__action__ExePath_GetResult_Response * item =
    ((mbf_msgs__action__ExePath_GetResult_Response *)
    mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__get_function__ExePath_GetResult_Event__response(untyped_member, index));
  const mbf_msgs__action__ExePath_GetResult_Response * value =
    (const mbf_msgs__action__ExePath_GetResult_Response *)(untyped_value);
  *item = *value;
}

bool mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__resize_function__ExePath_GetResult_Event__response(
  void * untyped_member, size_t size)
{
  mbf_msgs__action__ExePath_GetResult_Response__Sequence * member =
    (mbf_msgs__action__ExePath_GetResult_Response__Sequence *)(untyped_member);
  mbf_msgs__action__ExePath_GetResult_Response__Sequence__fini(member);
  return mbf_msgs__action__ExePath_GetResult_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__ExePath_GetResult_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_GetResult_Event, info),  // bytes offset in struct
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
    offsetof(mbf_msgs__action__ExePath_GetResult_Event, request),  // bytes offset in struct
    NULL,  // default value
    mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__size_function__ExePath_GetResult_Event__request,  // size() function pointer
    mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__ExePath_GetResult_Event__request,  // get_const(index) function pointer
    mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__get_function__ExePath_GetResult_Event__request,  // get(index) function pointer
    mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__fetch_function__ExePath_GetResult_Event__request,  // fetch(index, &value) function pointer
    mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__assign_function__ExePath_GetResult_Event__request,  // assign(index, value) function pointer
    mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__resize_function__ExePath_GetResult_Event__request  // resize(index) function pointer
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
    offsetof(mbf_msgs__action__ExePath_GetResult_Event, response),  // bytes offset in struct
    NULL,  // default value
    mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__size_function__ExePath_GetResult_Event__response,  // size() function pointer
    mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__get_const_function__ExePath_GetResult_Event__response,  // get_const(index) function pointer
    mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__get_function__ExePath_GetResult_Event__response,  // get(index) function pointer
    mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__fetch_function__ExePath_GetResult_Event__response,  // fetch(index, &value) function pointer
    mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__assign_function__ExePath_GetResult_Event__response,  // assign(index, value) function pointer
    mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__resize_function__ExePath_GetResult_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__ExePath_GetResult_Event_message_members = {
  "mbf_msgs__action",  // message namespace
  "ExePath_GetResult_Event",  // message name
  3,  // number of fields
  sizeof(mbf_msgs__action__ExePath_GetResult_Event),
  false,  // has_any_key_member_
  mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__ExePath_GetResult_Event_message_member_array,  // message members
  mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__ExePath_GetResult_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__ExePath_GetResult_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__ExePath_GetResult_Event_message_type_support_handle = {
  0,
  &mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__ExePath_GetResult_Event_message_members,
  get_message_typesupport_handle_function,
  &mbf_msgs__action__ExePath_GetResult_Event__get_type_hash,
  &mbf_msgs__action__ExePath_GetResult_Event__get_type_description,
  &mbf_msgs__action__ExePath_GetResult_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mbf_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_GetResult_Event)() {
  mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__ExePath_GetResult_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__ExePath_GetResult_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_GetResult_Request)();
  mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__ExePath_GetResult_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_GetResult_Response)();
  if (!mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__ExePath_GetResult_Event_message_type_support_handle.typesupport_identifier) {
    mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__ExePath_GetResult_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__ExePath_GetResult_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "mbf_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers mbf_msgs__action__detail__exe_path__rosidl_typesupport_introspection_c__ExePath_GetResult_service_members = {
  "mbf_msgs__action",  // service namespace
  "ExePath_GetResult",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // mbf_msgs__action__detail__exe_path__rosidl_typesupport_introspection_c__ExePath_GetResult_Request_message_type_support_handle,
  NULL,  // response message
  // mbf_msgs__action__detail__exe_path__rosidl_typesupport_introspection_c__ExePath_GetResult_Response_message_type_support_handle
  NULL  // event_message
  // mbf_msgs__action__detail__exe_path__rosidl_typesupport_introspection_c__ExePath_GetResult_Response_message_type_support_handle
};


static rosidl_service_type_support_t mbf_msgs__action__detail__exe_path__rosidl_typesupport_introspection_c__ExePath_GetResult_service_type_support_handle = {
  0,
  &mbf_msgs__action__detail__exe_path__rosidl_typesupport_introspection_c__ExePath_GetResult_service_members,
  get_service_typesupport_handle_function,
  &mbf_msgs__action__ExePath_GetResult_Request__rosidl_typesupport_introspection_c__ExePath_GetResult_Request_message_type_support_handle,
  &mbf_msgs__action__ExePath_GetResult_Response__rosidl_typesupport_introspection_c__ExePath_GetResult_Response_message_type_support_handle,
  &mbf_msgs__action__ExePath_GetResult_Event__rosidl_typesupport_introspection_c__ExePath_GetResult_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    mbf_msgs,
    action,
    ExePath_GetResult
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    mbf_msgs,
    action,
    ExePath_GetResult
  ),
  &mbf_msgs__action__ExePath_GetResult__get_type_hash,
  &mbf_msgs__action__ExePath_GetResult__get_type_description,
  &mbf_msgs__action__ExePath_GetResult__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_GetResult_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_GetResult_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_GetResult_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mbf_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_GetResult)(void) {
  if (!mbf_msgs__action__detail__exe_path__rosidl_typesupport_introspection_c__ExePath_GetResult_service_type_support_handle.typesupport_identifier) {
    mbf_msgs__action__detail__exe_path__rosidl_typesupport_introspection_c__ExePath_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)mbf_msgs__action__detail__exe_path__rosidl_typesupport_introspection_c__ExePath_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_GetResult_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_GetResult_Event)()->data;
  }

  return &mbf_msgs__action__detail__exe_path__rosidl_typesupport_introspection_c__ExePath_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "mbf_msgs/action/detail/exe_path__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mbf_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__functions.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "mbf_msgs/action/exe_path.h"
// Member `feedback`
// already included above
// #include "mbf_msgs/action/detail/exe_path__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mbf_msgs__action__ExePath_FeedbackMessage__rosidl_typesupport_introspection_c__ExePath_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mbf_msgs__action__ExePath_FeedbackMessage__init(message_memory);
}

void mbf_msgs__action__ExePath_FeedbackMessage__rosidl_typesupport_introspection_c__ExePath_FeedbackMessage_fini_function(void * message_memory)
{
  mbf_msgs__action__ExePath_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mbf_msgs__action__ExePath_FeedbackMessage__rosidl_typesupport_introspection_c__ExePath_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mbf_msgs__action__ExePath_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mbf_msgs__action__ExePath_FeedbackMessage__rosidl_typesupport_introspection_c__ExePath_FeedbackMessage_message_members = {
  "mbf_msgs__action",  // message namespace
  "ExePath_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(mbf_msgs__action__ExePath_FeedbackMessage),
  false,  // has_any_key_member_
  mbf_msgs__action__ExePath_FeedbackMessage__rosidl_typesupport_introspection_c__ExePath_FeedbackMessage_message_member_array,  // message members
  mbf_msgs__action__ExePath_FeedbackMessage__rosidl_typesupport_introspection_c__ExePath_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  mbf_msgs__action__ExePath_FeedbackMessage__rosidl_typesupport_introspection_c__ExePath_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mbf_msgs__action__ExePath_FeedbackMessage__rosidl_typesupport_introspection_c__ExePath_FeedbackMessage_message_type_support_handle = {
  0,
  &mbf_msgs__action__ExePath_FeedbackMessage__rosidl_typesupport_introspection_c__ExePath_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
  &mbf_msgs__action__ExePath_FeedbackMessage__get_type_hash,
  &mbf_msgs__action__ExePath_FeedbackMessage__get_type_description,
  &mbf_msgs__action__ExePath_FeedbackMessage__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mbf_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_FeedbackMessage)() {
  mbf_msgs__action__ExePath_FeedbackMessage__rosidl_typesupport_introspection_c__ExePath_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  mbf_msgs__action__ExePath_FeedbackMessage__rosidl_typesupport_introspection_c__ExePath_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, action, ExePath_Feedback)();
  if (!mbf_msgs__action__ExePath_FeedbackMessage__rosidl_typesupport_introspection_c__ExePath_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    mbf_msgs__action__ExePath_FeedbackMessage__rosidl_typesupport_introspection_c__ExePath_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mbf_msgs__action__ExePath_FeedbackMessage__rosidl_typesupport_introspection_c__ExePath_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
