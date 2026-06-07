// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from mbf_msgs:srv/SetTestRobotState.idl
// generated code does not contain a copyright notice
#include "mbf_msgs/srv/detail/set_test_robot_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "mbf_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "mbf_msgs/srv/detail/set_test_robot_state__struct.h"
#include "mbf_msgs/srv/detail/set_test_robot_state__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "geometry_msgs/msg/detail/transform_stamped__functions.h"  // transform
#include "geometry_msgs/msg/detail/twist_stamped__functions.h"  // velocity_robot

// forward declare type support functions

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
bool cdr_serialize_geometry_msgs__msg__TransformStamped(
  const geometry_msgs__msg__TransformStamped * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
bool cdr_deserialize_geometry_msgs__msg__TransformStamped(
  eprosima::fastcdr::Cdr & cdr,
  geometry_msgs__msg__TransformStamped * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
size_t get_serialized_size_geometry_msgs__msg__TransformStamped(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
size_t max_serialized_size_geometry_msgs__msg__TransformStamped(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
bool cdr_serialize_key_geometry_msgs__msg__TransformStamped(
  const geometry_msgs__msg__TransformStamped * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
size_t get_serialized_size_key_geometry_msgs__msg__TransformStamped(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
size_t max_serialized_size_key_geometry_msgs__msg__TransformStamped(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, TransformStamped)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
bool cdr_serialize_geometry_msgs__msg__TwistStamped(
  const geometry_msgs__msg__TwistStamped * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
bool cdr_deserialize_geometry_msgs__msg__TwistStamped(
  eprosima::fastcdr::Cdr & cdr,
  geometry_msgs__msg__TwistStamped * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
size_t get_serialized_size_geometry_msgs__msg__TwistStamped(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
size_t max_serialized_size_geometry_msgs__msg__TwistStamped(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
bool cdr_serialize_key_geometry_msgs__msg__TwistStamped(
  const geometry_msgs__msg__TwistStamped * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
size_t get_serialized_size_key_geometry_msgs__msg__TwistStamped(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
size_t max_serialized_size_key_geometry_msgs__msg__TwistStamped(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, TwistStamped)();


using _SetTestRobotState_Request__ros_msg_type = mbf_msgs__srv__SetTestRobotState_Request;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mbf_msgs
bool cdr_serialize_mbf_msgs__srv__SetTestRobotState_Request(
  const mbf_msgs__srv__SetTestRobotState_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: transform
  {
    cdr_serialize_geometry_msgs__msg__TransformStamped(
      &ros_message->transform, cdr);
  }

  // Field name: set_velocity
  {
    cdr << (ros_message->set_velocity ? true : false);
  }

  // Field name: velocity_robot
  {
    cdr_serialize_geometry_msgs__msg__TwistStamped(
      &ros_message->velocity_robot, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mbf_msgs
bool cdr_deserialize_mbf_msgs__srv__SetTestRobotState_Request(
  eprosima::fastcdr::Cdr & cdr,
  mbf_msgs__srv__SetTestRobotState_Request * ros_message)
{
  // Field name: transform
  {
    cdr_deserialize_geometry_msgs__msg__TransformStamped(cdr, &ros_message->transform);
  }

  // Field name: set_velocity
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->set_velocity = tmp ? true : false;
  }

  // Field name: velocity_robot
  {
    cdr_deserialize_geometry_msgs__msg__TwistStamped(cdr, &ros_message->velocity_robot);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mbf_msgs
size_t get_serialized_size_mbf_msgs__srv__SetTestRobotState_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SetTestRobotState_Request__ros_msg_type * ros_message = static_cast<const _SetTestRobotState_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: transform
  current_alignment += get_serialized_size_geometry_msgs__msg__TransformStamped(
    &(ros_message->transform), current_alignment);

  // Field name: set_velocity
  {
    size_t item_size = sizeof(ros_message->set_velocity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: velocity_robot
  current_alignment += get_serialized_size_geometry_msgs__msg__TwistStamped(
    &(ros_message->velocity_robot), current_alignment);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mbf_msgs
size_t max_serialized_size_mbf_msgs__srv__SetTestRobotState_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: transform
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__TransformStamped(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: set_velocity
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: velocity_robot
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__TwistStamped(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = mbf_msgs__srv__SetTestRobotState_Request;
    is_plain =
      (
      offsetof(DataType, velocity_robot) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mbf_msgs
bool cdr_serialize_key_mbf_msgs__srv__SetTestRobotState_Request(
  const mbf_msgs__srv__SetTestRobotState_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: transform
  {
    cdr_serialize_key_geometry_msgs__msg__TransformStamped(
      &ros_message->transform, cdr);
  }

  // Field name: set_velocity
  {
    cdr << (ros_message->set_velocity ? true : false);
  }

  // Field name: velocity_robot
  {
    cdr_serialize_key_geometry_msgs__msg__TwistStamped(
      &ros_message->velocity_robot, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mbf_msgs
size_t get_serialized_size_key_mbf_msgs__srv__SetTestRobotState_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SetTestRobotState_Request__ros_msg_type * ros_message = static_cast<const _SetTestRobotState_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: transform
  current_alignment += get_serialized_size_key_geometry_msgs__msg__TransformStamped(
    &(ros_message->transform), current_alignment);

  // Field name: set_velocity
  {
    size_t item_size = sizeof(ros_message->set_velocity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: velocity_robot
  current_alignment += get_serialized_size_key_geometry_msgs__msg__TwistStamped(
    &(ros_message->velocity_robot), current_alignment);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mbf_msgs
size_t max_serialized_size_key_mbf_msgs__srv__SetTestRobotState_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: transform
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__TransformStamped(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: set_velocity
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: velocity_robot
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__TwistStamped(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = mbf_msgs__srv__SetTestRobotState_Request;
    is_plain =
      (
      offsetof(DataType, velocity_robot) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _SetTestRobotState_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const mbf_msgs__srv__SetTestRobotState_Request * ros_message = static_cast<const mbf_msgs__srv__SetTestRobotState_Request *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_mbf_msgs__srv__SetTestRobotState_Request(ros_message, cdr);
}

static bool _SetTestRobotState_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  mbf_msgs__srv__SetTestRobotState_Request * ros_message = static_cast<mbf_msgs__srv__SetTestRobotState_Request *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_mbf_msgs__srv__SetTestRobotState_Request(cdr, ros_message);
}

static uint32_t _SetTestRobotState_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_mbf_msgs__srv__SetTestRobotState_Request(
      untyped_ros_message, 0));
}

static size_t _SetTestRobotState_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_mbf_msgs__srv__SetTestRobotState_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SetTestRobotState_Request = {
  "mbf_msgs::srv",
  "SetTestRobotState_Request",
  _SetTestRobotState_Request__cdr_serialize,
  _SetTestRobotState_Request__cdr_deserialize,
  _SetTestRobotState_Request__get_serialized_size,
  _SetTestRobotState_Request__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _SetTestRobotState_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SetTestRobotState_Request,
  get_message_typesupport_handle_function,
  &mbf_msgs__srv__SetTestRobotState_Request__get_type_hash,
  &mbf_msgs__srv__SetTestRobotState_Request__get_type_description,
  &mbf_msgs__srv__SetTestRobotState_Request__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mbf_msgs, srv, SetTestRobotState_Request)() {
  return &_SetTestRobotState_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <cstddef>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "mbf_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "mbf_msgs/srv/detail/set_test_robot_state__struct.h"
// already included above
// #include "mbf_msgs/srv/detail/set_test_robot_state__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _SetTestRobotState_Response__ros_msg_type = mbf_msgs__srv__SetTestRobotState_Response;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mbf_msgs
bool cdr_serialize_mbf_msgs__srv__SetTestRobotState_Response(
  const mbf_msgs__srv__SetTestRobotState_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mbf_msgs
bool cdr_deserialize_mbf_msgs__srv__SetTestRobotState_Response(
  eprosima::fastcdr::Cdr & cdr,
  mbf_msgs__srv__SetTestRobotState_Response * ros_message)
{
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mbf_msgs
size_t get_serialized_size_mbf_msgs__srv__SetTestRobotState_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SetTestRobotState_Response__ros_msg_type * ros_message = static_cast<const _SetTestRobotState_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mbf_msgs
size_t max_serialized_size_mbf_msgs__srv__SetTestRobotState_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: success
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = mbf_msgs__srv__SetTestRobotState_Response;
    is_plain =
      (
      offsetof(DataType, success) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mbf_msgs
bool cdr_serialize_key_mbf_msgs__srv__SetTestRobotState_Response(
  const mbf_msgs__srv__SetTestRobotState_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mbf_msgs
size_t get_serialized_size_key_mbf_msgs__srv__SetTestRobotState_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SetTestRobotState_Response__ros_msg_type * ros_message = static_cast<const _SetTestRobotState_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mbf_msgs
size_t max_serialized_size_key_mbf_msgs__srv__SetTestRobotState_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: success
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = mbf_msgs__srv__SetTestRobotState_Response;
    is_plain =
      (
      offsetof(DataType, success) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _SetTestRobotState_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const mbf_msgs__srv__SetTestRobotState_Response * ros_message = static_cast<const mbf_msgs__srv__SetTestRobotState_Response *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_mbf_msgs__srv__SetTestRobotState_Response(ros_message, cdr);
}

static bool _SetTestRobotState_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  mbf_msgs__srv__SetTestRobotState_Response * ros_message = static_cast<mbf_msgs__srv__SetTestRobotState_Response *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_mbf_msgs__srv__SetTestRobotState_Response(cdr, ros_message);
}

static uint32_t _SetTestRobotState_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_mbf_msgs__srv__SetTestRobotState_Response(
      untyped_ros_message, 0));
}

static size_t _SetTestRobotState_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_mbf_msgs__srv__SetTestRobotState_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SetTestRobotState_Response = {
  "mbf_msgs::srv",
  "SetTestRobotState_Response",
  _SetTestRobotState_Response__cdr_serialize,
  _SetTestRobotState_Response__cdr_deserialize,
  _SetTestRobotState_Response__get_serialized_size,
  _SetTestRobotState_Response__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _SetTestRobotState_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SetTestRobotState_Response,
  get_message_typesupport_handle_function,
  &mbf_msgs__srv__SetTestRobotState_Response__get_type_hash,
  &mbf_msgs__srv__SetTestRobotState_Response__get_type_description,
  &mbf_msgs__srv__SetTestRobotState_Response__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mbf_msgs, srv, SetTestRobotState_Response)() {
  return &_SetTestRobotState_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <cstddef>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "mbf_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "mbf_msgs/srv/detail/set_test_robot_state__struct.h"
// already included above
// #include "mbf_msgs/srv/detail/set_test_robot_state__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "service_msgs/msg/detail/service_event_info__functions.h"  // info

// forward declare type support functions

bool cdr_serialize_mbf_msgs__srv__SetTestRobotState_Request(
  const mbf_msgs__srv__SetTestRobotState_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_mbf_msgs__srv__SetTestRobotState_Request(
  eprosima::fastcdr::Cdr & cdr,
  mbf_msgs__srv__SetTestRobotState_Request * ros_message);

size_t get_serialized_size_mbf_msgs__srv__SetTestRobotState_Request(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_mbf_msgs__srv__SetTestRobotState_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_mbf_msgs__srv__SetTestRobotState_Request(
  const mbf_msgs__srv__SetTestRobotState_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_mbf_msgs__srv__SetTestRobotState_Request(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_mbf_msgs__srv__SetTestRobotState_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mbf_msgs, srv, SetTestRobotState_Request)();

bool cdr_serialize_mbf_msgs__srv__SetTestRobotState_Response(
  const mbf_msgs__srv__SetTestRobotState_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_mbf_msgs__srv__SetTestRobotState_Response(
  eprosima::fastcdr::Cdr & cdr,
  mbf_msgs__srv__SetTestRobotState_Response * ros_message);

size_t get_serialized_size_mbf_msgs__srv__SetTestRobotState_Response(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_mbf_msgs__srv__SetTestRobotState_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_mbf_msgs__srv__SetTestRobotState_Response(
  const mbf_msgs__srv__SetTestRobotState_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_mbf_msgs__srv__SetTestRobotState_Response(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_mbf_msgs__srv__SetTestRobotState_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mbf_msgs, srv, SetTestRobotState_Response)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
bool cdr_serialize_service_msgs__msg__ServiceEventInfo(
  const service_msgs__msg__ServiceEventInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
bool cdr_deserialize_service_msgs__msg__ServiceEventInfo(
  eprosima::fastcdr::Cdr & cdr,
  service_msgs__msg__ServiceEventInfo * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
size_t get_serialized_size_service_msgs__msg__ServiceEventInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
size_t max_serialized_size_service_msgs__msg__ServiceEventInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
bool cdr_serialize_key_service_msgs__msg__ServiceEventInfo(
  const service_msgs__msg__ServiceEventInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
size_t get_serialized_size_key_service_msgs__msg__ServiceEventInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
size_t max_serialized_size_key_service_msgs__msg__ServiceEventInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mbf_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, service_msgs, msg, ServiceEventInfo)();


using _SetTestRobotState_Event__ros_msg_type = mbf_msgs__srv__SetTestRobotState_Event;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mbf_msgs
bool cdr_serialize_mbf_msgs__srv__SetTestRobotState_Event(
  const mbf_msgs__srv__SetTestRobotState_Event * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: info
  {
    cdr_serialize_service_msgs__msg__ServiceEventInfo(
      &ros_message->info, cdr);
  }

  // Field name: request
  {
    size_t size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_mbf_msgs__srv__SetTestRobotState_Request(
        &array_ptr[i], cdr);
    }
  }

  // Field name: response
  {
    size_t size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_mbf_msgs__srv__SetTestRobotState_Response(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mbf_msgs
bool cdr_deserialize_mbf_msgs__srv__SetTestRobotState_Event(
  eprosima::fastcdr::Cdr & cdr,
  mbf_msgs__srv__SetTestRobotState_Event * ros_message)
{
  // Field name: info
  {
    cdr_deserialize_service_msgs__msg__ServiceEventInfo(cdr, &ros_message->info);
  }

  // Field name: request
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.get_state();
    bool correct_size = cdr.jump(size);
    cdr.set_state(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->request.data) {
      mbf_msgs__srv__SetTestRobotState_Request__Sequence__fini(&ros_message->request);
    }
    if (!mbf_msgs__srv__SetTestRobotState_Request__Sequence__init(&ros_message->request, size)) {
      fprintf(stderr, "failed to create array for field 'request'");
      return false;
    }
    auto array_ptr = ros_message->request.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_mbf_msgs__srv__SetTestRobotState_Request(cdr, &array_ptr[i]);
    }
  }

  // Field name: response
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.get_state();
    bool correct_size = cdr.jump(size);
    cdr.set_state(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->response.data) {
      mbf_msgs__srv__SetTestRobotState_Response__Sequence__fini(&ros_message->response);
    }
    if (!mbf_msgs__srv__SetTestRobotState_Response__Sequence__init(&ros_message->response, size)) {
      fprintf(stderr, "failed to create array for field 'response'");
      return false;
    }
    auto array_ptr = ros_message->response.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_mbf_msgs__srv__SetTestRobotState_Response(cdr, &array_ptr[i]);
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mbf_msgs
size_t get_serialized_size_mbf_msgs__srv__SetTestRobotState_Event(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SetTestRobotState_Event__ros_msg_type * ros_message = static_cast<const _SetTestRobotState_Event__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: info
  current_alignment += get_serialized_size_service_msgs__msg__ServiceEventInfo(
    &(ros_message->info), current_alignment);

  // Field name: request
  {
    size_t array_size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_mbf_msgs__srv__SetTestRobotState_Request(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: response
  {
    size_t array_size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_mbf_msgs__srv__SetTestRobotState_Response(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mbf_msgs
size_t max_serialized_size_mbf_msgs__srv__SetTestRobotState_Event(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: info
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_service_msgs__msg__ServiceEventInfo(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: request
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_mbf_msgs__srv__SetTestRobotState_Request(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: response
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_mbf_msgs__srv__SetTestRobotState_Response(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = mbf_msgs__srv__SetTestRobotState_Event;
    is_plain =
      (
      offsetof(DataType, response) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mbf_msgs
bool cdr_serialize_key_mbf_msgs__srv__SetTestRobotState_Event(
  const mbf_msgs__srv__SetTestRobotState_Event * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: info
  {
    cdr_serialize_key_service_msgs__msg__ServiceEventInfo(
      &ros_message->info, cdr);
  }

  // Field name: request
  {
    size_t size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_mbf_msgs__srv__SetTestRobotState_Request(
        &array_ptr[i], cdr);
    }
  }

  // Field name: response
  {
    size_t size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_mbf_msgs__srv__SetTestRobotState_Response(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mbf_msgs
size_t get_serialized_size_key_mbf_msgs__srv__SetTestRobotState_Event(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SetTestRobotState_Event__ros_msg_type * ros_message = static_cast<const _SetTestRobotState_Event__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: info
  current_alignment += get_serialized_size_key_service_msgs__msg__ServiceEventInfo(
    &(ros_message->info), current_alignment);

  // Field name: request
  {
    size_t array_size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_mbf_msgs__srv__SetTestRobotState_Request(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: response
  {
    size_t array_size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_mbf_msgs__srv__SetTestRobotState_Response(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mbf_msgs
size_t max_serialized_size_key_mbf_msgs__srv__SetTestRobotState_Event(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: info
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_service_msgs__msg__ServiceEventInfo(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: request
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_mbf_msgs__srv__SetTestRobotState_Request(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: response
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_mbf_msgs__srv__SetTestRobotState_Response(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = mbf_msgs__srv__SetTestRobotState_Event;
    is_plain =
      (
      offsetof(DataType, response) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _SetTestRobotState_Event__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const mbf_msgs__srv__SetTestRobotState_Event * ros_message = static_cast<const mbf_msgs__srv__SetTestRobotState_Event *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_mbf_msgs__srv__SetTestRobotState_Event(ros_message, cdr);
}

static bool _SetTestRobotState_Event__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  mbf_msgs__srv__SetTestRobotState_Event * ros_message = static_cast<mbf_msgs__srv__SetTestRobotState_Event *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_mbf_msgs__srv__SetTestRobotState_Event(cdr, ros_message);
}

static uint32_t _SetTestRobotState_Event__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_mbf_msgs__srv__SetTestRobotState_Event(
      untyped_ros_message, 0));
}

static size_t _SetTestRobotState_Event__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_mbf_msgs__srv__SetTestRobotState_Event(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SetTestRobotState_Event = {
  "mbf_msgs::srv",
  "SetTestRobotState_Event",
  _SetTestRobotState_Event__cdr_serialize,
  _SetTestRobotState_Event__cdr_deserialize,
  _SetTestRobotState_Event__get_serialized_size,
  _SetTestRobotState_Event__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _SetTestRobotState_Event__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SetTestRobotState_Event,
  get_message_typesupport_handle_function,
  &mbf_msgs__srv__SetTestRobotState_Event__get_type_hash,
  &mbf_msgs__srv__SetTestRobotState_Event__get_type_description,
  &mbf_msgs__srv__SetTestRobotState_Event__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mbf_msgs, srv, SetTestRobotState_Event)() {
  return &_SetTestRobotState_Event__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "mbf_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "mbf_msgs/srv/set_test_robot_state.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t SetTestRobotState__callbacks = {
  "mbf_msgs::srv",
  "SetTestRobotState",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mbf_msgs, srv, SetTestRobotState_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mbf_msgs, srv, SetTestRobotState_Response)(),
};

static rosidl_service_type_support_t SetTestRobotState__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &SetTestRobotState__callbacks,
  get_service_typesupport_handle_function,
  &_SetTestRobotState_Request__type_support,
  &_SetTestRobotState_Response__type_support,
  &_SetTestRobotState_Event__type_support,
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

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mbf_msgs, srv, SetTestRobotState)() {
  return &SetTestRobotState__handle;
}

#if defined(__cplusplus)
}
#endif
