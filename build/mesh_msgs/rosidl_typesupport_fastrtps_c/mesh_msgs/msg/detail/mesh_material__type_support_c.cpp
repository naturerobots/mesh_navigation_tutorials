// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from mesh_msgs:msg/MeshMaterial.idl
// generated code does not contain a copyright notice
#include "mesh_msgs/msg/detail/mesh_material__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "mesh_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "mesh_msgs/msg/detail/mesh_material__struct.h"
#include "mesh_msgs/msg/detail/mesh_material__functions.h"
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

#include "std_msgs/msg/detail/color_rgba__functions.h"  // color

// forward declare type support functions

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mesh_msgs
bool cdr_serialize_std_msgs__msg__ColorRGBA(
  const std_msgs__msg__ColorRGBA * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mesh_msgs
bool cdr_deserialize_std_msgs__msg__ColorRGBA(
  eprosima::fastcdr::Cdr & cdr,
  std_msgs__msg__ColorRGBA * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mesh_msgs
size_t get_serialized_size_std_msgs__msg__ColorRGBA(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mesh_msgs
size_t max_serialized_size_std_msgs__msg__ColorRGBA(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mesh_msgs
bool cdr_serialize_key_std_msgs__msg__ColorRGBA(
  const std_msgs__msg__ColorRGBA * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mesh_msgs
size_t get_serialized_size_key_std_msgs__msg__ColorRGBA(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mesh_msgs
size_t max_serialized_size_key_std_msgs__msg__ColorRGBA(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mesh_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, ColorRGBA)();


using _MeshMaterial__ros_msg_type = mesh_msgs__msg__MeshMaterial;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mesh_msgs
bool cdr_serialize_mesh_msgs__msg__MeshMaterial(
  const mesh_msgs__msg__MeshMaterial * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: texture_index
  {
    cdr << ros_message->texture_index;
  }

  // Field name: color
  {
    cdr_serialize_std_msgs__msg__ColorRGBA(
      &ros_message->color, cdr);
  }

  // Field name: has_texture
  {
    cdr << (ros_message->has_texture ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mesh_msgs
bool cdr_deserialize_mesh_msgs__msg__MeshMaterial(
  eprosima::fastcdr::Cdr & cdr,
  mesh_msgs__msg__MeshMaterial * ros_message)
{
  // Field name: texture_index
  {
    cdr >> ros_message->texture_index;
  }

  // Field name: color
  {
    cdr_deserialize_std_msgs__msg__ColorRGBA(cdr, &ros_message->color);
  }

  // Field name: has_texture
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->has_texture = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mesh_msgs
size_t get_serialized_size_mesh_msgs__msg__MeshMaterial(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MeshMaterial__ros_msg_type * ros_message = static_cast<const _MeshMaterial__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: texture_index
  {
    size_t item_size = sizeof(ros_message->texture_index);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: color
  current_alignment += get_serialized_size_std_msgs__msg__ColorRGBA(
    &(ros_message->color), current_alignment);

  // Field name: has_texture
  {
    size_t item_size = sizeof(ros_message->has_texture);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mesh_msgs
size_t max_serialized_size_mesh_msgs__msg__MeshMaterial(
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

  // Field name: texture_index
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: color
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__ColorRGBA(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: has_texture
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
    using DataType = mesh_msgs__msg__MeshMaterial;
    is_plain =
      (
      offsetof(DataType, has_texture) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mesh_msgs
bool cdr_serialize_key_mesh_msgs__msg__MeshMaterial(
  const mesh_msgs__msg__MeshMaterial * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: texture_index
  {
    cdr << ros_message->texture_index;
  }

  // Field name: color
  {
    cdr_serialize_key_std_msgs__msg__ColorRGBA(
      &ros_message->color, cdr);
  }

  // Field name: has_texture
  {
    cdr << (ros_message->has_texture ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mesh_msgs
size_t get_serialized_size_key_mesh_msgs__msg__MeshMaterial(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MeshMaterial__ros_msg_type * ros_message = static_cast<const _MeshMaterial__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: texture_index
  {
    size_t item_size = sizeof(ros_message->texture_index);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: color
  current_alignment += get_serialized_size_key_std_msgs__msg__ColorRGBA(
    &(ros_message->color), current_alignment);

  // Field name: has_texture
  {
    size_t item_size = sizeof(ros_message->has_texture);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mesh_msgs
size_t max_serialized_size_key_mesh_msgs__msg__MeshMaterial(
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
  // Field name: texture_index
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: color
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_std_msgs__msg__ColorRGBA(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: has_texture
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
    using DataType = mesh_msgs__msg__MeshMaterial;
    is_plain =
      (
      offsetof(DataType, has_texture) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _MeshMaterial__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const mesh_msgs__msg__MeshMaterial * ros_message = static_cast<const mesh_msgs__msg__MeshMaterial *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_mesh_msgs__msg__MeshMaterial(ros_message, cdr);
}

static bool _MeshMaterial__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  mesh_msgs__msg__MeshMaterial * ros_message = static_cast<mesh_msgs__msg__MeshMaterial *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_mesh_msgs__msg__MeshMaterial(cdr, ros_message);
}

static uint32_t _MeshMaterial__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_mesh_msgs__msg__MeshMaterial(
      untyped_ros_message, 0));
}

static size_t _MeshMaterial__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_mesh_msgs__msg__MeshMaterial(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_MeshMaterial = {
  "mesh_msgs::msg",
  "MeshMaterial",
  _MeshMaterial__cdr_serialize,
  _MeshMaterial__cdr_deserialize,
  _MeshMaterial__get_serialized_size,
  _MeshMaterial__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _MeshMaterial__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MeshMaterial,
  get_message_typesupport_handle_function,
  &mesh_msgs__msg__MeshMaterial__get_type_hash,
  &mesh_msgs__msg__MeshMaterial__get_type_description,
  &mesh_msgs__msg__MeshMaterial__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mesh_msgs, msg, MeshMaterial)() {
  return &_MeshMaterial__type_support;
}

#if defined(__cplusplus)
}
#endif
