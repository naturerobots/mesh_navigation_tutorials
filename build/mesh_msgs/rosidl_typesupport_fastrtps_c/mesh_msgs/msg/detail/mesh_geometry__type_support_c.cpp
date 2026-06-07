// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from mesh_msgs:msg/MeshGeometry.idl
// generated code does not contain a copyright notice
#include "mesh_msgs/msg/detail/mesh_geometry__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "mesh_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "mesh_msgs/msg/detail/mesh_geometry__struct.h"
#include "mesh_msgs/msg/detail/mesh_geometry__functions.h"
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

#include "geometry_msgs/msg/detail/point__functions.h"  // vertex_normals, vertices
#include "mesh_msgs/msg/detail/mesh_triangle_indices__functions.h"  // faces

// forward declare type support functions

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mesh_msgs
bool cdr_serialize_geometry_msgs__msg__Point(
  const geometry_msgs__msg__Point * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mesh_msgs
bool cdr_deserialize_geometry_msgs__msg__Point(
  eprosima::fastcdr::Cdr & cdr,
  geometry_msgs__msg__Point * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mesh_msgs
size_t get_serialized_size_geometry_msgs__msg__Point(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mesh_msgs
size_t max_serialized_size_geometry_msgs__msg__Point(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mesh_msgs
bool cdr_serialize_key_geometry_msgs__msg__Point(
  const geometry_msgs__msg__Point * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mesh_msgs
size_t get_serialized_size_key_geometry_msgs__msg__Point(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mesh_msgs
size_t max_serialized_size_key_geometry_msgs__msg__Point(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mesh_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Point)();

bool cdr_serialize_mesh_msgs__msg__MeshTriangleIndices(
  const mesh_msgs__msg__MeshTriangleIndices * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_mesh_msgs__msg__MeshTriangleIndices(
  eprosima::fastcdr::Cdr & cdr,
  mesh_msgs__msg__MeshTriangleIndices * ros_message);

size_t get_serialized_size_mesh_msgs__msg__MeshTriangleIndices(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_mesh_msgs__msg__MeshTriangleIndices(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_mesh_msgs__msg__MeshTriangleIndices(
  const mesh_msgs__msg__MeshTriangleIndices * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_mesh_msgs__msg__MeshTriangleIndices(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_mesh_msgs__msg__MeshTriangleIndices(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mesh_msgs, msg, MeshTriangleIndices)();


using _MeshGeometry__ros_msg_type = mesh_msgs__msg__MeshGeometry;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mesh_msgs
bool cdr_serialize_mesh_msgs__msg__MeshGeometry(
  const mesh_msgs__msg__MeshGeometry * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: vertices
  {
    size_t size = ros_message->vertices.size;
    auto array_ptr = ros_message->vertices.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_geometry_msgs__msg__Point(
        &array_ptr[i], cdr);
    }
  }

  // Field name: vertex_normals
  {
    size_t size = ros_message->vertex_normals.size;
    auto array_ptr = ros_message->vertex_normals.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_geometry_msgs__msg__Point(
        &array_ptr[i], cdr);
    }
  }

  // Field name: faces
  {
    size_t size = ros_message->faces.size;
    auto array_ptr = ros_message->faces.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_mesh_msgs__msg__MeshTriangleIndices(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mesh_msgs
bool cdr_deserialize_mesh_msgs__msg__MeshGeometry(
  eprosima::fastcdr::Cdr & cdr,
  mesh_msgs__msg__MeshGeometry * ros_message)
{
  // Field name: vertices
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

    if (ros_message->vertices.data) {
      geometry_msgs__msg__Point__Sequence__fini(&ros_message->vertices);
    }
    if (!geometry_msgs__msg__Point__Sequence__init(&ros_message->vertices, size)) {
      fprintf(stderr, "failed to create array for field 'vertices'");
      return false;
    }
    auto array_ptr = ros_message->vertices.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_geometry_msgs__msg__Point(cdr, &array_ptr[i]);
    }
  }

  // Field name: vertex_normals
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

    if (ros_message->vertex_normals.data) {
      geometry_msgs__msg__Point__Sequence__fini(&ros_message->vertex_normals);
    }
    if (!geometry_msgs__msg__Point__Sequence__init(&ros_message->vertex_normals, size)) {
      fprintf(stderr, "failed to create array for field 'vertex_normals'");
      return false;
    }
    auto array_ptr = ros_message->vertex_normals.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_geometry_msgs__msg__Point(cdr, &array_ptr[i]);
    }
  }

  // Field name: faces
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

    if (ros_message->faces.data) {
      mesh_msgs__msg__MeshTriangleIndices__Sequence__fini(&ros_message->faces);
    }
    if (!mesh_msgs__msg__MeshTriangleIndices__Sequence__init(&ros_message->faces, size)) {
      fprintf(stderr, "failed to create array for field 'faces'");
      return false;
    }
    auto array_ptr = ros_message->faces.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_mesh_msgs__msg__MeshTriangleIndices(cdr, &array_ptr[i]);
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mesh_msgs
size_t get_serialized_size_mesh_msgs__msg__MeshGeometry(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MeshGeometry__ros_msg_type * ros_message = static_cast<const _MeshGeometry__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: vertices
  {
    size_t array_size = ros_message->vertices.size;
    auto array_ptr = ros_message->vertices.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_geometry_msgs__msg__Point(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: vertex_normals
  {
    size_t array_size = ros_message->vertex_normals.size;
    auto array_ptr = ros_message->vertex_normals.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_geometry_msgs__msg__Point(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: faces
  {
    size_t array_size = ros_message->faces.size;
    auto array_ptr = ros_message->faces.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_mesh_msgs__msg__MeshTriangleIndices(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mesh_msgs
size_t max_serialized_size_mesh_msgs__msg__MeshGeometry(
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

  // Field name: vertices
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Point(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: vertex_normals
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Point(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: faces
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_mesh_msgs__msg__MeshTriangleIndices(
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
    using DataType = mesh_msgs__msg__MeshGeometry;
    is_plain =
      (
      offsetof(DataType, faces) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mesh_msgs
bool cdr_serialize_key_mesh_msgs__msg__MeshGeometry(
  const mesh_msgs__msg__MeshGeometry * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: vertices
  {
    size_t size = ros_message->vertices.size;
    auto array_ptr = ros_message->vertices.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_geometry_msgs__msg__Point(
        &array_ptr[i], cdr);
    }
  }

  // Field name: vertex_normals
  {
    size_t size = ros_message->vertex_normals.size;
    auto array_ptr = ros_message->vertex_normals.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_geometry_msgs__msg__Point(
        &array_ptr[i], cdr);
    }
  }

  // Field name: faces
  {
    size_t size = ros_message->faces.size;
    auto array_ptr = ros_message->faces.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_mesh_msgs__msg__MeshTriangleIndices(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mesh_msgs
size_t get_serialized_size_key_mesh_msgs__msg__MeshGeometry(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MeshGeometry__ros_msg_type * ros_message = static_cast<const _MeshGeometry__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: vertices
  {
    size_t array_size = ros_message->vertices.size;
    auto array_ptr = ros_message->vertices.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_geometry_msgs__msg__Point(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: vertex_normals
  {
    size_t array_size = ros_message->vertex_normals.size;
    auto array_ptr = ros_message->vertex_normals.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_geometry_msgs__msg__Point(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: faces
  {
    size_t array_size = ros_message->faces.size;
    auto array_ptr = ros_message->faces.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_mesh_msgs__msg__MeshTriangleIndices(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mesh_msgs
size_t max_serialized_size_key_mesh_msgs__msg__MeshGeometry(
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
  // Field name: vertices
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__Point(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: vertex_normals
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__Point(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: faces
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_mesh_msgs__msg__MeshTriangleIndices(
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
    using DataType = mesh_msgs__msg__MeshGeometry;
    is_plain =
      (
      offsetof(DataType, faces) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _MeshGeometry__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const mesh_msgs__msg__MeshGeometry * ros_message = static_cast<const mesh_msgs__msg__MeshGeometry *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_mesh_msgs__msg__MeshGeometry(ros_message, cdr);
}

static bool _MeshGeometry__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  mesh_msgs__msg__MeshGeometry * ros_message = static_cast<mesh_msgs__msg__MeshGeometry *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_mesh_msgs__msg__MeshGeometry(cdr, ros_message);
}

static uint32_t _MeshGeometry__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_mesh_msgs__msg__MeshGeometry(
      untyped_ros_message, 0));
}

static size_t _MeshGeometry__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_mesh_msgs__msg__MeshGeometry(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_MeshGeometry = {
  "mesh_msgs::msg",
  "MeshGeometry",
  _MeshGeometry__cdr_serialize,
  _MeshGeometry__cdr_deserialize,
  _MeshGeometry__get_serialized_size,
  _MeshGeometry__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _MeshGeometry__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MeshGeometry,
  get_message_typesupport_handle_function,
  &mesh_msgs__msg__MeshGeometry__get_type_hash,
  &mesh_msgs__msg__MeshGeometry__get_type_description,
  &mesh_msgs__msg__MeshGeometry__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mesh_msgs, msg, MeshGeometry)() {
  return &_MeshGeometry__type_support;
}

#if defined(__cplusplus)
}
#endif
