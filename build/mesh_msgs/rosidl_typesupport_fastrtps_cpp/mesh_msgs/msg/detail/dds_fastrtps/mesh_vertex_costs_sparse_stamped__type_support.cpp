// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from mesh_msgs:msg/MeshVertexCostsSparseStamped.idl
// generated code does not contain a copyright notice
#include "mesh_msgs/msg/detail/mesh_vertex_costs_sparse_stamped__rosidl_typesupport_fastrtps_cpp.hpp"
#include "mesh_msgs/msg/detail/mesh_vertex_costs_sparse_stamped__functions.h"
#include "mesh_msgs/msg/detail/mesh_vertex_costs_sparse_stamped__struct.hpp"

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
bool cdr_serialize_key(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
size_t get_serialized_size_key(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_key_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs

namespace mesh_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const mesh_msgs::msg::MeshVertexCostsSparse &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  mesh_msgs::msg::MeshVertexCostsSparse &);
size_t get_serialized_size(
  const mesh_msgs::msg::MeshVertexCostsSparse &,
  size_t current_alignment);
size_t
max_serialized_size_MeshVertexCostsSparse(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
bool cdr_serialize_key(
  const mesh_msgs::msg::MeshVertexCostsSparse &,
  eprosima::fastcdr::Cdr &);
size_t get_serialized_size_key(
  const mesh_msgs::msg::MeshVertexCostsSparse &,
  size_t current_alignment);
size_t
max_serialized_size_key_MeshVertexCostsSparse(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace mesh_msgs


namespace mesh_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{


bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mesh_msgs
cdr_serialize(
  const mesh_msgs::msg::MeshVertexCostsSparseStamped & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);

  // Member: uuid
  cdr << ros_message.uuid;

  // Member: type
  cdr << ros_message.type;

  // Member: mesh_vertex_costs
  mesh_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.mesh_vertex_costs,
    cdr);

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mesh_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  mesh_msgs::msg::MeshVertexCostsSparseStamped & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: uuid
  cdr >> ros_message.uuid;

  // Member: type
  cdr >> ros_message.type;

  // Member: mesh_vertex_costs
  mesh_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.mesh_vertex_costs);

  return true;
}  // NOLINT(readability/fn_size)


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mesh_msgs
get_serialized_size(
  const mesh_msgs::msg::MeshVertexCostsSparseStamped & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header
  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);

  // Member: uuid
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.uuid.size() + 1);

  // Member: type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.type.size() + 1);

  // Member: mesh_vertex_costs
  current_alignment +=
    mesh_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.mesh_vertex_costs, current_alignment);

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mesh_msgs
max_serialized_size_MeshVertexCostsSparseStamped(
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

  // Member: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // Member: uuid
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // Member: type
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // Member: mesh_vertex_costs
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        mesh_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_MeshVertexCostsSparse(
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
    using DataType = mesh_msgs::msg::MeshVertexCostsSparseStamped;
    is_plain =
      (
      offsetof(DataType, mesh_vertex_costs) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mesh_msgs
cdr_serialize_key(
  const mesh_msgs::msg::MeshVertexCostsSparseStamped & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize_key(
    ros_message.header,
    cdr);

  // Member: uuid
  cdr << ros_message.uuid;

  // Member: type
  cdr << ros_message.type;

  // Member: mesh_vertex_costs
  mesh_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize_key(
    ros_message.mesh_vertex_costs,
    cdr);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mesh_msgs
get_serialized_size_key(
  const mesh_msgs::msg::MeshVertexCostsSparseStamped & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header
  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size_key(
    ros_message.header, current_alignment);

  // Member: uuid
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.uuid.size() + 1);

  // Member: type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.type.size() + 1);

  // Member: mesh_vertex_costs
  current_alignment +=
    mesh_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size_key(
    ros_message.mesh_vertex_costs, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mesh_msgs
max_serialized_size_key_MeshVertexCostsSparseStamped(
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

  // Member: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_key_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: uuid
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: type
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: mesh_vertex_costs
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        mesh_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_key_MeshVertexCostsSparse(
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
    using DataType = mesh_msgs::msg::MeshVertexCostsSparseStamped;
    is_plain =
      (
      offsetof(DataType, mesh_vertex_costs) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}


static bool _MeshVertexCostsSparseStamped__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const mesh_msgs::msg::MeshVertexCostsSparseStamped *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _MeshVertexCostsSparseStamped__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<mesh_msgs::msg::MeshVertexCostsSparseStamped *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _MeshVertexCostsSparseStamped__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const mesh_msgs::msg::MeshVertexCostsSparseStamped *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _MeshVertexCostsSparseStamped__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_MeshVertexCostsSparseStamped(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _MeshVertexCostsSparseStamped__callbacks = {
  "mesh_msgs::msg",
  "MeshVertexCostsSparseStamped",
  _MeshVertexCostsSparseStamped__cdr_serialize,
  _MeshVertexCostsSparseStamped__cdr_deserialize,
  _MeshVertexCostsSparseStamped__get_serialized_size,
  _MeshVertexCostsSparseStamped__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _MeshVertexCostsSparseStamped__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_MeshVertexCostsSparseStamped__callbacks,
  get_message_typesupport_handle_function,
  &mesh_msgs__msg__MeshVertexCostsSparseStamped__get_type_hash,
  &mesh_msgs__msg__MeshVertexCostsSparseStamped__get_type_description,
  &mesh_msgs__msg__MeshVertexCostsSparseStamped__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace mesh_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_mesh_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<mesh_msgs::msg::MeshVertexCostsSparseStamped>()
{
  return &mesh_msgs::msg::typesupport_fastrtps_cpp::_MeshVertexCostsSparseStamped__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mesh_msgs, msg, MeshVertexCostsSparseStamped)() {
  return &mesh_msgs::msg::typesupport_fastrtps_cpp::_MeshVertexCostsSparseStamped__handle;
}

#ifdef __cplusplus
}
#endif
