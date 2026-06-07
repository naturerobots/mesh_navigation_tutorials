// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from mesh_msgs:msg/MeshMaterials.idl
// generated code does not contain a copyright notice
#include "mesh_msgs/msg/detail/mesh_materials__rosidl_typesupport_fastrtps_cpp.hpp"
#include "mesh_msgs/msg/detail/mesh_materials__functions.h"
#include "mesh_msgs/msg/detail/mesh_materials__struct.hpp"

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
namespace mesh_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const mesh_msgs::msg::MeshFaceCluster &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  mesh_msgs::msg::MeshFaceCluster &);
size_t get_serialized_size(
  const mesh_msgs::msg::MeshFaceCluster &,
  size_t current_alignment);
size_t
max_serialized_size_MeshFaceCluster(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
bool cdr_serialize_key(
  const mesh_msgs::msg::MeshFaceCluster &,
  eprosima::fastcdr::Cdr &);
size_t get_serialized_size_key(
  const mesh_msgs::msg::MeshFaceCluster &,
  size_t current_alignment);
size_t
max_serialized_size_key_MeshFaceCluster(
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
bool cdr_serialize(
  const mesh_msgs::msg::MeshMaterial &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  mesh_msgs::msg::MeshMaterial &);
size_t get_serialized_size(
  const mesh_msgs::msg::MeshMaterial &,
  size_t current_alignment);
size_t
max_serialized_size_MeshMaterial(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
bool cdr_serialize_key(
  const mesh_msgs::msg::MeshMaterial &,
  eprosima::fastcdr::Cdr &);
size_t get_serialized_size_key(
  const mesh_msgs::msg::MeshMaterial &,
  size_t current_alignment);
size_t
max_serialized_size_key_MeshMaterial(
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
bool cdr_serialize(
  const mesh_msgs::msg::MeshVertexTexCoords &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  mesh_msgs::msg::MeshVertexTexCoords &);
size_t get_serialized_size(
  const mesh_msgs::msg::MeshVertexTexCoords &,
  size_t current_alignment);
size_t
max_serialized_size_MeshVertexTexCoords(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
bool cdr_serialize_key(
  const mesh_msgs::msg::MeshVertexTexCoords &,
  eprosima::fastcdr::Cdr &);
size_t get_serialized_size_key(
  const mesh_msgs::msg::MeshVertexTexCoords &,
  size_t current_alignment);
size_t
max_serialized_size_key_MeshVertexTexCoords(
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
  const mesh_msgs::msg::MeshMaterials & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: clusters
  {
    size_t size = ros_message.clusters.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      mesh_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.clusters[i],
        cdr);
    }
  }

  // Member: materials
  {
    size_t size = ros_message.materials.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      mesh_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.materials[i],
        cdr);
    }
  }

  // Member: cluster_materials
  {
    cdr << ros_message.cluster_materials;
  }

  // Member: vertex_tex_coords
  {
    size_t size = ros_message.vertex_tex_coords.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      mesh_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.vertex_tex_coords[i],
        cdr);
    }
  }

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mesh_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  mesh_msgs::msg::MeshMaterials & ros_message)
{
  // Member: clusters
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

    ros_message.clusters.resize(size);
    for (size_t i = 0; i < size; i++) {
      mesh_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.clusters[i]);
    }
  }

  // Member: materials
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

    ros_message.materials.resize(size);
    for (size_t i = 0; i < size; i++) {
      mesh_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.materials[i]);
    }
  }

  // Member: cluster_materials
  {
    cdr >> ros_message.cluster_materials;
  }

  // Member: vertex_tex_coords
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

    ros_message.vertex_tex_coords.resize(size);
    for (size_t i = 0; i < size; i++) {
      mesh_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.vertex_tex_coords[i]);
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mesh_msgs
get_serialized_size(
  const mesh_msgs::msg::MeshMaterials & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: clusters
  {
    size_t array_size = ros_message.clusters.size();
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        mesh_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.clusters[index], current_alignment);
    }
  }

  // Member: materials
  {
    size_t array_size = ros_message.materials.size();
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        mesh_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.materials[index], current_alignment);
    }
  }

  // Member: cluster_materials
  {
    size_t array_size = ros_message.cluster_materials.size();
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.cluster_materials[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: vertex_tex_coords
  {
    size_t array_size = ros_message.vertex_tex_coords.size();
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        mesh_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.vertex_tex_coords[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mesh_msgs
max_serialized_size_MeshMaterials(
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

  // Member: clusters
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
      size_t inner_size =
        mesh_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_MeshFaceCluster(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // Member: materials
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
      size_t inner_size =
        mesh_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_MeshMaterial(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // Member: cluster_materials
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: vertex_tex_coords
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
      size_t inner_size =
        mesh_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_MeshVertexTexCoords(
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
    using DataType = mesh_msgs::msg::MeshMaterials;
    is_plain =
      (
      offsetof(DataType, vertex_tex_coords) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mesh_msgs
cdr_serialize_key(
  const mesh_msgs::msg::MeshMaterials & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: clusters
  {
    size_t size = ros_message.clusters.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      mesh_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize_key(
        ros_message.clusters[i],
        cdr);
    }
  }

  // Member: materials
  {
    size_t size = ros_message.materials.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      mesh_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize_key(
        ros_message.materials[i],
        cdr);
    }
  }

  // Member: cluster_materials
  {
    cdr << ros_message.cluster_materials;
  }

  // Member: vertex_tex_coords
  {
    size_t size = ros_message.vertex_tex_coords.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      mesh_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize_key(
        ros_message.vertex_tex_coords[i],
        cdr);
    }
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mesh_msgs
get_serialized_size_key(
  const mesh_msgs::msg::MeshMaterials & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: clusters
  {
    size_t array_size = ros_message.clusters.size();
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        mesh_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size_key(
        ros_message.clusters[index], current_alignment);
    }
  }

  // Member: materials
  {
    size_t array_size = ros_message.materials.size();
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        mesh_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size_key(
        ros_message.materials[index], current_alignment);
    }
  }

  // Member: cluster_materials
  {
    size_t array_size = ros_message.cluster_materials.size();
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.cluster_materials[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: vertex_tex_coords
  {
    size_t array_size = ros_message.vertex_tex_coords.size();
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        mesh_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size_key(
        ros_message.vertex_tex_coords[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mesh_msgs
max_serialized_size_key_MeshMaterials(
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

  // Member: clusters
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
      size_t inner_size =
        mesh_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_key_MeshFaceCluster(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: materials
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
      size_t inner_size =
        mesh_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_key_MeshMaterial(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: cluster_materials
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: vertex_tex_coords
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
      size_t inner_size =
        mesh_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_key_MeshVertexTexCoords(
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
    using DataType = mesh_msgs::msg::MeshMaterials;
    is_plain =
      (
      offsetof(DataType, vertex_tex_coords) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}


static bool _MeshMaterials__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const mesh_msgs::msg::MeshMaterials *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _MeshMaterials__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<mesh_msgs::msg::MeshMaterials *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _MeshMaterials__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const mesh_msgs::msg::MeshMaterials *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _MeshMaterials__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_MeshMaterials(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _MeshMaterials__callbacks = {
  "mesh_msgs::msg",
  "MeshMaterials",
  _MeshMaterials__cdr_serialize,
  _MeshMaterials__cdr_deserialize,
  _MeshMaterials__get_serialized_size,
  _MeshMaterials__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _MeshMaterials__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_MeshMaterials__callbacks,
  get_message_typesupport_handle_function,
  &mesh_msgs__msg__MeshMaterials__get_type_hash,
  &mesh_msgs__msg__MeshMaterials__get_type_description,
  &mesh_msgs__msg__MeshMaterials__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace mesh_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_mesh_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<mesh_msgs::msg::MeshMaterials>()
{
  return &mesh_msgs::msg::typesupport_fastrtps_cpp::_MeshMaterials__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mesh_msgs, msg, MeshMaterials)() {
  return &mesh_msgs::msg::typesupport_fastrtps_cpp::_MeshMaterials__handle;
}

#ifdef __cplusplus
}
#endif
