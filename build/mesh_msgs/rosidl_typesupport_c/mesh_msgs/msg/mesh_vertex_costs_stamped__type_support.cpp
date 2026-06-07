// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from mesh_msgs:msg/MeshVertexCostsStamped.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "mesh_msgs/msg/detail/mesh_vertex_costs_stamped__struct.h"
#include "mesh_msgs/msg/detail/mesh_vertex_costs_stamped__type_support.h"
#include "mesh_msgs/msg/detail/mesh_vertex_costs_stamped__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace mesh_msgs
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _MeshVertexCostsStamped_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MeshVertexCostsStamped_type_support_ids_t;

static const _MeshVertexCostsStamped_type_support_ids_t _MeshVertexCostsStamped_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MeshVertexCostsStamped_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MeshVertexCostsStamped_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MeshVertexCostsStamped_type_support_symbol_names_t _MeshVertexCostsStamped_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mesh_msgs, msg, MeshVertexCostsStamped)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, msg, MeshVertexCostsStamped)),
  }
};

typedef struct _MeshVertexCostsStamped_type_support_data_t
{
  void * data[2];
} _MeshVertexCostsStamped_type_support_data_t;

static _MeshVertexCostsStamped_type_support_data_t _MeshVertexCostsStamped_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MeshVertexCostsStamped_message_typesupport_map = {
  2,
  "mesh_msgs",
  &_MeshVertexCostsStamped_message_typesupport_ids.typesupport_identifier[0],
  &_MeshVertexCostsStamped_message_typesupport_symbol_names.symbol_name[0],
  &_MeshVertexCostsStamped_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MeshVertexCostsStamped_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MeshVertexCostsStamped_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &mesh_msgs__msg__MeshVertexCostsStamped__get_type_hash,
  &mesh_msgs__msg__MeshVertexCostsStamped__get_type_description,
  &mesh_msgs__msg__MeshVertexCostsStamped__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace mesh_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, mesh_msgs, msg, MeshVertexCostsStamped)() {
  return &::mesh_msgs::msg::rosidl_typesupport_c::MeshVertexCostsStamped_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
