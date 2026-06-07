// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mesh_msgs:msg/MeshVertexCosts.idl
// generated code does not contain a copyright notice

#include "mesh_msgs/msg/detail/mesh_vertex_costs__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__msg__MeshVertexCosts__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9b, 0xa7, 0xc4, 0x5f, 0x14, 0x4a, 0x59, 0xc4,
      0x3c, 0x66, 0x86, 0x97, 0x3a, 0x9c, 0x44, 0x8d,
      0x56, 0x40, 0xdb, 0xd3, 0x7c, 0xdf, 0xee, 0xbe,
      0x8b, 0x61, 0x54, 0xde, 0xf3, 0xef, 0x5a, 0x31,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char mesh_msgs__msg__MeshVertexCosts__TYPE_NAME[] = "mesh_msgs/msg/MeshVertexCosts";

// Define type names, field names, and default values
static char mesh_msgs__msg__MeshVertexCosts__FIELD_NAME__costs[] = "costs";

static rosidl_runtime_c__type_description__Field mesh_msgs__msg__MeshVertexCosts__FIELDS[] = {
  {
    {mesh_msgs__msg__MeshVertexCosts__FIELD_NAME__costs, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__msg__MeshVertexCosts__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__msg__MeshVertexCosts__TYPE_NAME, 29, 29},
      {mesh_msgs__msg__MeshVertexCosts__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Mesh Attribute Message\n"
  "float32[] costs";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__msg__MeshVertexCosts__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__msg__MeshVertexCosts__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 41, 41},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__msg__MeshVertexCosts__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__msg__MeshVertexCosts__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
