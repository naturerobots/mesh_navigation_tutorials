// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mesh_msgs:msg/MeshVertexCostsSparse.idl
// generated code does not contain a copyright notice

#include "mesh_msgs/msg/detail/mesh_vertex_costs_sparse__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__msg__MeshVertexCostsSparse__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd2, 0xad, 0x90, 0x34, 0x29, 0x8a, 0x21, 0xea,
      0x10, 0x08, 0xc5, 0x3d, 0x96, 0xda, 0xac, 0xdc,
      0xa6, 0xdb, 0x11, 0xb7, 0xd2, 0xfc, 0x91, 0x56,
      0x6d, 0x41, 0x34, 0xfa, 0x2a, 0x8d, 0xcb, 0xab,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char mesh_msgs__msg__MeshVertexCostsSparse__TYPE_NAME[] = "mesh_msgs/msg/MeshVertexCostsSparse";

// Define type names, field names, and default values
static char mesh_msgs__msg__MeshVertexCostsSparse__FIELD_NAME__vertices[] = "vertices";
static char mesh_msgs__msg__MeshVertexCostsSparse__FIELD_NAME__costs[] = "costs";
static char mesh_msgs__msg__MeshVertexCostsSparse__FIELD_NAME__default_value[] = "default_value";

static rosidl_runtime_c__type_description__Field mesh_msgs__msg__MeshVertexCostsSparse__FIELDS[] = {
  {
    {mesh_msgs__msg__MeshVertexCostsSparse__FIELD_NAME__vertices, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshVertexCostsSparse__FIELD_NAME__costs, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshVertexCostsSparse__FIELD_NAME__default_value, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__msg__MeshVertexCostsSparse__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__msg__MeshVertexCostsSparse__TYPE_NAME, 35, 35},
      {mesh_msgs__msg__MeshVertexCostsSparse__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# MeshVertexCostsSparse\n"
  "# Holds costs for a subset of the mesh vertices\n"
  "\n"
  "# The vertex indices\n"
  "uint32[] vertices\n"
  "\n"
  "# The cost per vertex\n"
  "float32[] costs\n"
  "\n"
  "# The default value for all other vertices\n"
  "float32 default_value";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__msg__MeshVertexCostsSparse__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__msg__MeshVertexCostsSparse__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 217, 217},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__msg__MeshVertexCostsSparse__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__msg__MeshVertexCostsSparse__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
