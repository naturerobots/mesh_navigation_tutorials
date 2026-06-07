// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mesh_msgs:msg/MeshVertexTexCoords.idl
// generated code does not contain a copyright notice

#include "mesh_msgs/msg/detail/mesh_vertex_tex_coords__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__msg__MeshVertexTexCoords__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x94, 0x6d, 0x38, 0x6a, 0x8d, 0xe8, 0x27, 0x1a,
      0xcc, 0x42, 0x1f, 0xa8, 0xd2, 0xd1, 0xf3, 0x12,
      0x7e, 0x2b, 0x41, 0x6c, 0x4a, 0x98, 0x81, 0x44,
      0xe2, 0x95, 0x15, 0xc4, 0xba, 0x77, 0xc8, 0x5b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char mesh_msgs__msg__MeshVertexTexCoords__TYPE_NAME[] = "mesh_msgs/msg/MeshVertexTexCoords";

// Define type names, field names, and default values
static char mesh_msgs__msg__MeshVertexTexCoords__FIELD_NAME__u[] = "u";
static char mesh_msgs__msg__MeshVertexTexCoords__FIELD_NAME__v[] = "v";

static rosidl_runtime_c__type_description__Field mesh_msgs__msg__MeshVertexTexCoords__FIELDS[] = {
  {
    {mesh_msgs__msg__MeshVertexTexCoords__FIELD_NAME__u, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshVertexTexCoords__FIELD_NAME__v, 1, 1},
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
mesh_msgs__msg__MeshVertexTexCoords__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__msg__MeshVertexTexCoords__TYPE_NAME, 33, 33},
      {mesh_msgs__msg__MeshVertexTexCoords__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Mesh Attribute Type\n"
  "float32 u\n"
  "float32 v";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__msg__MeshVertexTexCoords__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__msg__MeshVertexTexCoords__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 42, 42},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__msg__MeshVertexTexCoords__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__msg__MeshVertexTexCoords__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
