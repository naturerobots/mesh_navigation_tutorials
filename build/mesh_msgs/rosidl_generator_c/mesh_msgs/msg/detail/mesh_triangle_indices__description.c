// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mesh_msgs:msg/MeshTriangleIndices.idl
// generated code does not contain a copyright notice

#include "mesh_msgs/msg/detail/mesh_triangle_indices__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__msg__MeshTriangleIndices__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0d, 0xae, 0xe6, 0xbf, 0x6b, 0x63, 0x9f, 0xc7,
      0x4e, 0xb9, 0x2d, 0x51, 0xfb, 0x33, 0xb5, 0x3c,
      0x4f, 0xde, 0xb1, 0xcc, 0x1c, 0x1f, 0x2a, 0x3a,
      0x8a, 0x14, 0x0b, 0x4a, 0x89, 0xa3, 0x71, 0xa1,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char mesh_msgs__msg__MeshTriangleIndices__TYPE_NAME[] = "mesh_msgs/msg/MeshTriangleIndices";

// Define type names, field names, and default values
static char mesh_msgs__msg__MeshTriangleIndices__FIELD_NAME__vertex_indices[] = "vertex_indices";

static rosidl_runtime_c__type_description__Field mesh_msgs__msg__MeshTriangleIndices__FIELDS[] = {
  {
    {mesh_msgs__msg__MeshTriangleIndices__FIELD_NAME__vertex_indices, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__msg__MeshTriangleIndices__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__msg__MeshTriangleIndices__TYPE_NAME, 33, 33},
      {mesh_msgs__msg__MeshTriangleIndices__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Definition of a triangle's vertices\n"
  "uint32[3] vertex_indices";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__msg__MeshTriangleIndices__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__msg__MeshTriangleIndices__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 63, 63},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__msg__MeshTriangleIndices__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__msg__MeshTriangleIndices__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
