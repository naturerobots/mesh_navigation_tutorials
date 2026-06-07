// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mesh_msgs:msg/MeshFaceCluster.idl
// generated code does not contain a copyright notice

#include "mesh_msgs/msg/detail/mesh_face_cluster__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__msg__MeshFaceCluster__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x92, 0x99, 0x60, 0xe1, 0x90, 0x62, 0x0d, 0x08,
      0x0c, 0x50, 0x43, 0xa8, 0xac, 0x49, 0xbd, 0xbc,
      0xb8, 0x8d, 0x4f, 0x4b, 0x1b, 0xa4, 0xf1, 0xd6,
      0x73, 0x3c, 0xef, 0xde, 0x7e, 0x34, 0x03, 0x98,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char mesh_msgs__msg__MeshFaceCluster__TYPE_NAME[] = "mesh_msgs/msg/MeshFaceCluster";

// Define type names, field names, and default values
static char mesh_msgs__msg__MeshFaceCluster__FIELD_NAME__face_indices[] = "face_indices";
static char mesh_msgs__msg__MeshFaceCluster__FIELD_NAME__label[] = "label";

static rosidl_runtime_c__type_description__Field mesh_msgs__msg__MeshFaceCluster__FIELDS[] = {
  {
    {mesh_msgs__msg__MeshFaceCluster__FIELD_NAME__face_indices, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshFaceCluster__FIELD_NAME__label, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__msg__MeshFaceCluster__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__msg__MeshFaceCluster__TYPE_NAME, 29, 29},
      {mesh_msgs__msg__MeshFaceCluster__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "#Cluster\n"
  "uint32[] face_indices\n"
  "#optional\n"
  "string label";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__msg__MeshFaceCluster__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__msg__MeshFaceCluster__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 54, 54},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__msg__MeshFaceCluster__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__msg__MeshFaceCluster__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
