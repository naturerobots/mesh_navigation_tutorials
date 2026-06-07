// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mesh_msgs:msg/MeshMaterials.idl
// generated code does not contain a copyright notice

#include "mesh_msgs/msg/detail/mesh_materials__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__msg__MeshMaterials__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb3, 0x18, 0x2e, 0x23, 0xfc, 0x9d, 0xeb, 0xe1,
      0x97, 0x24, 0x61, 0xde, 0x27, 0xfd, 0x1d, 0x7e,
      0xd7, 0x45, 0x50, 0x8f, 0x4d, 0x59, 0x91, 0x92,
      0x1b, 0x16, 0x7f, 0x65, 0x5f, 0x38, 0x27, 0x8b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "mesh_msgs/msg/detail/mesh_face_cluster__functions.h"
#include "mesh_msgs/msg/detail/mesh_material__functions.h"
#include "std_msgs/msg/detail/color_rgba__functions.h"
#include "mesh_msgs/msg/detail/mesh_vertex_tex_coords__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t mesh_msgs__msg__MeshFaceCluster__EXPECTED_HASH = {1, {
    0x92, 0x99, 0x60, 0xe1, 0x90, 0x62, 0x0d, 0x08,
    0x0c, 0x50, 0x43, 0xa8, 0xac, 0x49, 0xbd, 0xbc,
    0xb8, 0x8d, 0x4f, 0x4b, 0x1b, 0xa4, 0xf1, 0xd6,
    0x73, 0x3c, 0xef, 0xde, 0x7e, 0x34, 0x03, 0x98,
  }};
static const rosidl_type_hash_t mesh_msgs__msg__MeshMaterial__EXPECTED_HASH = {1, {
    0x11, 0x20, 0xc8, 0xf2, 0xa4, 0x45, 0x2c, 0x1c,
    0xa4, 0x01, 0x8f, 0x2c, 0x6b, 0x85, 0x98, 0x51,
    0xd1, 0x88, 0xdf, 0xcb, 0x29, 0x0e, 0xc2, 0x66,
    0x0d, 0x3b, 0xe7, 0xd2, 0x08, 0x3b, 0x21, 0xd1,
  }};
static const rosidl_type_hash_t mesh_msgs__msg__MeshVertexTexCoords__EXPECTED_HASH = {1, {
    0x94, 0x6d, 0x38, 0x6a, 0x8d, 0xe8, 0x27, 0x1a,
    0xcc, 0x42, 0x1f, 0xa8, 0xd2, 0xd1, 0xf3, 0x12,
    0x7e, 0x2b, 0x41, 0x6c, 0x4a, 0x98, 0x81, 0x44,
    0xe2, 0x95, 0x15, 0xc4, 0xba, 0x77, 0xc8, 0x5b,
  }};
static const rosidl_type_hash_t std_msgs__msg__ColorRGBA__EXPECTED_HASH = {1, {
    0x77, 0xa7, 0xa5, 0xb9, 0xae, 0x47, 0x73, 0x06,
    0x09, 0x76, 0x65, 0x10, 0x6e, 0x04, 0x13, 0xba,
    0x74, 0x44, 0x02, 0x45, 0xb1, 0xf3, 0xd0, 0xc6,
    0xd6, 0x40, 0x5f, 0xe5, 0xc7, 0x81, 0x3f, 0xe8,
  }};
#endif

static char mesh_msgs__msg__MeshMaterials__TYPE_NAME[] = "mesh_msgs/msg/MeshMaterials";
static char mesh_msgs__msg__MeshFaceCluster__TYPE_NAME[] = "mesh_msgs/msg/MeshFaceCluster";
static char mesh_msgs__msg__MeshMaterial__TYPE_NAME[] = "mesh_msgs/msg/MeshMaterial";
static char mesh_msgs__msg__MeshVertexTexCoords__TYPE_NAME[] = "mesh_msgs/msg/MeshVertexTexCoords";
static char std_msgs__msg__ColorRGBA__TYPE_NAME[] = "std_msgs/msg/ColorRGBA";

// Define type names, field names, and default values
static char mesh_msgs__msg__MeshMaterials__FIELD_NAME__clusters[] = "clusters";
static char mesh_msgs__msg__MeshMaterials__FIELD_NAME__materials[] = "materials";
static char mesh_msgs__msg__MeshMaterials__FIELD_NAME__cluster_materials[] = "cluster_materials";
static char mesh_msgs__msg__MeshMaterials__FIELD_NAME__vertex_tex_coords[] = "vertex_tex_coords";

static rosidl_runtime_c__type_description__Field mesh_msgs__msg__MeshMaterials__FIELDS[] = {
  {
    {mesh_msgs__msg__MeshMaterials__FIELD_NAME__clusters, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {mesh_msgs__msg__MeshFaceCluster__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshMaterials__FIELD_NAME__materials, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {mesh_msgs__msg__MeshMaterial__TYPE_NAME, 26, 26},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshMaterials__FIELD_NAME__cluster_materials, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshMaterials__FIELD_NAME__vertex_tex_coords, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {mesh_msgs__msg__MeshVertexTexCoords__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mesh_msgs__msg__MeshMaterials__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {mesh_msgs__msg__MeshFaceCluster__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshMaterial__TYPE_NAME, 26, 26},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshVertexTexCoords__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__ColorRGBA__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__msg__MeshMaterials__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__msg__MeshMaterials__TYPE_NAME, 27, 27},
      {mesh_msgs__msg__MeshMaterials__FIELDS, 4, 4},
    },
    {mesh_msgs__msg__MeshMaterials__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&mesh_msgs__msg__MeshFaceCluster__EXPECTED_HASH, mesh_msgs__msg__MeshFaceCluster__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = mesh_msgs__msg__MeshFaceCluster__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshMaterial__EXPECTED_HASH, mesh_msgs__msg__MeshMaterial__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = mesh_msgs__msg__MeshMaterial__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshVertexTexCoords__EXPECTED_HASH, mesh_msgs__msg__MeshVertexTexCoords__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = mesh_msgs__msg__MeshVertexTexCoords__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__ColorRGBA__EXPECTED_HASH, std_msgs__msg__ColorRGBA__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = std_msgs__msg__ColorRGBA__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Mesh Attribute Message\n"
  "mesh_msgs/MeshFaceCluster[] clusters\n"
  "mesh_msgs/MeshMaterial[] materials\n"
  "uint32[] cluster_materials\n"
  "mesh_msgs/MeshVertexTexCoords[] vertex_tex_coords";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__msg__MeshMaterials__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__msg__MeshMaterials__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 174, 174},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__msg__MeshMaterials__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__msg__MeshMaterials__get_individual_type_description_source(NULL),
    sources[1] = *mesh_msgs__msg__MeshFaceCluster__get_individual_type_description_source(NULL);
    sources[2] = *mesh_msgs__msg__MeshMaterial__get_individual_type_description_source(NULL);
    sources[3] = *mesh_msgs__msg__MeshVertexTexCoords__get_individual_type_description_source(NULL);
    sources[4] = *std_msgs__msg__ColorRGBA__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
