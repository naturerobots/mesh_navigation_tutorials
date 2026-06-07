// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mesh_msgs:msg/MeshMaterialsStamped.idl
// generated code does not contain a copyright notice

#include "mesh_msgs/msg/detail/mesh_materials_stamped__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__msg__MeshMaterialsStamped__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc9, 0x91, 0xc0, 0xf3, 0x6c, 0x86, 0x36, 0xf2,
      0x29, 0xb4, 0x62, 0x0d, 0x34, 0x1c, 0x5a, 0x0a,
      0x90, 0x2e, 0x4b, 0xd5, 0x00, 0x91, 0x8d, 0xc0,
      0xa5, 0xcb, 0xd5, 0x44, 0xbc, 0xa9, 0x08, 0xf3,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "mesh_msgs/msg/detail/mesh_materials__functions.h"
#include "mesh_msgs/msg/detail/mesh_material__functions.h"
#include "mesh_msgs/msg/detail/mesh_face_cluster__functions.h"
#include "std_msgs/msg/detail/color_rgba__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "mesh_msgs/msg/detail/mesh_vertex_tex_coords__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
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
static const rosidl_type_hash_t mesh_msgs__msg__MeshMaterials__EXPECTED_HASH = {1, {
    0xb3, 0x18, 0x2e, 0x23, 0xfc, 0x9d, 0xeb, 0xe1,
    0x97, 0x24, 0x61, 0xde, 0x27, 0xfd, 0x1d, 0x7e,
    0xd7, 0x45, 0x50, 0x8f, 0x4d, 0x59, 0x91, 0x92,
    0x1b, 0x16, 0x7f, 0x65, 0x5f, 0x38, 0x27, 0x8b,
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
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char mesh_msgs__msg__MeshMaterialsStamped__TYPE_NAME[] = "mesh_msgs/msg/MeshMaterialsStamped";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char mesh_msgs__msg__MeshFaceCluster__TYPE_NAME[] = "mesh_msgs/msg/MeshFaceCluster";
static char mesh_msgs__msg__MeshMaterial__TYPE_NAME[] = "mesh_msgs/msg/MeshMaterial";
static char mesh_msgs__msg__MeshMaterials__TYPE_NAME[] = "mesh_msgs/msg/MeshMaterials";
static char mesh_msgs__msg__MeshVertexTexCoords__TYPE_NAME[] = "mesh_msgs/msg/MeshVertexTexCoords";
static char std_msgs__msg__ColorRGBA__TYPE_NAME[] = "std_msgs/msg/ColorRGBA";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char mesh_msgs__msg__MeshMaterialsStamped__FIELD_NAME__header[] = "header";
static char mesh_msgs__msg__MeshMaterialsStamped__FIELD_NAME__uuid[] = "uuid";
static char mesh_msgs__msg__MeshMaterialsStamped__FIELD_NAME__mesh_materials[] = "mesh_materials";

static rosidl_runtime_c__type_description__Field mesh_msgs__msg__MeshMaterialsStamped__FIELDS[] = {
  {
    {mesh_msgs__msg__MeshMaterialsStamped__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshMaterialsStamped__FIELD_NAME__uuid, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshMaterialsStamped__FIELD_NAME__mesh_materials, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mesh_msgs__msg__MeshMaterials__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mesh_msgs__msg__MeshMaterialsStamped__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshFaceCluster__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshMaterial__TYPE_NAME, 26, 26},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshMaterials__TYPE_NAME, 27, 27},
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
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__msg__MeshMaterialsStamped__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__msg__MeshMaterialsStamped__TYPE_NAME, 34, 34},
      {mesh_msgs__msg__MeshMaterialsStamped__FIELDS, 3, 3},
    },
    {mesh_msgs__msg__MeshMaterialsStamped__REFERENCED_TYPE_DESCRIPTIONS, 7, 7},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshFaceCluster__EXPECTED_HASH, mesh_msgs__msg__MeshFaceCluster__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = mesh_msgs__msg__MeshFaceCluster__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshMaterial__EXPECTED_HASH, mesh_msgs__msg__MeshMaterial__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = mesh_msgs__msg__MeshMaterial__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshMaterials__EXPECTED_HASH, mesh_msgs__msg__MeshMaterials__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = mesh_msgs__msg__MeshMaterials__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshVertexTexCoords__EXPECTED_HASH, mesh_msgs__msg__MeshVertexTexCoords__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = mesh_msgs__msg__MeshVertexTexCoords__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__ColorRGBA__EXPECTED_HASH, std_msgs__msg__ColorRGBA__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = std_msgs__msg__ColorRGBA__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Mesh Attribute Message\n"
  "std_msgs/Header header\n"
  "string uuid\n"
  "mesh_msgs/MeshMaterials mesh_materials";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__msg__MeshMaterialsStamped__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__msg__MeshMaterialsStamped__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 99, 99},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__msg__MeshMaterialsStamped__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[8];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 8, 8};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__msg__MeshMaterialsStamped__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *mesh_msgs__msg__MeshFaceCluster__get_individual_type_description_source(NULL);
    sources[3] = *mesh_msgs__msg__MeshMaterial__get_individual_type_description_source(NULL);
    sources[4] = *mesh_msgs__msg__MeshMaterials__get_individual_type_description_source(NULL);
    sources[5] = *mesh_msgs__msg__MeshVertexTexCoords__get_individual_type_description_source(NULL);
    sources[6] = *std_msgs__msg__ColorRGBA__get_individual_type_description_source(NULL);
    sources[7] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
