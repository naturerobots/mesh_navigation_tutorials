// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mesh_msgs:msg/MeshVertexCostsStamped.idl
// generated code does not contain a copyright notice

#include "mesh_msgs/msg/detail/mesh_vertex_costs_stamped__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__msg__MeshVertexCostsStamped__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8d, 0x8f, 0x68, 0xbc, 0x4e, 0x18, 0xb8, 0x29,
      0xa8, 0x2b, 0x45, 0xfb, 0xc6, 0x98, 0x7a, 0x35,
      0x92, 0x4f, 0x8d, 0x7a, 0x58, 0x11, 0x97, 0x7b,
      0x10, 0x53, 0x80, 0xef, 0x22, 0x13, 0x88, 0xc4,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "mesh_msgs/msg/detail/mesh_vertex_costs__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t mesh_msgs__msg__MeshVertexCosts__EXPECTED_HASH = {1, {
    0x9b, 0xa7, 0xc4, 0x5f, 0x14, 0x4a, 0x59, 0xc4,
    0x3c, 0x66, 0x86, 0x97, 0x3a, 0x9c, 0x44, 0x8d,
    0x56, 0x40, 0xdb, 0xd3, 0x7c, 0xdf, 0xee, 0xbe,
    0x8b, 0x61, 0x54, 0xde, 0xf3, 0xef, 0x5a, 0x31,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char mesh_msgs__msg__MeshVertexCostsStamped__TYPE_NAME[] = "mesh_msgs/msg/MeshVertexCostsStamped";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char mesh_msgs__msg__MeshVertexCosts__TYPE_NAME[] = "mesh_msgs/msg/MeshVertexCosts";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char mesh_msgs__msg__MeshVertexCostsStamped__FIELD_NAME__header[] = "header";
static char mesh_msgs__msg__MeshVertexCostsStamped__FIELD_NAME__uuid[] = "uuid";
static char mesh_msgs__msg__MeshVertexCostsStamped__FIELD_NAME__type[] = "type";
static char mesh_msgs__msg__MeshVertexCostsStamped__FIELD_NAME__mesh_vertex_costs[] = "mesh_vertex_costs";

static rosidl_runtime_c__type_description__Field mesh_msgs__msg__MeshVertexCostsStamped__FIELDS[] = {
  {
    {mesh_msgs__msg__MeshVertexCostsStamped__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshVertexCostsStamped__FIELD_NAME__uuid, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshVertexCostsStamped__FIELD_NAME__type, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshVertexCostsStamped__FIELD_NAME__mesh_vertex_costs, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mesh_msgs__msg__MeshVertexCosts__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mesh_msgs__msg__MeshVertexCostsStamped__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshVertexCosts__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__msg__MeshVertexCostsStamped__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__msg__MeshVertexCostsStamped__TYPE_NAME, 36, 36},
      {mesh_msgs__msg__MeshVertexCostsStamped__FIELDS, 4, 4},
    },
    {mesh_msgs__msg__MeshVertexCostsStamped__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshVertexCosts__EXPECTED_HASH, mesh_msgs__msg__MeshVertexCosts__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = mesh_msgs__msg__MeshVertexCosts__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Mesh Attribute Message\n"
  "std_msgs/Header header\n"
  "string uuid\n"
  "string type\n"
  "mesh_msgs/MeshVertexCosts mesh_vertex_costs";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__msg__MeshVertexCostsStamped__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__msg__MeshVertexCostsStamped__TYPE_NAME, 36, 36},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 116, 116},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__msg__MeshVertexCostsStamped__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__msg__MeshVertexCostsStamped__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *mesh_msgs__msg__MeshVertexCosts__get_individual_type_description_source(NULL);
    sources[3] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
