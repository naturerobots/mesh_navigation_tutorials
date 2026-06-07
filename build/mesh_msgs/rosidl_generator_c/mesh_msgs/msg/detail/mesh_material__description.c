// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mesh_msgs:msg/MeshMaterial.idl
// generated code does not contain a copyright notice

#include "mesh_msgs/msg/detail/mesh_material__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__msg__MeshMaterial__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x11, 0x20, 0xc8, 0xf2, 0xa4, 0x45, 0x2c, 0x1c,
      0xa4, 0x01, 0x8f, 0x2c, 0x6b, 0x85, 0x98, 0x51,
      0xd1, 0x88, 0xdf, 0xcb, 0x29, 0x0e, 0xc2, 0x66,
      0x0d, 0x3b, 0xe7, 0xd2, 0x08, 0x3b, 0x21, 0xd1,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "std_msgs/msg/detail/color_rgba__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t std_msgs__msg__ColorRGBA__EXPECTED_HASH = {1, {
    0x77, 0xa7, 0xa5, 0xb9, 0xae, 0x47, 0x73, 0x06,
    0x09, 0x76, 0x65, 0x10, 0x6e, 0x04, 0x13, 0xba,
    0x74, 0x44, 0x02, 0x45, 0xb1, 0xf3, 0xd0, 0xc6,
    0xd6, 0x40, 0x5f, 0xe5, 0xc7, 0x81, 0x3f, 0xe8,
  }};
#endif

static char mesh_msgs__msg__MeshMaterial__TYPE_NAME[] = "mesh_msgs/msg/MeshMaterial";
static char std_msgs__msg__ColorRGBA__TYPE_NAME[] = "std_msgs/msg/ColorRGBA";

// Define type names, field names, and default values
static char mesh_msgs__msg__MeshMaterial__FIELD_NAME__texture_index[] = "texture_index";
static char mesh_msgs__msg__MeshMaterial__FIELD_NAME__color[] = "color";
static char mesh_msgs__msg__MeshMaterial__FIELD_NAME__has_texture[] = "has_texture";

static rosidl_runtime_c__type_description__Field mesh_msgs__msg__MeshMaterial__FIELDS[] = {
  {
    {mesh_msgs__msg__MeshMaterial__FIELD_NAME__texture_index, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshMaterial__FIELD_NAME__color, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__ColorRGBA__TYPE_NAME, 22, 22},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshMaterial__FIELD_NAME__has_texture, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mesh_msgs__msg__MeshMaterial__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {std_msgs__msg__ColorRGBA__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__msg__MeshMaterial__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__msg__MeshMaterial__TYPE_NAME, 26, 26},
      {mesh_msgs__msg__MeshMaterial__FIELDS, 3, 3},
    },
    {mesh_msgs__msg__MeshMaterial__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&std_msgs__msg__ColorRGBA__EXPECTED_HASH, std_msgs__msg__ColorRGBA__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = std_msgs__msg__ColorRGBA__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint32 texture_index\n"
  "std_msgs/ColorRGBA color\n"
  "bool has_texture";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__msg__MeshMaterial__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__msg__MeshMaterial__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 63, 63},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__msg__MeshMaterial__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__msg__MeshMaterial__get_individual_type_description_source(NULL),
    sources[1] = *std_msgs__msg__ColorRGBA__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
