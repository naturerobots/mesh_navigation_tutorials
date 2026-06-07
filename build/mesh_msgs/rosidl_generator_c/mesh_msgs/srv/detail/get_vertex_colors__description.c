// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mesh_msgs:srv/GetVertexColors.idl
// generated code does not contain a copyright notice

#include "mesh_msgs/srv/detail/get_vertex_colors__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetVertexColors__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7f, 0x78, 0x03, 0x25, 0x77, 0x8d, 0xc9, 0xc6,
      0x49, 0x95, 0x07, 0xac, 0xaa, 0x0b, 0x28, 0x61,
      0xc6, 0x6f, 0xb7, 0x2b, 0x14, 0x65, 0xf3, 0xd4,
      0xc2, 0xb7, 0xa8, 0xc7, 0xa6, 0x21, 0x1c, 0xd2,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetVertexColors_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x02, 0x41, 0x45, 0x45, 0x36, 0x2e, 0x7a, 0x63,
      0x7c, 0x30, 0x21, 0x50, 0xde, 0xe0, 0x58, 0x07,
      0xfc, 0x7a, 0x64, 0x3d, 0xe7, 0x5e, 0xae, 0xac,
      0xda, 0xf0, 0x76, 0xfa, 0x2b, 0x3d, 0xd8, 0xa6,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetVertexColors_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9a, 0x74, 0x23, 0x8f, 0xbf, 0xcd, 0xbc, 0x23,
      0x77, 0x3f, 0xb2, 0x3a, 0x88, 0x10, 0x25, 0xc3,
      0x44, 0x58, 0x5f, 0x6c, 0x81, 0x9e, 0x89, 0xbc,
      0x97, 0x24, 0x1a, 0x04, 0x21, 0x6f, 0xe3, 0x17,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetVertexColors_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x17, 0x81, 0x58, 0x63, 0x1d, 0x96, 0x33, 0x88,
      0x5a, 0x9a, 0xd5, 0x79, 0x0c, 0x85, 0x48, 0xb4,
      0x80, 0x14, 0x6c, 0x87, 0x59, 0x09, 0xa0, 0x29,
      0x03, 0x2e, 0x2e, 0x41, 0x02, 0x95, 0x2d, 0x10,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "std_msgs/msg/detail/color_rgba__functions.h"
#include "mesh_msgs/msg/detail/mesh_vertex_colors_stamped__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "mesh_msgs/msg/detail/mesh_vertex_colors__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t mesh_msgs__msg__MeshVertexColors__EXPECTED_HASH = {1, {
    0x17, 0x59, 0x24, 0xe9, 0x6b, 0x33, 0x83, 0xe9,
    0x84, 0x59, 0x42, 0x09, 0x64, 0x39, 0x5c, 0x81,
    0x88, 0xa4, 0x67, 0xd1, 0xd2, 0xc2, 0x27, 0x61,
    0xd2, 0x54, 0xf6, 0xa6, 0x12, 0x72, 0x63, 0xdd,
  }};
static const rosidl_type_hash_t mesh_msgs__msg__MeshVertexColorsStamped__EXPECTED_HASH = {1, {
    0x8f, 0x0e, 0x45, 0x9d, 0x21, 0x0e, 0xa2, 0x60,
    0x08, 0x46, 0x94, 0xed, 0x41, 0x6c, 0xb2, 0xf0,
    0x0b, 0x7c, 0x90, 0x3d, 0x4e, 0x06, 0xe1, 0x02,
    0x3c, 0xdb, 0xaf, 0xb1, 0xe5, 0x82, 0x15, 0x88,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
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

static char mesh_msgs__srv__GetVertexColors__TYPE_NAME[] = "mesh_msgs/srv/GetVertexColors";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char mesh_msgs__msg__MeshVertexColors__TYPE_NAME[] = "mesh_msgs/msg/MeshVertexColors";
static char mesh_msgs__msg__MeshVertexColorsStamped__TYPE_NAME[] = "mesh_msgs/msg/MeshVertexColorsStamped";
static char mesh_msgs__srv__GetVertexColors_Event__TYPE_NAME[] = "mesh_msgs/srv/GetVertexColors_Event";
static char mesh_msgs__srv__GetVertexColors_Request__TYPE_NAME[] = "mesh_msgs/srv/GetVertexColors_Request";
static char mesh_msgs__srv__GetVertexColors_Response__TYPE_NAME[] = "mesh_msgs/srv/GetVertexColors_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char std_msgs__msg__ColorRGBA__TYPE_NAME[] = "std_msgs/msg/ColorRGBA";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char mesh_msgs__srv__GetVertexColors__FIELD_NAME__request_message[] = "request_message";
static char mesh_msgs__srv__GetVertexColors__FIELD_NAME__response_message[] = "response_message";
static char mesh_msgs__srv__GetVertexColors__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field mesh_msgs__srv__GetVertexColors__FIELDS[] = {
  {
    {mesh_msgs__srv__GetVertexColors__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mesh_msgs__srv__GetVertexColors_Request__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetVertexColors__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mesh_msgs__srv__GetVertexColors_Response__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetVertexColors__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mesh_msgs__srv__GetVertexColors_Event__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mesh_msgs__srv__GetVertexColors__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshVertexColors__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshVertexColorsStamped__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetVertexColors_Event__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetVertexColors_Request__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetVertexColors_Response__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
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
mesh_msgs__srv__GetVertexColors__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__srv__GetVertexColors__TYPE_NAME, 29, 29},
      {mesh_msgs__srv__GetVertexColors__FIELDS, 3, 3},
    },
    {mesh_msgs__srv__GetVertexColors__REFERENCED_TYPE_DESCRIPTIONS, 9, 9},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshVertexColors__EXPECTED_HASH, mesh_msgs__msg__MeshVertexColors__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = mesh_msgs__msg__MeshVertexColors__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshVertexColorsStamped__EXPECTED_HASH, mesh_msgs__msg__MeshVertexColorsStamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = mesh_msgs__msg__MeshVertexColorsStamped__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = mesh_msgs__srv__GetVertexColors_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = mesh_msgs__srv__GetVertexColors_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[5].fields = mesh_msgs__srv__GetVertexColors_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__ColorRGBA__EXPECTED_HASH, std_msgs__msg__ColorRGBA__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = std_msgs__msg__ColorRGBA__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[8].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mesh_msgs__srv__GetVertexColors_Request__FIELD_NAME__uuid[] = "uuid";

static rosidl_runtime_c__type_description__Field mesh_msgs__srv__GetVertexColors_Request__FIELDS[] = {
  {
    {mesh_msgs__srv__GetVertexColors_Request__FIELD_NAME__uuid, 4, 4},
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
mesh_msgs__srv__GetVertexColors_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__srv__GetVertexColors_Request__TYPE_NAME, 37, 37},
      {mesh_msgs__srv__GetVertexColors_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mesh_msgs__srv__GetVertexColors_Response__FIELD_NAME__mesh_vertex_colors_stamped[] = "mesh_vertex_colors_stamped";

static rosidl_runtime_c__type_description__Field mesh_msgs__srv__GetVertexColors_Response__FIELDS[] = {
  {
    {mesh_msgs__srv__GetVertexColors_Response__FIELD_NAME__mesh_vertex_colors_stamped, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mesh_msgs__msg__MeshVertexColorsStamped__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mesh_msgs__srv__GetVertexColors_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshVertexColors__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshVertexColorsStamped__TYPE_NAME, 37, 37},
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
mesh_msgs__srv__GetVertexColors_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__srv__GetVertexColors_Response__TYPE_NAME, 38, 38},
      {mesh_msgs__srv__GetVertexColors_Response__FIELDS, 1, 1},
    },
    {mesh_msgs__srv__GetVertexColors_Response__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshVertexColors__EXPECTED_HASH, mesh_msgs__msg__MeshVertexColors__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = mesh_msgs__msg__MeshVertexColors__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshVertexColorsStamped__EXPECTED_HASH, mesh_msgs__msg__MeshVertexColorsStamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = mesh_msgs__msg__MeshVertexColorsStamped__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__ColorRGBA__EXPECTED_HASH, std_msgs__msg__ColorRGBA__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = std_msgs__msg__ColorRGBA__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mesh_msgs__srv__GetVertexColors_Event__FIELD_NAME__info[] = "info";
static char mesh_msgs__srv__GetVertexColors_Event__FIELD_NAME__request[] = "request";
static char mesh_msgs__srv__GetVertexColors_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field mesh_msgs__srv__GetVertexColors_Event__FIELDS[] = {
  {
    {mesh_msgs__srv__GetVertexColors_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetVertexColors_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {mesh_msgs__srv__GetVertexColors_Request__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetVertexColors_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {mesh_msgs__srv__GetVertexColors_Response__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mesh_msgs__srv__GetVertexColors_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshVertexColors__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshVertexColorsStamped__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetVertexColors_Request__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetVertexColors_Response__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
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
mesh_msgs__srv__GetVertexColors_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__srv__GetVertexColors_Event__TYPE_NAME, 35, 35},
      {mesh_msgs__srv__GetVertexColors_Event__FIELDS, 3, 3},
    },
    {mesh_msgs__srv__GetVertexColors_Event__REFERENCED_TYPE_DESCRIPTIONS, 8, 8},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshVertexColors__EXPECTED_HASH, mesh_msgs__msg__MeshVertexColors__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = mesh_msgs__msg__MeshVertexColors__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshVertexColorsStamped__EXPECTED_HASH, mesh_msgs__msg__MeshVertexColorsStamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = mesh_msgs__msg__MeshVertexColorsStamped__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = mesh_msgs__srv__GetVertexColors_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = mesh_msgs__srv__GetVertexColors_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__ColorRGBA__EXPECTED_HASH, std_msgs__msg__ColorRGBA__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = std_msgs__msg__ColorRGBA__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string uuid\n"
  "---\n"
  "mesh_msgs/MeshVertexColorsStamped mesh_vertex_colors_stamped";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetVertexColors__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__srv__GetVertexColors__TYPE_NAME, 29, 29},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 77, 77},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetVertexColors_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__srv__GetVertexColors_Request__TYPE_NAME, 37, 37},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetVertexColors_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__srv__GetVertexColors_Response__TYPE_NAME, 38, 38},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetVertexColors_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__srv__GetVertexColors_Event__TYPE_NAME, 35, 35},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetVertexColors__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[10];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 10, 10};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__srv__GetVertexColors__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *mesh_msgs__msg__MeshVertexColors__get_individual_type_description_source(NULL);
    sources[3] = *mesh_msgs__msg__MeshVertexColorsStamped__get_individual_type_description_source(NULL);
    sources[4] = *mesh_msgs__srv__GetVertexColors_Event__get_individual_type_description_source(NULL);
    sources[5] = *mesh_msgs__srv__GetVertexColors_Request__get_individual_type_description_source(NULL);
    sources[6] = *mesh_msgs__srv__GetVertexColors_Response__get_individual_type_description_source(NULL);
    sources[7] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[8] = *std_msgs__msg__ColorRGBA__get_individual_type_description_source(NULL);
    sources[9] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetVertexColors_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__srv__GetVertexColors_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetVertexColors_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__srv__GetVertexColors_Response__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *mesh_msgs__msg__MeshVertexColors__get_individual_type_description_source(NULL);
    sources[3] = *mesh_msgs__msg__MeshVertexColorsStamped__get_individual_type_description_source(NULL);
    sources[4] = *std_msgs__msg__ColorRGBA__get_individual_type_description_source(NULL);
    sources[5] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetVertexColors_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[9];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 9, 9};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__srv__GetVertexColors_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *mesh_msgs__msg__MeshVertexColors__get_individual_type_description_source(NULL);
    sources[3] = *mesh_msgs__msg__MeshVertexColorsStamped__get_individual_type_description_source(NULL);
    sources[4] = *mesh_msgs__srv__GetVertexColors_Request__get_individual_type_description_source(NULL);
    sources[5] = *mesh_msgs__srv__GetVertexColors_Response__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[7] = *std_msgs__msg__ColorRGBA__get_individual_type_description_source(NULL);
    sources[8] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
