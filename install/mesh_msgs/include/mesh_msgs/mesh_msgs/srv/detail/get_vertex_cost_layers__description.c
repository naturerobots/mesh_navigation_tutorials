// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mesh_msgs:srv/GetVertexCostLayers.idl
// generated code does not contain a copyright notice

#include "mesh_msgs/srv/detail/get_vertex_cost_layers__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetVertexCostLayers__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd2, 0x3d, 0x9b, 0x0e, 0xff, 0xa7, 0x7f, 0x94,
      0x64, 0x7c, 0x61, 0x3a, 0x3b, 0x55, 0x3d, 0x02,
      0xa9, 0x4a, 0x4f, 0xb3, 0x31, 0x0d, 0xea, 0x6e,
      0x64, 0x54, 0xc0, 0x17, 0xdb, 0x58, 0x2f, 0x89,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetVertexCostLayers_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc8, 0x11, 0xd3, 0xf2, 0xb1, 0xd4, 0xe3, 0x2e,
      0x1c, 0xaf, 0xf3, 0x93, 0x56, 0x6f, 0x37, 0x94,
      0x4e, 0xc4, 0xb4, 0x20, 0x3e, 0x7d, 0xae, 0x4a,
      0xa6, 0x85, 0x32, 0x10, 0x93, 0xe4, 0xdf, 0x8e,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetVertexCostLayers_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x85, 0xac, 0x53, 0x07, 0x16, 0x93, 0x7b, 0x0b,
      0xfe, 0x5e, 0x5a, 0xea, 0x71, 0x24, 0x82, 0xfd,
      0xb5, 0xef, 0xb4, 0xd7, 0x14, 0x1d, 0xb6, 0x7c,
      0xff, 0x22, 0x63, 0xf1, 0xbd, 0xe2, 0x87, 0x0f,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetVertexCostLayers_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xfe, 0x31, 0x83, 0xce, 0x13, 0xaf, 0xf6, 0x7d,
      0xcb, 0x7a, 0x53, 0x7a, 0xfc, 0x17, 0x1a, 0xa9,
      0x79, 0x37, 0x18, 0xc1, 0xed, 0x7d, 0xbf, 0xfb,
      0xf8, 0xf9, 0x11, 0xe2, 0x20, 0xec, 0x1f, 0xdd,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char mesh_msgs__srv__GetVertexCostLayers__TYPE_NAME[] = "mesh_msgs/srv/GetVertexCostLayers";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char mesh_msgs__srv__GetVertexCostLayers_Event__TYPE_NAME[] = "mesh_msgs/srv/GetVertexCostLayers_Event";
static char mesh_msgs__srv__GetVertexCostLayers_Request__TYPE_NAME[] = "mesh_msgs/srv/GetVertexCostLayers_Request";
static char mesh_msgs__srv__GetVertexCostLayers_Response__TYPE_NAME[] = "mesh_msgs/srv/GetVertexCostLayers_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char mesh_msgs__srv__GetVertexCostLayers__FIELD_NAME__request_message[] = "request_message";
static char mesh_msgs__srv__GetVertexCostLayers__FIELD_NAME__response_message[] = "response_message";
static char mesh_msgs__srv__GetVertexCostLayers__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field mesh_msgs__srv__GetVertexCostLayers__FIELDS[] = {
  {
    {mesh_msgs__srv__GetVertexCostLayers__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mesh_msgs__srv__GetVertexCostLayers_Request__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetVertexCostLayers__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mesh_msgs__srv__GetVertexCostLayers_Response__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetVertexCostLayers__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mesh_msgs__srv__GetVertexCostLayers_Event__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mesh_msgs__srv__GetVertexCostLayers__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetVertexCostLayers_Event__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetVertexCostLayers_Request__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetVertexCostLayers_Response__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__srv__GetVertexCostLayers__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__srv__GetVertexCostLayers__TYPE_NAME, 33, 33},
      {mesh_msgs__srv__GetVertexCostLayers__FIELDS, 3, 3},
    },
    {mesh_msgs__srv__GetVertexCostLayers__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = mesh_msgs__srv__GetVertexCostLayers_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = mesh_msgs__srv__GetVertexCostLayers_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = mesh_msgs__srv__GetVertexCostLayers_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mesh_msgs__srv__GetVertexCostLayers_Request__FIELD_NAME__uuid[] = "uuid";

static rosidl_runtime_c__type_description__Field mesh_msgs__srv__GetVertexCostLayers_Request__FIELDS[] = {
  {
    {mesh_msgs__srv__GetVertexCostLayers_Request__FIELD_NAME__uuid, 4, 4},
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
mesh_msgs__srv__GetVertexCostLayers_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__srv__GetVertexCostLayers_Request__TYPE_NAME, 41, 41},
      {mesh_msgs__srv__GetVertexCostLayers_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mesh_msgs__srv__GetVertexCostLayers_Response__FIELD_NAME__layers[] = "layers";

static rosidl_runtime_c__type_description__Field mesh_msgs__srv__GetVertexCostLayers_Response__FIELDS[] = {
  {
    {mesh_msgs__srv__GetVertexCostLayers_Response__FIELD_NAME__layers, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__srv__GetVertexCostLayers_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__srv__GetVertexCostLayers_Response__TYPE_NAME, 42, 42},
      {mesh_msgs__srv__GetVertexCostLayers_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mesh_msgs__srv__GetVertexCostLayers_Event__FIELD_NAME__info[] = "info";
static char mesh_msgs__srv__GetVertexCostLayers_Event__FIELD_NAME__request[] = "request";
static char mesh_msgs__srv__GetVertexCostLayers_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field mesh_msgs__srv__GetVertexCostLayers_Event__FIELDS[] = {
  {
    {mesh_msgs__srv__GetVertexCostLayers_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetVertexCostLayers_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {mesh_msgs__srv__GetVertexCostLayers_Request__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetVertexCostLayers_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {mesh_msgs__srv__GetVertexCostLayers_Response__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mesh_msgs__srv__GetVertexCostLayers_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetVertexCostLayers_Request__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetVertexCostLayers_Response__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__srv__GetVertexCostLayers_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__srv__GetVertexCostLayers_Event__TYPE_NAME, 39, 39},
      {mesh_msgs__srv__GetVertexCostLayers_Event__FIELDS, 3, 3},
    },
    {mesh_msgs__srv__GetVertexCostLayers_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = mesh_msgs__srv__GetVertexCostLayers_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = mesh_msgs__srv__GetVertexCostLayers_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string uuid\n"
  "---\n"
  "string[] layers";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetVertexCostLayers__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__srv__GetVertexCostLayers__TYPE_NAME, 33, 33},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 31, 31},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetVertexCostLayers_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__srv__GetVertexCostLayers_Request__TYPE_NAME, 41, 41},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetVertexCostLayers_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__srv__GetVertexCostLayers_Response__TYPE_NAME, 42, 42},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetVertexCostLayers_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__srv__GetVertexCostLayers_Event__TYPE_NAME, 39, 39},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetVertexCostLayers__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__srv__GetVertexCostLayers__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *mesh_msgs__srv__GetVertexCostLayers_Event__get_individual_type_description_source(NULL);
    sources[3] = *mesh_msgs__srv__GetVertexCostLayers_Request__get_individual_type_description_source(NULL);
    sources[4] = *mesh_msgs__srv__GetVertexCostLayers_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetVertexCostLayers_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__srv__GetVertexCostLayers_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetVertexCostLayers_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__srv__GetVertexCostLayers_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetVertexCostLayers_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__srv__GetVertexCostLayers_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *mesh_msgs__srv__GetVertexCostLayers_Request__get_individual_type_description_source(NULL);
    sources[3] = *mesh_msgs__srv__GetVertexCostLayers_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
