// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mesh_msgs:srv/GetUUIDs.idl
// generated code does not contain a copyright notice

#include "mesh_msgs/srv/detail/get_uui_ds__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetUUIDs__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5c, 0x01, 0x48, 0x01, 0xc9, 0x53, 0x6e, 0x08,
      0xcc, 0xc2, 0x3a, 0x7e, 0x87, 0xdd, 0x61, 0x2e,
      0x77, 0xc7, 0xb1, 0x7a, 0x76, 0xd8, 0xac, 0x82,
      0xee, 0xfe, 0x7f, 0xd4, 0x25, 0x01, 0xa6, 0x16,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetUUIDs_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe3, 0x50, 0x44, 0xc8, 0xbb, 0x1f, 0xf1, 0x1e,
      0xe6, 0x74, 0xa0, 0x21, 0xad, 0xc0, 0xf2, 0x56,
      0xe0, 0x63, 0x30, 0x01, 0x1c, 0xfc, 0xd5, 0xca,
      0x9a, 0x27, 0x89, 0x3a, 0x4e, 0x5f, 0x60, 0x14,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetUUIDs_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe3, 0x91, 0xd3, 0x28, 0x1f, 0x30, 0x0d, 0x72,
      0x50, 0x3b, 0x5f, 0x9c, 0x83, 0xe6, 0xce, 0x75,
      0xb1, 0xe8, 0xd4, 0x20, 0x66, 0x1c, 0xd3, 0x62,
      0xc7, 0x5d, 0x42, 0x1b, 0xe4, 0x12, 0x65, 0xd9,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetUUIDs_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x78, 0xdf, 0x03, 0x49, 0xb9, 0x0c, 0xb6, 0x5a,
      0x88, 0x6c, 0xed, 0x7c, 0x0e, 0xa3, 0xd9, 0xf2,
      0x46, 0x3e, 0xa2, 0x5b, 0x16, 0xd0, 0x8d, 0x4f,
      0xaa, 0x91, 0xbd, 0x9e, 0xa9, 0x48, 0x26, 0x8a,
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

static char mesh_msgs__srv__GetUUIDs__TYPE_NAME[] = "mesh_msgs/srv/GetUUIDs";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char mesh_msgs__srv__GetUUIDs_Event__TYPE_NAME[] = "mesh_msgs/srv/GetUUIDs_Event";
static char mesh_msgs__srv__GetUUIDs_Request__TYPE_NAME[] = "mesh_msgs/srv/GetUUIDs_Request";
static char mesh_msgs__srv__GetUUIDs_Response__TYPE_NAME[] = "mesh_msgs/srv/GetUUIDs_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char mesh_msgs__srv__GetUUIDs__FIELD_NAME__request_message[] = "request_message";
static char mesh_msgs__srv__GetUUIDs__FIELD_NAME__response_message[] = "response_message";
static char mesh_msgs__srv__GetUUIDs__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field mesh_msgs__srv__GetUUIDs__FIELDS[] = {
  {
    {mesh_msgs__srv__GetUUIDs__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mesh_msgs__srv__GetUUIDs_Request__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetUUIDs__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mesh_msgs__srv__GetUUIDs_Response__TYPE_NAME, 31, 31},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetUUIDs__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mesh_msgs__srv__GetUUIDs_Event__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mesh_msgs__srv__GetUUIDs__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetUUIDs_Event__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetUUIDs_Request__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetUUIDs_Response__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__srv__GetUUIDs__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__srv__GetUUIDs__TYPE_NAME, 22, 22},
      {mesh_msgs__srv__GetUUIDs__FIELDS, 3, 3},
    },
    {mesh_msgs__srv__GetUUIDs__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = mesh_msgs__srv__GetUUIDs_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = mesh_msgs__srv__GetUUIDs_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = mesh_msgs__srv__GetUUIDs_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mesh_msgs__srv__GetUUIDs_Request__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field mesh_msgs__srv__GetUUIDs_Request__FIELDS[] = {
  {
    {mesh_msgs__srv__GetUUIDs_Request__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__srv__GetUUIDs_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__srv__GetUUIDs_Request__TYPE_NAME, 30, 30},
      {mesh_msgs__srv__GetUUIDs_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mesh_msgs__srv__GetUUIDs_Response__FIELD_NAME__uuids[] = "uuids";

static rosidl_runtime_c__type_description__Field mesh_msgs__srv__GetUUIDs_Response__FIELDS[] = {
  {
    {mesh_msgs__srv__GetUUIDs_Response__FIELD_NAME__uuids, 5, 5},
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
mesh_msgs__srv__GetUUIDs_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__srv__GetUUIDs_Response__TYPE_NAME, 31, 31},
      {mesh_msgs__srv__GetUUIDs_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mesh_msgs__srv__GetUUIDs_Event__FIELD_NAME__info[] = "info";
static char mesh_msgs__srv__GetUUIDs_Event__FIELD_NAME__request[] = "request";
static char mesh_msgs__srv__GetUUIDs_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field mesh_msgs__srv__GetUUIDs_Event__FIELDS[] = {
  {
    {mesh_msgs__srv__GetUUIDs_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetUUIDs_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {mesh_msgs__srv__GetUUIDs_Request__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetUUIDs_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {mesh_msgs__srv__GetUUIDs_Response__TYPE_NAME, 31, 31},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mesh_msgs__srv__GetUUIDs_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetUUIDs_Request__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetUUIDs_Response__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__srv__GetUUIDs_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__srv__GetUUIDs_Event__TYPE_NAME, 28, 28},
      {mesh_msgs__srv__GetUUIDs_Event__FIELDS, 3, 3},
    },
    {mesh_msgs__srv__GetUUIDs_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = mesh_msgs__srv__GetUUIDs_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = mesh_msgs__srv__GetUUIDs_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# uuids of all currently available meshes\n"
  "\n"
  "---\n"
  "string[] uuids";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetUUIDs__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__srv__GetUUIDs__TYPE_NAME, 22, 22},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 62, 62},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetUUIDs_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__srv__GetUUIDs_Request__TYPE_NAME, 30, 30},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetUUIDs_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__srv__GetUUIDs_Response__TYPE_NAME, 31, 31},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetUUIDs_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__srv__GetUUIDs_Event__TYPE_NAME, 28, 28},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetUUIDs__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__srv__GetUUIDs__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *mesh_msgs__srv__GetUUIDs_Event__get_individual_type_description_source(NULL);
    sources[3] = *mesh_msgs__srv__GetUUIDs_Request__get_individual_type_description_source(NULL);
    sources[4] = *mesh_msgs__srv__GetUUIDs_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetUUIDs_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__srv__GetUUIDs_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetUUIDs_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__srv__GetUUIDs_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetUUIDs_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__srv__GetUUIDs_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *mesh_msgs__srv__GetUUIDs_Request__get_individual_type_description_source(NULL);
    sources[3] = *mesh_msgs__srv__GetUUIDs_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
