// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mesh_msgs:srv/GetLabeledClusters.idl
// generated code does not contain a copyright notice

#include "mesh_msgs/srv/detail/get_labeled_clusters__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetLabeledClusters__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x80, 0x18, 0xa5, 0xe4, 0x49, 0x99, 0x1c, 0x9d,
      0xaa, 0x99, 0x08, 0x87, 0x20, 0x1f, 0xa7, 0x81,
      0x32, 0x2e, 0xfd, 0xf4, 0xd7, 0x06, 0xfa, 0x9d,
      0x26, 0x98, 0x53, 0x61, 0xa2, 0xf1, 0x59, 0x3f,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetLabeledClusters_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xbd, 0x4f, 0xed, 0xda, 0x4d, 0xc3, 0xe2, 0x36,
      0x8d, 0x76, 0x6b, 0x43, 0xc1, 0x64, 0xca, 0x65,
      0xf5, 0x77, 0x59, 0x95, 0x46, 0x71, 0x03, 0x50,
      0xb7, 0xde, 0x8f, 0x10, 0xf8, 0xf4, 0xa9, 0xdc,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetLabeledClusters_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc6, 0xd2, 0xb1, 0xe7, 0xa1, 0x35, 0x01, 0x19,
      0x8e, 0xde, 0x5d, 0x2d, 0xf8, 0x33, 0xca, 0x37,
      0xcc, 0x3a, 0x20, 0x46, 0x61, 0x7d, 0x7c, 0x57,
      0xcc, 0x05, 0x21, 0x6f, 0x69, 0x82, 0x55, 0x38,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetLabeledClusters_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x19, 0x7a, 0x96, 0x68, 0x1a, 0xc6, 0x05, 0x83,
      0xd3, 0x7d, 0x16, 0x42, 0xb7, 0xd0, 0xe1, 0x09,
      0x64, 0x8c, 0x75, 0x09, 0x9c, 0x61, 0xd6, 0x1c,
      0xbe, 0x5b, 0x88, 0x27, 0xd1, 0x80, 0xa0, 0x17,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "mesh_msgs/msg/detail/mesh_face_cluster__functions.h"
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
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char mesh_msgs__srv__GetLabeledClusters__TYPE_NAME[] = "mesh_msgs/srv/GetLabeledClusters";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char mesh_msgs__msg__MeshFaceCluster__TYPE_NAME[] = "mesh_msgs/msg/MeshFaceCluster";
static char mesh_msgs__srv__GetLabeledClusters_Event__TYPE_NAME[] = "mesh_msgs/srv/GetLabeledClusters_Event";
static char mesh_msgs__srv__GetLabeledClusters_Request__TYPE_NAME[] = "mesh_msgs/srv/GetLabeledClusters_Request";
static char mesh_msgs__srv__GetLabeledClusters_Response__TYPE_NAME[] = "mesh_msgs/srv/GetLabeledClusters_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char mesh_msgs__srv__GetLabeledClusters__FIELD_NAME__request_message[] = "request_message";
static char mesh_msgs__srv__GetLabeledClusters__FIELD_NAME__response_message[] = "response_message";
static char mesh_msgs__srv__GetLabeledClusters__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field mesh_msgs__srv__GetLabeledClusters__FIELDS[] = {
  {
    {mesh_msgs__srv__GetLabeledClusters__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mesh_msgs__srv__GetLabeledClusters_Request__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetLabeledClusters__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mesh_msgs__srv__GetLabeledClusters_Response__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetLabeledClusters__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mesh_msgs__srv__GetLabeledClusters_Event__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mesh_msgs__srv__GetLabeledClusters__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshFaceCluster__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetLabeledClusters_Event__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetLabeledClusters_Request__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetLabeledClusters_Response__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__srv__GetLabeledClusters__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__srv__GetLabeledClusters__TYPE_NAME, 32, 32},
      {mesh_msgs__srv__GetLabeledClusters__FIELDS, 3, 3},
    },
    {mesh_msgs__srv__GetLabeledClusters__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshFaceCluster__EXPECTED_HASH, mesh_msgs__msg__MeshFaceCluster__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = mesh_msgs__msg__MeshFaceCluster__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = mesh_msgs__srv__GetLabeledClusters_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = mesh_msgs__srv__GetLabeledClusters_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = mesh_msgs__srv__GetLabeledClusters_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mesh_msgs__srv__GetLabeledClusters_Request__FIELD_NAME__uuid[] = "uuid";

static rosidl_runtime_c__type_description__Field mesh_msgs__srv__GetLabeledClusters_Request__FIELDS[] = {
  {
    {mesh_msgs__srv__GetLabeledClusters_Request__FIELD_NAME__uuid, 4, 4},
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
mesh_msgs__srv__GetLabeledClusters_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__srv__GetLabeledClusters_Request__TYPE_NAME, 40, 40},
      {mesh_msgs__srv__GetLabeledClusters_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mesh_msgs__srv__GetLabeledClusters_Response__FIELD_NAME__clusters[] = "clusters";

static rosidl_runtime_c__type_description__Field mesh_msgs__srv__GetLabeledClusters_Response__FIELDS[] = {
  {
    {mesh_msgs__srv__GetLabeledClusters_Response__FIELD_NAME__clusters, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {mesh_msgs__msg__MeshFaceCluster__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mesh_msgs__srv__GetLabeledClusters_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {mesh_msgs__msg__MeshFaceCluster__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__srv__GetLabeledClusters_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__srv__GetLabeledClusters_Response__TYPE_NAME, 41, 41},
      {mesh_msgs__srv__GetLabeledClusters_Response__FIELDS, 1, 1},
    },
    {mesh_msgs__srv__GetLabeledClusters_Response__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&mesh_msgs__msg__MeshFaceCluster__EXPECTED_HASH, mesh_msgs__msg__MeshFaceCluster__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = mesh_msgs__msg__MeshFaceCluster__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mesh_msgs__srv__GetLabeledClusters_Event__FIELD_NAME__info[] = "info";
static char mesh_msgs__srv__GetLabeledClusters_Event__FIELD_NAME__request[] = "request";
static char mesh_msgs__srv__GetLabeledClusters_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field mesh_msgs__srv__GetLabeledClusters_Event__FIELDS[] = {
  {
    {mesh_msgs__srv__GetLabeledClusters_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetLabeledClusters_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {mesh_msgs__srv__GetLabeledClusters_Request__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetLabeledClusters_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {mesh_msgs__srv__GetLabeledClusters_Response__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mesh_msgs__srv__GetLabeledClusters_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshFaceCluster__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetLabeledClusters_Request__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetLabeledClusters_Response__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__srv__GetLabeledClusters_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__srv__GetLabeledClusters_Event__TYPE_NAME, 38, 38},
      {mesh_msgs__srv__GetLabeledClusters_Event__FIELDS, 3, 3},
    },
    {mesh_msgs__srv__GetLabeledClusters_Event__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshFaceCluster__EXPECTED_HASH, mesh_msgs__msg__MeshFaceCluster__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = mesh_msgs__msg__MeshFaceCluster__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = mesh_msgs__srv__GetLabeledClusters_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = mesh_msgs__srv__GetLabeledClusters_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string uuid\n"
  "---\n"
  "MeshFaceCluster[] clusters";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetLabeledClusters__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__srv__GetLabeledClusters__TYPE_NAME, 32, 32},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 43, 43},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetLabeledClusters_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__srv__GetLabeledClusters_Request__TYPE_NAME, 40, 40},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetLabeledClusters_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__srv__GetLabeledClusters_Response__TYPE_NAME, 41, 41},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetLabeledClusters_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__srv__GetLabeledClusters_Event__TYPE_NAME, 38, 38},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetLabeledClusters__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__srv__GetLabeledClusters__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *mesh_msgs__msg__MeshFaceCluster__get_individual_type_description_source(NULL);
    sources[3] = *mesh_msgs__srv__GetLabeledClusters_Event__get_individual_type_description_source(NULL);
    sources[4] = *mesh_msgs__srv__GetLabeledClusters_Request__get_individual_type_description_source(NULL);
    sources[5] = *mesh_msgs__srv__GetLabeledClusters_Response__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetLabeledClusters_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__srv__GetLabeledClusters_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetLabeledClusters_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__srv__GetLabeledClusters_Response__get_individual_type_description_source(NULL),
    sources[1] = *mesh_msgs__msg__MeshFaceCluster__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetLabeledClusters_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__srv__GetLabeledClusters_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *mesh_msgs__msg__MeshFaceCluster__get_individual_type_description_source(NULL);
    sources[3] = *mesh_msgs__srv__GetLabeledClusters_Request__get_individual_type_description_source(NULL);
    sources[4] = *mesh_msgs__srv__GetLabeledClusters_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
