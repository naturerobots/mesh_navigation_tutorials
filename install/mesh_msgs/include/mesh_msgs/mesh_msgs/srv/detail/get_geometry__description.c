// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from mesh_msgs:srv/GetGeometry.idl
// generated code does not contain a copyright notice

#include "mesh_msgs/srv/detail/get_geometry__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetGeometry__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd3, 0x38, 0x01, 0xb7, 0xf8, 0x56, 0xe1, 0xe9,
      0x70, 0x9b, 0x52, 0x35, 0xc8, 0xdf, 0xba, 0x32,
      0x34, 0x94, 0x64, 0xaf, 0x7b, 0xf9, 0x47, 0x2d,
      0x4b, 0x8a, 0x7e, 0x2a, 0xd3, 0xc8, 0x2f, 0x54,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetGeometry_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x83, 0xe8, 0x97, 0xa4, 0x6b, 0xb1, 0x40, 0x8e,
      0x24, 0x41, 0xd3, 0xee, 0xf6, 0xf8, 0x70, 0xae,
      0xb1, 0x5a, 0x1a, 0xc6, 0xd0, 0x12, 0x24, 0x51,
      0x2c, 0x49, 0xec, 0x95, 0xb0, 0xee, 0x1c, 0x34,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetGeometry_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0d, 0x5b, 0xbf, 0xbd, 0xde, 0x75, 0x54, 0xb8,
      0x14, 0x22, 0xc1, 0x5a, 0x5b, 0x99, 0x21, 0xd2,
      0x1d, 0x60, 0x2e, 0x66, 0x9c, 0xb8, 0x98, 0x2b,
      0x12, 0x3c, 0xf9, 0x82, 0x87, 0x23, 0x3c, 0xb2,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetGeometry_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x79, 0x54, 0x9c, 0x3a, 0x52, 0x5c, 0x6b, 0x78,
      0x7e, 0x8b, 0xbd, 0xb1, 0x12, 0x0c, 0x38, 0xb1,
      0x08, 0x64, 0x0d, 0xcd, 0x7e, 0x13, 0x67, 0x8e,
      0x85, 0x71, 0xdc, 0x70, 0x4b, 0x5c, 0x25, 0xb9,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "mesh_msgs/msg/detail/mesh_triangle_indices__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "mesh_msgs/msg/detail/mesh_geometry__functions.h"
#include "geometry_msgs/msg/detail/point__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "mesh_msgs/msg/detail/mesh_geometry_stamped__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Point__EXPECTED_HASH = {1, {
    0x69, 0x63, 0x08, 0x48, 0x42, 0xa9, 0xb0, 0x44,
    0x94, 0xd6, 0xb2, 0x94, 0x1d, 0x11, 0x44, 0x47,
    0x08, 0xd8, 0x92, 0xda, 0x2f, 0x4b, 0x09, 0x84,
    0x3b, 0x9c, 0x43, 0xf4, 0x2a, 0x7f, 0x68, 0x81,
  }};
static const rosidl_type_hash_t mesh_msgs__msg__MeshGeometry__EXPECTED_HASH = {1, {
    0x98, 0x20, 0x39, 0x0f, 0xa4, 0xde, 0xa7, 0x73,
    0xc6, 0xb9, 0x97, 0xed, 0x00, 0xf4, 0x26, 0x48,
    0x4b, 0xd4, 0x8c, 0x3c, 0xc1, 0xb5, 0x5b, 0xdf,
    0xad, 0x54, 0x48, 0xee, 0xaf, 0x52, 0x05, 0x60,
  }};
static const rosidl_type_hash_t mesh_msgs__msg__MeshGeometryStamped__EXPECTED_HASH = {1, {
    0x4a, 0xcc, 0xf4, 0x34, 0xb9, 0xb8, 0xba, 0x79,
    0x14, 0xb1, 0xa0, 0x52, 0x8d, 0xa5, 0x28, 0x0e,
    0xc1, 0x51, 0xb8, 0x7c, 0x11, 0x01, 0x48, 0xec,
    0x28, 0xe4, 0x89, 0xbb, 0x31, 0x6a, 0xea, 0x95,
  }};
static const rosidl_type_hash_t mesh_msgs__msg__MeshTriangleIndices__EXPECTED_HASH = {1, {
    0x0d, 0xae, 0xe6, 0xbf, 0x6b, 0x63, 0x9f, 0xc7,
    0x4e, 0xb9, 0x2d, 0x51, 0xfb, 0x33, 0xb5, 0x3c,
    0x4f, 0xde, 0xb1, 0xcc, 0x1c, 0x1f, 0x2a, 0x3a,
    0x8a, 0x14, 0x0b, 0x4a, 0x89, 0xa3, 0x71, 0xa1,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char mesh_msgs__srv__GetGeometry__TYPE_NAME[] = "mesh_msgs/srv/GetGeometry";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char geometry_msgs__msg__Point__TYPE_NAME[] = "geometry_msgs/msg/Point";
static char mesh_msgs__msg__MeshGeometry__TYPE_NAME[] = "mesh_msgs/msg/MeshGeometry";
static char mesh_msgs__msg__MeshGeometryStamped__TYPE_NAME[] = "mesh_msgs/msg/MeshGeometryStamped";
static char mesh_msgs__msg__MeshTriangleIndices__TYPE_NAME[] = "mesh_msgs/msg/MeshTriangleIndices";
static char mesh_msgs__srv__GetGeometry_Event__TYPE_NAME[] = "mesh_msgs/srv/GetGeometry_Event";
static char mesh_msgs__srv__GetGeometry_Request__TYPE_NAME[] = "mesh_msgs/srv/GetGeometry_Request";
static char mesh_msgs__srv__GetGeometry_Response__TYPE_NAME[] = "mesh_msgs/srv/GetGeometry_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char mesh_msgs__srv__GetGeometry__FIELD_NAME__request_message[] = "request_message";
static char mesh_msgs__srv__GetGeometry__FIELD_NAME__response_message[] = "response_message";
static char mesh_msgs__srv__GetGeometry__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field mesh_msgs__srv__GetGeometry__FIELDS[] = {
  {
    {mesh_msgs__srv__GetGeometry__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mesh_msgs__srv__GetGeometry_Request__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetGeometry__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mesh_msgs__srv__GetGeometry_Response__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetGeometry__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mesh_msgs__srv__GetGeometry_Event__TYPE_NAME, 31, 31},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mesh_msgs__srv__GetGeometry__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshGeometry__TYPE_NAME, 26, 26},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshGeometryStamped__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshTriangleIndices__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetGeometry_Event__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetGeometry_Request__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetGeometry_Response__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__srv__GetGeometry__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__srv__GetGeometry__TYPE_NAME, 25, 25},
      {mesh_msgs__srv__GetGeometry__FIELDS, 3, 3},
    },
    {mesh_msgs__srv__GetGeometry__REFERENCED_TYPE_DESCRIPTIONS, 10, 10},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshGeometry__EXPECTED_HASH, mesh_msgs__msg__MeshGeometry__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = mesh_msgs__msg__MeshGeometry__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshGeometryStamped__EXPECTED_HASH, mesh_msgs__msg__MeshGeometryStamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = mesh_msgs__msg__MeshGeometryStamped__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshTriangleIndices__EXPECTED_HASH, mesh_msgs__msg__MeshTriangleIndices__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = mesh_msgs__msg__MeshTriangleIndices__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[5].fields = mesh_msgs__srv__GetGeometry_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[6].fields = mesh_msgs__srv__GetGeometry_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[7].fields = mesh_msgs__srv__GetGeometry_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[8].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[9].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mesh_msgs__srv__GetGeometry_Request__FIELD_NAME__uuid[] = "uuid";

static rosidl_runtime_c__type_description__Field mesh_msgs__srv__GetGeometry_Request__FIELDS[] = {
  {
    {mesh_msgs__srv__GetGeometry_Request__FIELD_NAME__uuid, 4, 4},
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
mesh_msgs__srv__GetGeometry_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__srv__GetGeometry_Request__TYPE_NAME, 33, 33},
      {mesh_msgs__srv__GetGeometry_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mesh_msgs__srv__GetGeometry_Response__FIELD_NAME__mesh_geometry_stamped[] = "mesh_geometry_stamped";

static rosidl_runtime_c__type_description__Field mesh_msgs__srv__GetGeometry_Response__FIELDS[] = {
  {
    {mesh_msgs__srv__GetGeometry_Response__FIELD_NAME__mesh_geometry_stamped, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {mesh_msgs__msg__MeshGeometryStamped__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mesh_msgs__srv__GetGeometry_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshGeometry__TYPE_NAME, 26, 26},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshGeometryStamped__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshTriangleIndices__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__srv__GetGeometry_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__srv__GetGeometry_Response__TYPE_NAME, 34, 34},
      {mesh_msgs__srv__GetGeometry_Response__FIELDS, 1, 1},
    },
    {mesh_msgs__srv__GetGeometry_Response__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshGeometry__EXPECTED_HASH, mesh_msgs__msg__MeshGeometry__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = mesh_msgs__msg__MeshGeometry__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshGeometryStamped__EXPECTED_HASH, mesh_msgs__msg__MeshGeometryStamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = mesh_msgs__msg__MeshGeometryStamped__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshTriangleIndices__EXPECTED_HASH, mesh_msgs__msg__MeshTriangleIndices__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = mesh_msgs__msg__MeshTriangleIndices__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char mesh_msgs__srv__GetGeometry_Event__FIELD_NAME__info[] = "info";
static char mesh_msgs__srv__GetGeometry_Event__FIELD_NAME__request[] = "request";
static char mesh_msgs__srv__GetGeometry_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field mesh_msgs__srv__GetGeometry_Event__FIELDS[] = {
  {
    {mesh_msgs__srv__GetGeometry_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetGeometry_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {mesh_msgs__srv__GetGeometry_Request__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetGeometry_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {mesh_msgs__srv__GetGeometry_Response__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription mesh_msgs__srv__GetGeometry_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshGeometry__TYPE_NAME, 26, 26},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshGeometryStamped__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__msg__MeshTriangleIndices__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetGeometry_Request__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {mesh_msgs__srv__GetGeometry_Response__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__srv__GetGeometry_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {mesh_msgs__srv__GetGeometry_Event__TYPE_NAME, 31, 31},
      {mesh_msgs__srv__GetGeometry_Event__FIELDS, 3, 3},
    },
    {mesh_msgs__srv__GetGeometry_Event__REFERENCED_TYPE_DESCRIPTIONS, 9, 9},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshGeometry__EXPECTED_HASH, mesh_msgs__msg__MeshGeometry__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = mesh_msgs__msg__MeshGeometry__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshGeometryStamped__EXPECTED_HASH, mesh_msgs__msg__MeshGeometryStamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = mesh_msgs__msg__MeshGeometryStamped__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&mesh_msgs__msg__MeshTriangleIndices__EXPECTED_HASH, mesh_msgs__msg__MeshTriangleIndices__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = mesh_msgs__msg__MeshTriangleIndices__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[5].fields = mesh_msgs__srv__GetGeometry_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[6].fields = mesh_msgs__srv__GetGeometry_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[8].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string uuid\n"
  "---\n"
  "mesh_msgs/MeshGeometryStamped mesh_geometry_stamped";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetGeometry__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__srv__GetGeometry__TYPE_NAME, 25, 25},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 68, 68},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetGeometry_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__srv__GetGeometry_Request__TYPE_NAME, 33, 33},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetGeometry_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__srv__GetGeometry_Response__TYPE_NAME, 34, 34},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetGeometry_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {mesh_msgs__srv__GetGeometry_Event__TYPE_NAME, 31, 31},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetGeometry__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[11];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 11, 11};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__srv__GetGeometry__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[3] = *mesh_msgs__msg__MeshGeometry__get_individual_type_description_source(NULL);
    sources[4] = *mesh_msgs__msg__MeshGeometryStamped__get_individual_type_description_source(NULL);
    sources[5] = *mesh_msgs__msg__MeshTriangleIndices__get_individual_type_description_source(NULL);
    sources[6] = *mesh_msgs__srv__GetGeometry_Event__get_individual_type_description_source(NULL);
    sources[7] = *mesh_msgs__srv__GetGeometry_Request__get_individual_type_description_source(NULL);
    sources[8] = *mesh_msgs__srv__GetGeometry_Response__get_individual_type_description_source(NULL);
    sources[9] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[10] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetGeometry_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__srv__GetGeometry_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetGeometry_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__srv__GetGeometry_Response__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[3] = *mesh_msgs__msg__MeshGeometry__get_individual_type_description_source(NULL);
    sources[4] = *mesh_msgs__msg__MeshGeometryStamped__get_individual_type_description_source(NULL);
    sources[5] = *mesh_msgs__msg__MeshTriangleIndices__get_individual_type_description_source(NULL);
    sources[6] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetGeometry_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[10];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 10, 10};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *mesh_msgs__srv__GetGeometry_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[3] = *mesh_msgs__msg__MeshGeometry__get_individual_type_description_source(NULL);
    sources[4] = *mesh_msgs__msg__MeshGeometryStamped__get_individual_type_description_source(NULL);
    sources[5] = *mesh_msgs__msg__MeshTriangleIndices__get_individual_type_description_source(NULL);
    sources[6] = *mesh_msgs__srv__GetGeometry_Request__get_individual_type_description_source(NULL);
    sources[7] = *mesh_msgs__srv__GetGeometry_Response__get_individual_type_description_source(NULL);
    sources[8] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[9] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
