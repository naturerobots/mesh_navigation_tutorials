// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mesh_msgs:srv/GetVertexCosts.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/srv/get_vertex_costs.h"


#ifndef MESH_MSGS__SRV__DETAIL__GET_VERTEX_COSTS__STRUCT_H_
#define MESH_MSGS__SRV__DETAIL__GET_VERTEX_COSTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'uuid'
// Member 'layer'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetVertexCosts in the package mesh_msgs.
typedef struct mesh_msgs__srv__GetVertexCosts_Request
{
  rosidl_runtime_c__String uuid;
  /// costlayer (something like "roughness" or "height_diff")
  rosidl_runtime_c__String layer;
} mesh_msgs__srv__GetVertexCosts_Request;

// Struct for a sequence of mesh_msgs__srv__GetVertexCosts_Request.
typedef struct mesh_msgs__srv__GetVertexCosts_Request__Sequence
{
  mesh_msgs__srv__GetVertexCosts_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__srv__GetVertexCosts_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'mesh_vertex_costs_stamped'
#include "mesh_msgs/msg/detail/mesh_vertex_costs_stamped__struct.h"

/// Struct defined in srv/GetVertexCosts in the package mesh_msgs.
typedef struct mesh_msgs__srv__GetVertexCosts_Response
{
  mesh_msgs__msg__MeshVertexCostsStamped mesh_vertex_costs_stamped;
} mesh_msgs__srv__GetVertexCosts_Response;

// Struct for a sequence of mesh_msgs__srv__GetVertexCosts_Response.
typedef struct mesh_msgs__srv__GetVertexCosts_Response__Sequence
{
  mesh_msgs__srv__GetVertexCosts_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__srv__GetVertexCosts_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  mesh_msgs__srv__GetVertexCosts_Event__request__MAX_SIZE = 1
};
// response
enum
{
  mesh_msgs__srv__GetVertexCosts_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetVertexCosts in the package mesh_msgs.
typedef struct mesh_msgs__srv__GetVertexCosts_Event
{
  service_msgs__msg__ServiceEventInfo info;
  mesh_msgs__srv__GetVertexCosts_Request__Sequence request;
  mesh_msgs__srv__GetVertexCosts_Response__Sequence response;
} mesh_msgs__srv__GetVertexCosts_Event;

// Struct for a sequence of mesh_msgs__srv__GetVertexCosts_Event.
typedef struct mesh_msgs__srv__GetVertexCosts_Event__Sequence
{
  mesh_msgs__srv__GetVertexCosts_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__srv__GetVertexCosts_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESH_MSGS__SRV__DETAIL__GET_VERTEX_COSTS__STRUCT_H_
