// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mesh_msgs:srv/GetTexture.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/srv/get_texture.h"


#ifndef MESH_MSGS__SRV__DETAIL__GET_TEXTURE__STRUCT_H_
#define MESH_MSGS__SRV__DETAIL__GET_TEXTURE__STRUCT_H_

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
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetTexture in the package mesh_msgs.
typedef struct mesh_msgs__srv__GetTexture_Request
{
  rosidl_runtime_c__String uuid;
  uint32_t texture_index;
} mesh_msgs__srv__GetTexture_Request;

// Struct for a sequence of mesh_msgs__srv__GetTexture_Request.
typedef struct mesh_msgs__srv__GetTexture_Request__Sequence
{
  mesh_msgs__srv__GetTexture_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__srv__GetTexture_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'texture'
#include "mesh_msgs/msg/detail/mesh_texture__struct.h"

/// Struct defined in srv/GetTexture in the package mesh_msgs.
typedef struct mesh_msgs__srv__GetTexture_Response
{
  mesh_msgs__msg__MeshTexture texture;
} mesh_msgs__srv__GetTexture_Response;

// Struct for a sequence of mesh_msgs__srv__GetTexture_Response.
typedef struct mesh_msgs__srv__GetTexture_Response__Sequence
{
  mesh_msgs__srv__GetTexture_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__srv__GetTexture_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  mesh_msgs__srv__GetTexture_Event__request__MAX_SIZE = 1
};
// response
enum
{
  mesh_msgs__srv__GetTexture_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetTexture in the package mesh_msgs.
typedef struct mesh_msgs__srv__GetTexture_Event
{
  service_msgs__msg__ServiceEventInfo info;
  mesh_msgs__srv__GetTexture_Request__Sequence request;
  mesh_msgs__srv__GetTexture_Response__Sequence response;
} mesh_msgs__srv__GetTexture_Event;

// Struct for a sequence of mesh_msgs__srv__GetTexture_Event.
typedef struct mesh_msgs__srv__GetTexture_Event__Sequence
{
  mesh_msgs__srv__GetTexture_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__srv__GetTexture_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESH_MSGS__SRV__DETAIL__GET_TEXTURE__STRUCT_H_
