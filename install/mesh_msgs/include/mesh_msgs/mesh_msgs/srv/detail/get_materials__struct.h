// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mesh_msgs:srv/GetMaterials.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/srv/get_materials.h"


#ifndef MESH_MSGS__SRV__DETAIL__GET_MATERIALS__STRUCT_H_
#define MESH_MSGS__SRV__DETAIL__GET_MATERIALS__STRUCT_H_

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

/// Struct defined in srv/GetMaterials in the package mesh_msgs.
typedef struct mesh_msgs__srv__GetMaterials_Request
{
  rosidl_runtime_c__String uuid;
} mesh_msgs__srv__GetMaterials_Request;

// Struct for a sequence of mesh_msgs__srv__GetMaterials_Request.
typedef struct mesh_msgs__srv__GetMaterials_Request__Sequence
{
  mesh_msgs__srv__GetMaterials_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__srv__GetMaterials_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'mesh_materials_stamped'
#include "mesh_msgs/msg/detail/mesh_materials_stamped__struct.h"

/// Struct defined in srv/GetMaterials in the package mesh_msgs.
typedef struct mesh_msgs__srv__GetMaterials_Response
{
  mesh_msgs__msg__MeshMaterialsStamped mesh_materials_stamped;
} mesh_msgs__srv__GetMaterials_Response;

// Struct for a sequence of mesh_msgs__srv__GetMaterials_Response.
typedef struct mesh_msgs__srv__GetMaterials_Response__Sequence
{
  mesh_msgs__srv__GetMaterials_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__srv__GetMaterials_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  mesh_msgs__srv__GetMaterials_Event__request__MAX_SIZE = 1
};
// response
enum
{
  mesh_msgs__srv__GetMaterials_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetMaterials in the package mesh_msgs.
typedef struct mesh_msgs__srv__GetMaterials_Event
{
  service_msgs__msg__ServiceEventInfo info;
  mesh_msgs__srv__GetMaterials_Request__Sequence request;
  mesh_msgs__srv__GetMaterials_Response__Sequence response;
} mesh_msgs__srv__GetMaterials_Event;

// Struct for a sequence of mesh_msgs__srv__GetMaterials_Event.
typedef struct mesh_msgs__srv__GetMaterials_Event__Sequence
{
  mesh_msgs__srv__GetMaterials_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mesh_msgs__srv__GetMaterials_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESH_MSGS__SRV__DETAIL__GET_MATERIALS__STRUCT_H_
