// generated from rosidl_generator_c/resource/idl__type_support.h.em
// with input from mesh_msgs:srv/GetVertexCostLayers.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/srv/get_vertex_cost_layers.h"


#ifndef MESH_MSGS__SRV__DETAIL__GET_VERTEX_COST_LAYERS__TYPE_SUPPORT_H_
#define MESH_MSGS__SRV__DETAIL__GET_VERTEX_COST_LAYERS__TYPE_SUPPORT_H_

#include "rosidl_typesupport_interface/macros.h"

#include "mesh_msgs/msg/rosidl_generator_c__visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  mesh_msgs,
  srv,
  GetVertexCostLayers_Request
)(void);

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  mesh_msgs,
  srv,
  GetVertexCostLayers_Response
)(void);

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  mesh_msgs,
  srv,
  GetVertexCostLayers_Event
)(void);

#include "rosidl_runtime_c/service_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
  rosidl_typesupport_c,
  mesh_msgs,
  srv,
  GetVertexCostLayers
)(void);

// Forward declare the function to create a service event message for this type.
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
void *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  mesh_msgs,
  srv,
  GetVertexCostLayers
)(
  const rosidl_service_introspection_info_t * info,
  rcutils_allocator_t * allocator,
  const void * request_message,
  const void * response_message);

// Forward declare the function to destroy a service event message for this type.
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
bool
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  mesh_msgs,
  srv,
  GetVertexCostLayers
)(
  void * event_msg,
  rcutils_allocator_t * allocator);

#ifdef __cplusplus
}
#endif

#endif  // MESH_MSGS__SRV__DETAIL__GET_VERTEX_COST_LAYERS__TYPE_SUPPORT_H_
