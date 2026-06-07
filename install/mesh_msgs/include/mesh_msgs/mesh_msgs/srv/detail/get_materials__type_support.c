// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mesh_msgs:srv/GetMaterials.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mesh_msgs/srv/detail/get_materials__rosidl_typesupport_introspection_c.h"
#include "mesh_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mesh_msgs/srv/detail/get_materials__functions.h"
#include "mesh_msgs/srv/detail/get_materials__struct.h"


// Include directives for member types
// Member `uuid`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mesh_msgs__srv__GetMaterials_Request__rosidl_typesupport_introspection_c__GetMaterials_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mesh_msgs__srv__GetMaterials_Request__init(message_memory);
}

void mesh_msgs__srv__GetMaterials_Request__rosidl_typesupport_introspection_c__GetMaterials_Request_fini_function(void * message_memory)
{
  mesh_msgs__srv__GetMaterials_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mesh_msgs__srv__GetMaterials_Request__rosidl_typesupport_introspection_c__GetMaterials_Request_message_member_array[1] = {
  {
    "uuid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs__srv__GetMaterials_Request, uuid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mesh_msgs__srv__GetMaterials_Request__rosidl_typesupport_introspection_c__GetMaterials_Request_message_members = {
  "mesh_msgs__srv",  // message namespace
  "GetMaterials_Request",  // message name
  1,  // number of fields
  sizeof(mesh_msgs__srv__GetMaterials_Request),
  false,  // has_any_key_member_
  mesh_msgs__srv__GetMaterials_Request__rosidl_typesupport_introspection_c__GetMaterials_Request_message_member_array,  // message members
  mesh_msgs__srv__GetMaterials_Request__rosidl_typesupport_introspection_c__GetMaterials_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  mesh_msgs__srv__GetMaterials_Request__rosidl_typesupport_introspection_c__GetMaterials_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mesh_msgs__srv__GetMaterials_Request__rosidl_typesupport_introspection_c__GetMaterials_Request_message_type_support_handle = {
  0,
  &mesh_msgs__srv__GetMaterials_Request__rosidl_typesupport_introspection_c__GetMaterials_Request_message_members,
  get_message_typesupport_handle_function,
  &mesh_msgs__srv__GetMaterials_Request__get_type_hash,
  &mesh_msgs__srv__GetMaterials_Request__get_type_description,
  &mesh_msgs__srv__GetMaterials_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mesh_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, srv, GetMaterials_Request)() {
  if (!mesh_msgs__srv__GetMaterials_Request__rosidl_typesupport_introspection_c__GetMaterials_Request_message_type_support_handle.typesupport_identifier) {
    mesh_msgs__srv__GetMaterials_Request__rosidl_typesupport_introspection_c__GetMaterials_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mesh_msgs__srv__GetMaterials_Request__rosidl_typesupport_introspection_c__GetMaterials_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "mesh_msgs/srv/detail/get_materials__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mesh_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mesh_msgs/srv/detail/get_materials__functions.h"
// already included above
// #include "mesh_msgs/srv/detail/get_materials__struct.h"


// Include directives for member types
// Member `mesh_materials_stamped`
#include "mesh_msgs/msg/mesh_materials_stamped.h"
// Member `mesh_materials_stamped`
#include "mesh_msgs/msg/detail/mesh_materials_stamped__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mesh_msgs__srv__GetMaterials_Response__rosidl_typesupport_introspection_c__GetMaterials_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mesh_msgs__srv__GetMaterials_Response__init(message_memory);
}

void mesh_msgs__srv__GetMaterials_Response__rosidl_typesupport_introspection_c__GetMaterials_Response_fini_function(void * message_memory)
{
  mesh_msgs__srv__GetMaterials_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mesh_msgs__srv__GetMaterials_Response__rosidl_typesupport_introspection_c__GetMaterials_Response_message_member_array[1] = {
  {
    "mesh_materials_stamped",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs__srv__GetMaterials_Response, mesh_materials_stamped),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mesh_msgs__srv__GetMaterials_Response__rosidl_typesupport_introspection_c__GetMaterials_Response_message_members = {
  "mesh_msgs__srv",  // message namespace
  "GetMaterials_Response",  // message name
  1,  // number of fields
  sizeof(mesh_msgs__srv__GetMaterials_Response),
  false,  // has_any_key_member_
  mesh_msgs__srv__GetMaterials_Response__rosidl_typesupport_introspection_c__GetMaterials_Response_message_member_array,  // message members
  mesh_msgs__srv__GetMaterials_Response__rosidl_typesupport_introspection_c__GetMaterials_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  mesh_msgs__srv__GetMaterials_Response__rosidl_typesupport_introspection_c__GetMaterials_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mesh_msgs__srv__GetMaterials_Response__rosidl_typesupport_introspection_c__GetMaterials_Response_message_type_support_handle = {
  0,
  &mesh_msgs__srv__GetMaterials_Response__rosidl_typesupport_introspection_c__GetMaterials_Response_message_members,
  get_message_typesupport_handle_function,
  &mesh_msgs__srv__GetMaterials_Response__get_type_hash,
  &mesh_msgs__srv__GetMaterials_Response__get_type_description,
  &mesh_msgs__srv__GetMaterials_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mesh_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, srv, GetMaterials_Response)() {
  mesh_msgs__srv__GetMaterials_Response__rosidl_typesupport_introspection_c__GetMaterials_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, msg, MeshMaterialsStamped)();
  if (!mesh_msgs__srv__GetMaterials_Response__rosidl_typesupport_introspection_c__GetMaterials_Response_message_type_support_handle.typesupport_identifier) {
    mesh_msgs__srv__GetMaterials_Response__rosidl_typesupport_introspection_c__GetMaterials_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mesh_msgs__srv__GetMaterials_Response__rosidl_typesupport_introspection_c__GetMaterials_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "mesh_msgs/srv/detail/get_materials__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mesh_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mesh_msgs/srv/detail/get_materials__functions.h"
// already included above
// #include "mesh_msgs/srv/detail/get_materials__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "mesh_msgs/srv/get_materials.h"
// Member `request`
// Member `response`
// already included above
// #include "mesh_msgs/srv/detail/get_materials__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__GetMaterials_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mesh_msgs__srv__GetMaterials_Event__init(message_memory);
}

void mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__GetMaterials_Event_fini_function(void * message_memory)
{
  mesh_msgs__srv__GetMaterials_Event__fini(message_memory);
}

size_t mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__size_function__GetMaterials_Event__request(
  const void * untyped_member)
{
  const mesh_msgs__srv__GetMaterials_Request__Sequence * member =
    (const mesh_msgs__srv__GetMaterials_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__get_const_function__GetMaterials_Event__request(
  const void * untyped_member, size_t index)
{
  const mesh_msgs__srv__GetMaterials_Request__Sequence * member =
    (const mesh_msgs__srv__GetMaterials_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__get_function__GetMaterials_Event__request(
  void * untyped_member, size_t index)
{
  mesh_msgs__srv__GetMaterials_Request__Sequence * member =
    (mesh_msgs__srv__GetMaterials_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__fetch_function__GetMaterials_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const mesh_msgs__srv__GetMaterials_Request * item =
    ((const mesh_msgs__srv__GetMaterials_Request *)
    mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__get_const_function__GetMaterials_Event__request(untyped_member, index));
  mesh_msgs__srv__GetMaterials_Request * value =
    (mesh_msgs__srv__GetMaterials_Request *)(untyped_value);
  *value = *item;
}

void mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__assign_function__GetMaterials_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  mesh_msgs__srv__GetMaterials_Request * item =
    ((mesh_msgs__srv__GetMaterials_Request *)
    mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__get_function__GetMaterials_Event__request(untyped_member, index));
  const mesh_msgs__srv__GetMaterials_Request * value =
    (const mesh_msgs__srv__GetMaterials_Request *)(untyped_value);
  *item = *value;
}

bool mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__resize_function__GetMaterials_Event__request(
  void * untyped_member, size_t size)
{
  mesh_msgs__srv__GetMaterials_Request__Sequence * member =
    (mesh_msgs__srv__GetMaterials_Request__Sequence *)(untyped_member);
  mesh_msgs__srv__GetMaterials_Request__Sequence__fini(member);
  return mesh_msgs__srv__GetMaterials_Request__Sequence__init(member, size);
}

size_t mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__size_function__GetMaterials_Event__response(
  const void * untyped_member)
{
  const mesh_msgs__srv__GetMaterials_Response__Sequence * member =
    (const mesh_msgs__srv__GetMaterials_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__get_const_function__GetMaterials_Event__response(
  const void * untyped_member, size_t index)
{
  const mesh_msgs__srv__GetMaterials_Response__Sequence * member =
    (const mesh_msgs__srv__GetMaterials_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__get_function__GetMaterials_Event__response(
  void * untyped_member, size_t index)
{
  mesh_msgs__srv__GetMaterials_Response__Sequence * member =
    (mesh_msgs__srv__GetMaterials_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__fetch_function__GetMaterials_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const mesh_msgs__srv__GetMaterials_Response * item =
    ((const mesh_msgs__srv__GetMaterials_Response *)
    mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__get_const_function__GetMaterials_Event__response(untyped_member, index));
  mesh_msgs__srv__GetMaterials_Response * value =
    (mesh_msgs__srv__GetMaterials_Response *)(untyped_value);
  *value = *item;
}

void mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__assign_function__GetMaterials_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  mesh_msgs__srv__GetMaterials_Response * item =
    ((mesh_msgs__srv__GetMaterials_Response *)
    mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__get_function__GetMaterials_Event__response(untyped_member, index));
  const mesh_msgs__srv__GetMaterials_Response * value =
    (const mesh_msgs__srv__GetMaterials_Response *)(untyped_value);
  *item = *value;
}

bool mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__resize_function__GetMaterials_Event__response(
  void * untyped_member, size_t size)
{
  mesh_msgs__srv__GetMaterials_Response__Sequence * member =
    (mesh_msgs__srv__GetMaterials_Response__Sequence *)(untyped_member);
  mesh_msgs__srv__GetMaterials_Response__Sequence__fini(member);
  return mesh_msgs__srv__GetMaterials_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__GetMaterials_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mesh_msgs__srv__GetMaterials_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(mesh_msgs__srv__GetMaterials_Event, request),  // bytes offset in struct
    NULL,  // default value
    mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__size_function__GetMaterials_Event__request,  // size() function pointer
    mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__get_const_function__GetMaterials_Event__request,  // get_const(index) function pointer
    mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__get_function__GetMaterials_Event__request,  // get(index) function pointer
    mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__fetch_function__GetMaterials_Event__request,  // fetch(index, &value) function pointer
    mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__assign_function__GetMaterials_Event__request,  // assign(index, value) function pointer
    mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__resize_function__GetMaterials_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(mesh_msgs__srv__GetMaterials_Event, response),  // bytes offset in struct
    NULL,  // default value
    mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__size_function__GetMaterials_Event__response,  // size() function pointer
    mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__get_const_function__GetMaterials_Event__response,  // get_const(index) function pointer
    mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__get_function__GetMaterials_Event__response,  // get(index) function pointer
    mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__fetch_function__GetMaterials_Event__response,  // fetch(index, &value) function pointer
    mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__assign_function__GetMaterials_Event__response,  // assign(index, value) function pointer
    mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__resize_function__GetMaterials_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__GetMaterials_Event_message_members = {
  "mesh_msgs__srv",  // message namespace
  "GetMaterials_Event",  // message name
  3,  // number of fields
  sizeof(mesh_msgs__srv__GetMaterials_Event),
  false,  // has_any_key_member_
  mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__GetMaterials_Event_message_member_array,  // message members
  mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__GetMaterials_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__GetMaterials_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__GetMaterials_Event_message_type_support_handle = {
  0,
  &mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__GetMaterials_Event_message_members,
  get_message_typesupport_handle_function,
  &mesh_msgs__srv__GetMaterials_Event__get_type_hash,
  &mesh_msgs__srv__GetMaterials_Event__get_type_description,
  &mesh_msgs__srv__GetMaterials_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mesh_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, srv, GetMaterials_Event)() {
  mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__GetMaterials_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__GetMaterials_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, srv, GetMaterials_Request)();
  mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__GetMaterials_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, srv, GetMaterials_Response)();
  if (!mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__GetMaterials_Event_message_type_support_handle.typesupport_identifier) {
    mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__GetMaterials_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__GetMaterials_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "mesh_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "mesh_msgs/srv/detail/get_materials__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers mesh_msgs__srv__detail__get_materials__rosidl_typesupport_introspection_c__GetMaterials_service_members = {
  "mesh_msgs__srv",  // service namespace
  "GetMaterials",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // mesh_msgs__srv__detail__get_materials__rosidl_typesupport_introspection_c__GetMaterials_Request_message_type_support_handle,
  NULL,  // response message
  // mesh_msgs__srv__detail__get_materials__rosidl_typesupport_introspection_c__GetMaterials_Response_message_type_support_handle
  NULL  // event_message
  // mesh_msgs__srv__detail__get_materials__rosidl_typesupport_introspection_c__GetMaterials_Response_message_type_support_handle
};


static rosidl_service_type_support_t mesh_msgs__srv__detail__get_materials__rosidl_typesupport_introspection_c__GetMaterials_service_type_support_handle = {
  0,
  &mesh_msgs__srv__detail__get_materials__rosidl_typesupport_introspection_c__GetMaterials_service_members,
  get_service_typesupport_handle_function,
  &mesh_msgs__srv__GetMaterials_Request__rosidl_typesupport_introspection_c__GetMaterials_Request_message_type_support_handle,
  &mesh_msgs__srv__GetMaterials_Response__rosidl_typesupport_introspection_c__GetMaterials_Response_message_type_support_handle,
  &mesh_msgs__srv__GetMaterials_Event__rosidl_typesupport_introspection_c__GetMaterials_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    mesh_msgs,
    srv,
    GetMaterials
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    mesh_msgs,
    srv,
    GetMaterials
  ),
  &mesh_msgs__srv__GetMaterials__get_type_hash,
  &mesh_msgs__srv__GetMaterials__get_type_description,
  &mesh_msgs__srv__GetMaterials__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, srv, GetMaterials_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, srv, GetMaterials_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, srv, GetMaterials_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mesh_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, srv, GetMaterials)(void) {
  if (!mesh_msgs__srv__detail__get_materials__rosidl_typesupport_introspection_c__GetMaterials_service_type_support_handle.typesupport_identifier) {
    mesh_msgs__srv__detail__get_materials__rosidl_typesupport_introspection_c__GetMaterials_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)mesh_msgs__srv__detail__get_materials__rosidl_typesupport_introspection_c__GetMaterials_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, srv, GetMaterials_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, srv, GetMaterials_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mesh_msgs, srv, GetMaterials_Event)()->data;
  }

  return &mesh_msgs__srv__detail__get_materials__rosidl_typesupport_introspection_c__GetMaterials_service_type_support_handle;
}
