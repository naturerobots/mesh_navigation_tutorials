// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from mbf_msgs:srv/FindValidPose.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "mbf_msgs/srv/detail/find_valid_pose__struct.h"
#include "mbf_msgs/srv/detail/find_valid_pose__type_support.h"
#include "mbf_msgs/srv/detail/find_valid_pose__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace mbf_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _FindValidPose_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FindValidPose_Request_type_support_ids_t;

static const _FindValidPose_Request_type_support_ids_t _FindValidPose_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _FindValidPose_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FindValidPose_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FindValidPose_Request_type_support_symbol_names_t _FindValidPose_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mbf_msgs, srv, FindValidPose_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, FindValidPose_Request)),
  }
};

typedef struct _FindValidPose_Request_type_support_data_t
{
  void * data[2];
} _FindValidPose_Request_type_support_data_t;

static _FindValidPose_Request_type_support_data_t _FindValidPose_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FindValidPose_Request_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_FindValidPose_Request_message_typesupport_ids.typesupport_identifier[0],
  &_FindValidPose_Request_message_typesupport_symbol_names.symbol_name[0],
  &_FindValidPose_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FindValidPose_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FindValidPose_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &mbf_msgs__srv__FindValidPose_Request__get_type_hash,
  &mbf_msgs__srv__FindValidPose_Request__get_type_description,
  &mbf_msgs__srv__FindValidPose_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace mbf_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, mbf_msgs, srv, FindValidPose_Request)() {
  return &::mbf_msgs::srv::rosidl_typesupport_c::FindValidPose_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "mbf_msgs/srv/detail/find_valid_pose__struct.h"
// already included above
// #include "mbf_msgs/srv/detail/find_valid_pose__type_support.h"
// already included above
// #include "mbf_msgs/srv/detail/find_valid_pose__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace mbf_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _FindValidPose_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FindValidPose_Response_type_support_ids_t;

static const _FindValidPose_Response_type_support_ids_t _FindValidPose_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _FindValidPose_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FindValidPose_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FindValidPose_Response_type_support_symbol_names_t _FindValidPose_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mbf_msgs, srv, FindValidPose_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, FindValidPose_Response)),
  }
};

typedef struct _FindValidPose_Response_type_support_data_t
{
  void * data[2];
} _FindValidPose_Response_type_support_data_t;

static _FindValidPose_Response_type_support_data_t _FindValidPose_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FindValidPose_Response_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_FindValidPose_Response_message_typesupport_ids.typesupport_identifier[0],
  &_FindValidPose_Response_message_typesupport_symbol_names.symbol_name[0],
  &_FindValidPose_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FindValidPose_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FindValidPose_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &mbf_msgs__srv__FindValidPose_Response__get_type_hash,
  &mbf_msgs__srv__FindValidPose_Response__get_type_description,
  &mbf_msgs__srv__FindValidPose_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace mbf_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, mbf_msgs, srv, FindValidPose_Response)() {
  return &::mbf_msgs::srv::rosidl_typesupport_c::FindValidPose_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "mbf_msgs/srv/detail/find_valid_pose__struct.h"
// already included above
// #include "mbf_msgs/srv/detail/find_valid_pose__type_support.h"
// already included above
// #include "mbf_msgs/srv/detail/find_valid_pose__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace mbf_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _FindValidPose_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FindValidPose_Event_type_support_ids_t;

static const _FindValidPose_Event_type_support_ids_t _FindValidPose_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _FindValidPose_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FindValidPose_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FindValidPose_Event_type_support_symbol_names_t _FindValidPose_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mbf_msgs, srv, FindValidPose_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, FindValidPose_Event)),
  }
};

typedef struct _FindValidPose_Event_type_support_data_t
{
  void * data[2];
} _FindValidPose_Event_type_support_data_t;

static _FindValidPose_Event_type_support_data_t _FindValidPose_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FindValidPose_Event_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_FindValidPose_Event_message_typesupport_ids.typesupport_identifier[0],
  &_FindValidPose_Event_message_typesupport_symbol_names.symbol_name[0],
  &_FindValidPose_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t FindValidPose_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FindValidPose_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &mbf_msgs__srv__FindValidPose_Event__get_type_hash,
  &mbf_msgs__srv__FindValidPose_Event__get_type_description,
  &mbf_msgs__srv__FindValidPose_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace mbf_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, mbf_msgs, srv, FindValidPose_Event)() {
  return &::mbf_msgs::srv::rosidl_typesupport_c::FindValidPose_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "mbf_msgs/srv/detail/find_valid_pose__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
#include "service_msgs/msg/service_event_info.h"
#include "builtin_interfaces/msg/time.h"

namespace mbf_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{
typedef struct _FindValidPose_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _FindValidPose_type_support_ids_t;

static const _FindValidPose_type_support_ids_t _FindValidPose_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _FindValidPose_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _FindValidPose_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _FindValidPose_type_support_symbol_names_t _FindValidPose_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mbf_msgs, srv, FindValidPose)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mbf_msgs, srv, FindValidPose)),
  }
};

typedef struct _FindValidPose_type_support_data_t
{
  void * data[2];
} _FindValidPose_type_support_data_t;

static _FindValidPose_type_support_data_t _FindValidPose_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _FindValidPose_service_typesupport_map = {
  2,
  "mbf_msgs",
  &_FindValidPose_service_typesupport_ids.typesupport_identifier[0],
  &_FindValidPose_service_typesupport_symbol_names.symbol_name[0],
  &_FindValidPose_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t FindValidPose_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_FindValidPose_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &FindValidPose_Request_message_type_support_handle,
  &FindValidPose_Response_message_type_support_handle,
  &FindValidPose_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    mbf_msgs,
    srv,
    FindValidPose
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    mbf_msgs,
    srv,
    FindValidPose
  ),
  &mbf_msgs__srv__FindValidPose__get_type_hash,
  &mbf_msgs__srv__FindValidPose__get_type_description,
  &mbf_msgs__srv__FindValidPose__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace mbf_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, mbf_msgs, srv, FindValidPose)() {
  return &::mbf_msgs::srv::rosidl_typesupport_c::FindValidPose_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
