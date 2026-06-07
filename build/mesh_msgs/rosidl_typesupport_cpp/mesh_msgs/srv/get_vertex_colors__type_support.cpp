// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from mesh_msgs:srv/GetVertexColors.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "mesh_msgs/srv/detail/get_vertex_colors__functions.h"
#include "mesh_msgs/srv/detail/get_vertex_colors__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace mesh_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetVertexColors_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetVertexColors_Request_type_support_ids_t;

static const _GetVertexColors_Request_type_support_ids_t _GetVertexColors_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetVertexColors_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetVertexColors_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetVertexColors_Request_type_support_symbol_names_t _GetVertexColors_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mesh_msgs, srv, GetVertexColors_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mesh_msgs, srv, GetVertexColors_Request)),
  }
};

typedef struct _GetVertexColors_Request_type_support_data_t
{
  void * data[2];
} _GetVertexColors_Request_type_support_data_t;

static _GetVertexColors_Request_type_support_data_t _GetVertexColors_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetVertexColors_Request_message_typesupport_map = {
  2,
  "mesh_msgs",
  &_GetVertexColors_Request_message_typesupport_ids.typesupport_identifier[0],
  &_GetVertexColors_Request_message_typesupport_symbol_names.symbol_name[0],
  &_GetVertexColors_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetVertexColors_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetVertexColors_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mesh_msgs__srv__GetVertexColors_Request__get_type_hash,
  &mesh_msgs__srv__GetVertexColors_Request__get_type_description,
  &mesh_msgs__srv__GetVertexColors_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace mesh_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mesh_msgs::srv::GetVertexColors_Request>()
{
  return &::mesh_msgs::srv::rosidl_typesupport_cpp::GetVertexColors_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mesh_msgs, srv, GetVertexColors_Request)() {
  return get_message_type_support_handle<mesh_msgs::srv::GetVertexColors_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "mesh_msgs/srv/detail/get_vertex_colors__functions.h"
// already included above
// #include "mesh_msgs/srv/detail/get_vertex_colors__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace mesh_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetVertexColors_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetVertexColors_Response_type_support_ids_t;

static const _GetVertexColors_Response_type_support_ids_t _GetVertexColors_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetVertexColors_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetVertexColors_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetVertexColors_Response_type_support_symbol_names_t _GetVertexColors_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mesh_msgs, srv, GetVertexColors_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mesh_msgs, srv, GetVertexColors_Response)),
  }
};

typedef struct _GetVertexColors_Response_type_support_data_t
{
  void * data[2];
} _GetVertexColors_Response_type_support_data_t;

static _GetVertexColors_Response_type_support_data_t _GetVertexColors_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetVertexColors_Response_message_typesupport_map = {
  2,
  "mesh_msgs",
  &_GetVertexColors_Response_message_typesupport_ids.typesupport_identifier[0],
  &_GetVertexColors_Response_message_typesupport_symbol_names.symbol_name[0],
  &_GetVertexColors_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetVertexColors_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetVertexColors_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mesh_msgs__srv__GetVertexColors_Response__get_type_hash,
  &mesh_msgs__srv__GetVertexColors_Response__get_type_description,
  &mesh_msgs__srv__GetVertexColors_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace mesh_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mesh_msgs::srv::GetVertexColors_Response>()
{
  return &::mesh_msgs::srv::rosidl_typesupport_cpp::GetVertexColors_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mesh_msgs, srv, GetVertexColors_Response)() {
  return get_message_type_support_handle<mesh_msgs::srv::GetVertexColors_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "mesh_msgs/srv/detail/get_vertex_colors__functions.h"
// already included above
// #include "mesh_msgs/srv/detail/get_vertex_colors__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace mesh_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetVertexColors_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetVertexColors_Event_type_support_ids_t;

static const _GetVertexColors_Event_type_support_ids_t _GetVertexColors_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetVertexColors_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetVertexColors_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetVertexColors_Event_type_support_symbol_names_t _GetVertexColors_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mesh_msgs, srv, GetVertexColors_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mesh_msgs, srv, GetVertexColors_Event)),
  }
};

typedef struct _GetVertexColors_Event_type_support_data_t
{
  void * data[2];
} _GetVertexColors_Event_type_support_data_t;

static _GetVertexColors_Event_type_support_data_t _GetVertexColors_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetVertexColors_Event_message_typesupport_map = {
  2,
  "mesh_msgs",
  &_GetVertexColors_Event_message_typesupport_ids.typesupport_identifier[0],
  &_GetVertexColors_Event_message_typesupport_symbol_names.symbol_name[0],
  &_GetVertexColors_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetVertexColors_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetVertexColors_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mesh_msgs__srv__GetVertexColors_Event__get_type_hash,
  &mesh_msgs__srv__GetVertexColors_Event__get_type_description,
  &mesh_msgs__srv__GetVertexColors_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace mesh_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mesh_msgs::srv::GetVertexColors_Event>()
{
  return &::mesh_msgs::srv::rosidl_typesupport_cpp::GetVertexColors_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mesh_msgs, srv, GetVertexColors_Event)() {
  return get_message_type_support_handle<mesh_msgs::srv::GetVertexColors_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "mesh_msgs/srv/detail/get_vertex_colors__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace mesh_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetVertexColors_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetVertexColors_type_support_ids_t;

static const _GetVertexColors_type_support_ids_t _GetVertexColors_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetVertexColors_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetVertexColors_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetVertexColors_type_support_symbol_names_t _GetVertexColors_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mesh_msgs, srv, GetVertexColors)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mesh_msgs, srv, GetVertexColors)),
  }
};

typedef struct _GetVertexColors_type_support_data_t
{
  void * data[2];
} _GetVertexColors_type_support_data_t;

static _GetVertexColors_type_support_data_t _GetVertexColors_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetVertexColors_service_typesupport_map = {
  2,
  "mesh_msgs",
  &_GetVertexColors_service_typesupport_ids.typesupport_identifier[0],
  &_GetVertexColors_service_typesupport_symbol_names.symbol_name[0],
  &_GetVertexColors_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t GetVertexColors_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetVertexColors_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<mesh_msgs::srv::GetVertexColors_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<mesh_msgs::srv::GetVertexColors_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<mesh_msgs::srv::GetVertexColors_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<mesh_msgs::srv::GetVertexColors>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<mesh_msgs::srv::GetVertexColors>,
  &mesh_msgs__srv__GetVertexColors__get_type_hash,
  &mesh_msgs__srv__GetVertexColors__get_type_description,
  &mesh_msgs__srv__GetVertexColors__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace mesh_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<mesh_msgs::srv::GetVertexColors>()
{
  return &::mesh_msgs::srv::rosidl_typesupport_cpp::GetVertexColors_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, mesh_msgs, srv, GetVertexColors)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<mesh_msgs::srv::GetVertexColors>();
}

#ifdef __cplusplus
}
#endif
