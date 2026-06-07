// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from mbf_msgs:srv/CheckPath.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "mbf_msgs/srv/detail/check_path__functions.h"
#include "mbf_msgs/srv/detail/check_path__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace mbf_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _CheckPath_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CheckPath_Request_type_support_ids_t;

static const _CheckPath_Request_type_support_ids_t _CheckPath_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CheckPath_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CheckPath_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CheckPath_Request_type_support_symbol_names_t _CheckPath_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, srv, CheckPath_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, srv, CheckPath_Request)),
  }
};

typedef struct _CheckPath_Request_type_support_data_t
{
  void * data[2];
} _CheckPath_Request_type_support_data_t;

static _CheckPath_Request_type_support_data_t _CheckPath_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CheckPath_Request_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_CheckPath_Request_message_typesupport_ids.typesupport_identifier[0],
  &_CheckPath_Request_message_typesupport_symbol_names.symbol_name[0],
  &_CheckPath_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CheckPath_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CheckPath_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__srv__CheckPath_Request__get_type_hash,
  &mbf_msgs__srv__CheckPath_Request__get_type_description,
  &mbf_msgs__srv__CheckPath_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::srv::CheckPath_Request>()
{
  return &::mbf_msgs::srv::rosidl_typesupport_cpp::CheckPath_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, srv, CheckPath_Request)() {
  return get_message_type_support_handle<mbf_msgs::srv::CheckPath_Request>();
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
// #include "mbf_msgs/srv/detail/check_path__functions.h"
// already included above
// #include "mbf_msgs/srv/detail/check_path__struct.hpp"
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

namespace mbf_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _CheckPath_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CheckPath_Response_type_support_ids_t;

static const _CheckPath_Response_type_support_ids_t _CheckPath_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CheckPath_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CheckPath_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CheckPath_Response_type_support_symbol_names_t _CheckPath_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, srv, CheckPath_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, srv, CheckPath_Response)),
  }
};

typedef struct _CheckPath_Response_type_support_data_t
{
  void * data[2];
} _CheckPath_Response_type_support_data_t;

static _CheckPath_Response_type_support_data_t _CheckPath_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CheckPath_Response_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_CheckPath_Response_message_typesupport_ids.typesupport_identifier[0],
  &_CheckPath_Response_message_typesupport_symbol_names.symbol_name[0],
  &_CheckPath_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CheckPath_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CheckPath_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__srv__CheckPath_Response__get_type_hash,
  &mbf_msgs__srv__CheckPath_Response__get_type_description,
  &mbf_msgs__srv__CheckPath_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::srv::CheckPath_Response>()
{
  return &::mbf_msgs::srv::rosidl_typesupport_cpp::CheckPath_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, srv, CheckPath_Response)() {
  return get_message_type_support_handle<mbf_msgs::srv::CheckPath_Response>();
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
// #include "mbf_msgs/srv/detail/check_path__functions.h"
// already included above
// #include "mbf_msgs/srv/detail/check_path__struct.hpp"
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

namespace mbf_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _CheckPath_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CheckPath_Event_type_support_ids_t;

static const _CheckPath_Event_type_support_ids_t _CheckPath_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CheckPath_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CheckPath_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CheckPath_Event_type_support_symbol_names_t _CheckPath_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, srv, CheckPath_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, srv, CheckPath_Event)),
  }
};

typedef struct _CheckPath_Event_type_support_data_t
{
  void * data[2];
} _CheckPath_Event_type_support_data_t;

static _CheckPath_Event_type_support_data_t _CheckPath_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CheckPath_Event_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_CheckPath_Event_message_typesupport_ids.typesupport_identifier[0],
  &_CheckPath_Event_message_typesupport_symbol_names.symbol_name[0],
  &_CheckPath_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CheckPath_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CheckPath_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__srv__CheckPath_Event__get_type_hash,
  &mbf_msgs__srv__CheckPath_Event__get_type_description,
  &mbf_msgs__srv__CheckPath_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::srv::CheckPath_Event>()
{
  return &::mbf_msgs::srv::rosidl_typesupport_cpp::CheckPath_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, srv, CheckPath_Event)() {
  return get_message_type_support_handle<mbf_msgs::srv::CheckPath_Event>();
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
// #include "mbf_msgs/srv/detail/check_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace mbf_msgs
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _CheckPath_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CheckPath_type_support_ids_t;

static const _CheckPath_type_support_ids_t _CheckPath_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CheckPath_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CheckPath_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CheckPath_type_support_symbol_names_t _CheckPath_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, srv, CheckPath)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, srv, CheckPath)),
  }
};

typedef struct _CheckPath_type_support_data_t
{
  void * data[2];
} _CheckPath_type_support_data_t;

static _CheckPath_type_support_data_t _CheckPath_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CheckPath_service_typesupport_map = {
  2,
  "mbf_msgs",
  &_CheckPath_service_typesupport_ids.typesupport_identifier[0],
  &_CheckPath_service_typesupport_symbol_names.symbol_name[0],
  &_CheckPath_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t CheckPath_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CheckPath_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<mbf_msgs::srv::CheckPath_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<mbf_msgs::srv::CheckPath_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<mbf_msgs::srv::CheckPath_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<mbf_msgs::srv::CheckPath>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<mbf_msgs::srv::CheckPath>,
  &mbf_msgs__srv__CheckPath__get_type_hash,
  &mbf_msgs__srv__CheckPath__get_type_description,
  &mbf_msgs__srv__CheckPath__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<mbf_msgs::srv::CheckPath>()
{
  return &::mbf_msgs::srv::rosidl_typesupport_cpp::CheckPath_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, srv, CheckPath)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<mbf_msgs::srv::CheckPath>();
}

#ifdef __cplusplus
}
#endif
