// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from mbf_msgs:srv/CheckPoint.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "mbf_msgs/srv/detail/check_point__functions.h"
#include "mbf_msgs/srv/detail/check_point__struct.hpp"
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

typedef struct _CheckPoint_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CheckPoint_Request_type_support_ids_t;

static const _CheckPoint_Request_type_support_ids_t _CheckPoint_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CheckPoint_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CheckPoint_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CheckPoint_Request_type_support_symbol_names_t _CheckPoint_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, srv, CheckPoint_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, srv, CheckPoint_Request)),
  }
};

typedef struct _CheckPoint_Request_type_support_data_t
{
  void * data[2];
} _CheckPoint_Request_type_support_data_t;

static _CheckPoint_Request_type_support_data_t _CheckPoint_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CheckPoint_Request_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_CheckPoint_Request_message_typesupport_ids.typesupport_identifier[0],
  &_CheckPoint_Request_message_typesupport_symbol_names.symbol_name[0],
  &_CheckPoint_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CheckPoint_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CheckPoint_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__srv__CheckPoint_Request__get_type_hash,
  &mbf_msgs__srv__CheckPoint_Request__get_type_description,
  &mbf_msgs__srv__CheckPoint_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::srv::CheckPoint_Request>()
{
  return &::mbf_msgs::srv::rosidl_typesupport_cpp::CheckPoint_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, srv, CheckPoint_Request)() {
  return get_message_type_support_handle<mbf_msgs::srv::CheckPoint_Request>();
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
// #include "mbf_msgs/srv/detail/check_point__functions.h"
// already included above
// #include "mbf_msgs/srv/detail/check_point__struct.hpp"
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

typedef struct _CheckPoint_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CheckPoint_Response_type_support_ids_t;

static const _CheckPoint_Response_type_support_ids_t _CheckPoint_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CheckPoint_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CheckPoint_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CheckPoint_Response_type_support_symbol_names_t _CheckPoint_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, srv, CheckPoint_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, srv, CheckPoint_Response)),
  }
};

typedef struct _CheckPoint_Response_type_support_data_t
{
  void * data[2];
} _CheckPoint_Response_type_support_data_t;

static _CheckPoint_Response_type_support_data_t _CheckPoint_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CheckPoint_Response_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_CheckPoint_Response_message_typesupport_ids.typesupport_identifier[0],
  &_CheckPoint_Response_message_typesupport_symbol_names.symbol_name[0],
  &_CheckPoint_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CheckPoint_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CheckPoint_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__srv__CheckPoint_Response__get_type_hash,
  &mbf_msgs__srv__CheckPoint_Response__get_type_description,
  &mbf_msgs__srv__CheckPoint_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::srv::CheckPoint_Response>()
{
  return &::mbf_msgs::srv::rosidl_typesupport_cpp::CheckPoint_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, srv, CheckPoint_Response)() {
  return get_message_type_support_handle<mbf_msgs::srv::CheckPoint_Response>();
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
// #include "mbf_msgs/srv/detail/check_point__functions.h"
// already included above
// #include "mbf_msgs/srv/detail/check_point__struct.hpp"
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

typedef struct _CheckPoint_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CheckPoint_Event_type_support_ids_t;

static const _CheckPoint_Event_type_support_ids_t _CheckPoint_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CheckPoint_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CheckPoint_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CheckPoint_Event_type_support_symbol_names_t _CheckPoint_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, srv, CheckPoint_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, srv, CheckPoint_Event)),
  }
};

typedef struct _CheckPoint_Event_type_support_data_t
{
  void * data[2];
} _CheckPoint_Event_type_support_data_t;

static _CheckPoint_Event_type_support_data_t _CheckPoint_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CheckPoint_Event_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_CheckPoint_Event_message_typesupport_ids.typesupport_identifier[0],
  &_CheckPoint_Event_message_typesupport_symbol_names.symbol_name[0],
  &_CheckPoint_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t CheckPoint_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CheckPoint_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__srv__CheckPoint_Event__get_type_hash,
  &mbf_msgs__srv__CheckPoint_Event__get_type_description,
  &mbf_msgs__srv__CheckPoint_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::srv::CheckPoint_Event>()
{
  return &::mbf_msgs::srv::rosidl_typesupport_cpp::CheckPoint_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, srv, CheckPoint_Event)() {
  return get_message_type_support_handle<mbf_msgs::srv::CheckPoint_Event>();
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
// #include "mbf_msgs/srv/detail/check_point__struct.hpp"
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

typedef struct _CheckPoint_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _CheckPoint_type_support_ids_t;

static const _CheckPoint_type_support_ids_t _CheckPoint_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _CheckPoint_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _CheckPoint_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _CheckPoint_type_support_symbol_names_t _CheckPoint_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, srv, CheckPoint)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, srv, CheckPoint)),
  }
};

typedef struct _CheckPoint_type_support_data_t
{
  void * data[2];
} _CheckPoint_type_support_data_t;

static _CheckPoint_type_support_data_t _CheckPoint_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _CheckPoint_service_typesupport_map = {
  2,
  "mbf_msgs",
  &_CheckPoint_service_typesupport_ids.typesupport_identifier[0],
  &_CheckPoint_service_typesupport_symbol_names.symbol_name[0],
  &_CheckPoint_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t CheckPoint_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_CheckPoint_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<mbf_msgs::srv::CheckPoint_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<mbf_msgs::srv::CheckPoint_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<mbf_msgs::srv::CheckPoint_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<mbf_msgs::srv::CheckPoint>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<mbf_msgs::srv::CheckPoint>,
  &mbf_msgs__srv__CheckPoint__get_type_hash,
  &mbf_msgs__srv__CheckPoint__get_type_description,
  &mbf_msgs__srv__CheckPoint__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<mbf_msgs::srv::CheckPoint>()
{
  return &::mbf_msgs::srv::rosidl_typesupport_cpp::CheckPoint_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, srv, CheckPoint)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<mbf_msgs::srv::CheckPoint>();
}

#ifdef __cplusplus
}
#endif
