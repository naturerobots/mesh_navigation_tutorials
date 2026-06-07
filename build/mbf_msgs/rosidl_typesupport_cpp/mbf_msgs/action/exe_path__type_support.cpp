// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from mbf_msgs:action/ExePath.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "mbf_msgs/action/detail/exe_path__functions.h"
#include "mbf_msgs/action/detail/exe_path__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace mbf_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ExePath_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ExePath_Goal_type_support_ids_t;

static const _ExePath_Goal_type_support_ids_t _ExePath_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ExePath_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ExePath_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ExePath_Goal_type_support_symbol_names_t _ExePath_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, ExePath_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, ExePath_Goal)),
  }
};

typedef struct _ExePath_Goal_type_support_data_t
{
  void * data[2];
} _ExePath_Goal_type_support_data_t;

static _ExePath_Goal_type_support_data_t _ExePath_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ExePath_Goal_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_ExePath_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_ExePath_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_ExePath_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ExePath_Goal_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ExePath_Goal_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__action__ExePath_Goal__get_type_hash,
  &mbf_msgs__action__ExePath_Goal__get_type_description,
  &mbf_msgs__action__ExePath_Goal__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::action::ExePath_Goal>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::ExePath_Goal_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, ExePath_Goal)() {
  return get_message_type_support_handle<mbf_msgs::action::ExePath_Goal>();
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
// #include "mbf_msgs/action/detail/exe_path__functions.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.hpp"
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

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ExePath_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ExePath_Result_type_support_ids_t;

static const _ExePath_Result_type_support_ids_t _ExePath_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ExePath_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ExePath_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ExePath_Result_type_support_symbol_names_t _ExePath_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, ExePath_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, ExePath_Result)),
  }
};

typedef struct _ExePath_Result_type_support_data_t
{
  void * data[2];
} _ExePath_Result_type_support_data_t;

static _ExePath_Result_type_support_data_t _ExePath_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ExePath_Result_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_ExePath_Result_message_typesupport_ids.typesupport_identifier[0],
  &_ExePath_Result_message_typesupport_symbol_names.symbol_name[0],
  &_ExePath_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ExePath_Result_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ExePath_Result_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__action__ExePath_Result__get_type_hash,
  &mbf_msgs__action__ExePath_Result__get_type_description,
  &mbf_msgs__action__ExePath_Result__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::action::ExePath_Result>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::ExePath_Result_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, ExePath_Result)() {
  return get_message_type_support_handle<mbf_msgs::action::ExePath_Result>();
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
// #include "mbf_msgs/action/detail/exe_path__functions.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.hpp"
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

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ExePath_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ExePath_Feedback_type_support_ids_t;

static const _ExePath_Feedback_type_support_ids_t _ExePath_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ExePath_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ExePath_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ExePath_Feedback_type_support_symbol_names_t _ExePath_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, ExePath_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, ExePath_Feedback)),
  }
};

typedef struct _ExePath_Feedback_type_support_data_t
{
  void * data[2];
} _ExePath_Feedback_type_support_data_t;

static _ExePath_Feedback_type_support_data_t _ExePath_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ExePath_Feedback_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_ExePath_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_ExePath_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_ExePath_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ExePath_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ExePath_Feedback_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__action__ExePath_Feedback__get_type_hash,
  &mbf_msgs__action__ExePath_Feedback__get_type_description,
  &mbf_msgs__action__ExePath_Feedback__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::action::ExePath_Feedback>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::ExePath_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, ExePath_Feedback)() {
  return get_message_type_support_handle<mbf_msgs::action::ExePath_Feedback>();
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
// #include "mbf_msgs/action/detail/exe_path__functions.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.hpp"
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

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ExePath_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ExePath_SendGoal_Request_type_support_ids_t;

static const _ExePath_SendGoal_Request_type_support_ids_t _ExePath_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ExePath_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ExePath_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ExePath_SendGoal_Request_type_support_symbol_names_t _ExePath_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, ExePath_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, ExePath_SendGoal_Request)),
  }
};

typedef struct _ExePath_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _ExePath_SendGoal_Request_type_support_data_t;

static _ExePath_SendGoal_Request_type_support_data_t _ExePath_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ExePath_SendGoal_Request_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_ExePath_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_ExePath_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_ExePath_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ExePath_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ExePath_SendGoal_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__action__ExePath_SendGoal_Request__get_type_hash,
  &mbf_msgs__action__ExePath_SendGoal_Request__get_type_description,
  &mbf_msgs__action__ExePath_SendGoal_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::action::ExePath_SendGoal_Request>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::ExePath_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, ExePath_SendGoal_Request)() {
  return get_message_type_support_handle<mbf_msgs::action::ExePath_SendGoal_Request>();
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
// #include "mbf_msgs/action/detail/exe_path__functions.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.hpp"
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

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ExePath_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ExePath_SendGoal_Response_type_support_ids_t;

static const _ExePath_SendGoal_Response_type_support_ids_t _ExePath_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ExePath_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ExePath_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ExePath_SendGoal_Response_type_support_symbol_names_t _ExePath_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, ExePath_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, ExePath_SendGoal_Response)),
  }
};

typedef struct _ExePath_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _ExePath_SendGoal_Response_type_support_data_t;

static _ExePath_SendGoal_Response_type_support_data_t _ExePath_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ExePath_SendGoal_Response_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_ExePath_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_ExePath_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_ExePath_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ExePath_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ExePath_SendGoal_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__action__ExePath_SendGoal_Response__get_type_hash,
  &mbf_msgs__action__ExePath_SendGoal_Response__get_type_description,
  &mbf_msgs__action__ExePath_SendGoal_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::action::ExePath_SendGoal_Response>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::ExePath_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, ExePath_SendGoal_Response)() {
  return get_message_type_support_handle<mbf_msgs::action::ExePath_SendGoal_Response>();
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
// #include "mbf_msgs/action/detail/exe_path__functions.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.hpp"
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

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ExePath_SendGoal_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ExePath_SendGoal_Event_type_support_ids_t;

static const _ExePath_SendGoal_Event_type_support_ids_t _ExePath_SendGoal_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ExePath_SendGoal_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ExePath_SendGoal_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ExePath_SendGoal_Event_type_support_symbol_names_t _ExePath_SendGoal_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, ExePath_SendGoal_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, ExePath_SendGoal_Event)),
  }
};

typedef struct _ExePath_SendGoal_Event_type_support_data_t
{
  void * data[2];
} _ExePath_SendGoal_Event_type_support_data_t;

static _ExePath_SendGoal_Event_type_support_data_t _ExePath_SendGoal_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ExePath_SendGoal_Event_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_ExePath_SendGoal_Event_message_typesupport_ids.typesupport_identifier[0],
  &_ExePath_SendGoal_Event_message_typesupport_symbol_names.symbol_name[0],
  &_ExePath_SendGoal_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ExePath_SendGoal_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ExePath_SendGoal_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__action__ExePath_SendGoal_Event__get_type_hash,
  &mbf_msgs__action__ExePath_SendGoal_Event__get_type_description,
  &mbf_msgs__action__ExePath_SendGoal_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::action::ExePath_SendGoal_Event>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::ExePath_SendGoal_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, ExePath_SendGoal_Event)() {
  return get_message_type_support_handle<mbf_msgs::action::ExePath_SendGoal_Event>();
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
// #include "mbf_msgs/action/detail/exe_path__struct.hpp"
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

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ExePath_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ExePath_SendGoal_type_support_ids_t;

static const _ExePath_SendGoal_type_support_ids_t _ExePath_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ExePath_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ExePath_SendGoal_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ExePath_SendGoal_type_support_symbol_names_t _ExePath_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, ExePath_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, ExePath_SendGoal)),
  }
};

typedef struct _ExePath_SendGoal_type_support_data_t
{
  void * data[2];
} _ExePath_SendGoal_type_support_data_t;

static _ExePath_SendGoal_type_support_data_t _ExePath_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ExePath_SendGoal_service_typesupport_map = {
  2,
  "mbf_msgs",
  &_ExePath_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_ExePath_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_ExePath_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t ExePath_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ExePath_SendGoal_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<mbf_msgs::action::ExePath_SendGoal_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<mbf_msgs::action::ExePath_SendGoal_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<mbf_msgs::action::ExePath_SendGoal_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<mbf_msgs::action::ExePath_SendGoal>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<mbf_msgs::action::ExePath_SendGoal>,
  &mbf_msgs__action__ExePath_SendGoal__get_type_hash,
  &mbf_msgs__action__ExePath_SendGoal__get_type_description,
  &mbf_msgs__action__ExePath_SendGoal__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<mbf_msgs::action::ExePath_SendGoal>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::ExePath_SendGoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, ExePath_SendGoal)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<mbf_msgs::action::ExePath_SendGoal>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__functions.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.hpp"
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

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ExePath_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ExePath_GetResult_Request_type_support_ids_t;

static const _ExePath_GetResult_Request_type_support_ids_t _ExePath_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ExePath_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ExePath_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ExePath_GetResult_Request_type_support_symbol_names_t _ExePath_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, ExePath_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, ExePath_GetResult_Request)),
  }
};

typedef struct _ExePath_GetResult_Request_type_support_data_t
{
  void * data[2];
} _ExePath_GetResult_Request_type_support_data_t;

static _ExePath_GetResult_Request_type_support_data_t _ExePath_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ExePath_GetResult_Request_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_ExePath_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_ExePath_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_ExePath_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ExePath_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ExePath_GetResult_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__action__ExePath_GetResult_Request__get_type_hash,
  &mbf_msgs__action__ExePath_GetResult_Request__get_type_description,
  &mbf_msgs__action__ExePath_GetResult_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::action::ExePath_GetResult_Request>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::ExePath_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, ExePath_GetResult_Request)() {
  return get_message_type_support_handle<mbf_msgs::action::ExePath_GetResult_Request>();
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
// #include "mbf_msgs/action/detail/exe_path__functions.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.hpp"
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

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ExePath_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ExePath_GetResult_Response_type_support_ids_t;

static const _ExePath_GetResult_Response_type_support_ids_t _ExePath_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ExePath_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ExePath_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ExePath_GetResult_Response_type_support_symbol_names_t _ExePath_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, ExePath_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, ExePath_GetResult_Response)),
  }
};

typedef struct _ExePath_GetResult_Response_type_support_data_t
{
  void * data[2];
} _ExePath_GetResult_Response_type_support_data_t;

static _ExePath_GetResult_Response_type_support_data_t _ExePath_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ExePath_GetResult_Response_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_ExePath_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_ExePath_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_ExePath_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ExePath_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ExePath_GetResult_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__action__ExePath_GetResult_Response__get_type_hash,
  &mbf_msgs__action__ExePath_GetResult_Response__get_type_description,
  &mbf_msgs__action__ExePath_GetResult_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::action::ExePath_GetResult_Response>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::ExePath_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, ExePath_GetResult_Response)() {
  return get_message_type_support_handle<mbf_msgs::action::ExePath_GetResult_Response>();
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
// #include "mbf_msgs/action/detail/exe_path__functions.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.hpp"
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

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ExePath_GetResult_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ExePath_GetResult_Event_type_support_ids_t;

static const _ExePath_GetResult_Event_type_support_ids_t _ExePath_GetResult_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ExePath_GetResult_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ExePath_GetResult_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ExePath_GetResult_Event_type_support_symbol_names_t _ExePath_GetResult_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, ExePath_GetResult_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, ExePath_GetResult_Event)),
  }
};

typedef struct _ExePath_GetResult_Event_type_support_data_t
{
  void * data[2];
} _ExePath_GetResult_Event_type_support_data_t;

static _ExePath_GetResult_Event_type_support_data_t _ExePath_GetResult_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ExePath_GetResult_Event_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_ExePath_GetResult_Event_message_typesupport_ids.typesupport_identifier[0],
  &_ExePath_GetResult_Event_message_typesupport_symbol_names.symbol_name[0],
  &_ExePath_GetResult_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ExePath_GetResult_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ExePath_GetResult_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__action__ExePath_GetResult_Event__get_type_hash,
  &mbf_msgs__action__ExePath_GetResult_Event__get_type_description,
  &mbf_msgs__action__ExePath_GetResult_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::action::ExePath_GetResult_Event>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::ExePath_GetResult_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, ExePath_GetResult_Event)() {
  return get_message_type_support_handle<mbf_msgs::action::ExePath_GetResult_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace mbf_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ExePath_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ExePath_GetResult_type_support_ids_t;

static const _ExePath_GetResult_type_support_ids_t _ExePath_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ExePath_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ExePath_GetResult_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ExePath_GetResult_type_support_symbol_names_t _ExePath_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, ExePath_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, ExePath_GetResult)),
  }
};

typedef struct _ExePath_GetResult_type_support_data_t
{
  void * data[2];
} _ExePath_GetResult_type_support_data_t;

static _ExePath_GetResult_type_support_data_t _ExePath_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ExePath_GetResult_service_typesupport_map = {
  2,
  "mbf_msgs",
  &_ExePath_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_ExePath_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_ExePath_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t ExePath_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ExePath_GetResult_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<mbf_msgs::action::ExePath_GetResult_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<mbf_msgs::action::ExePath_GetResult_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<mbf_msgs::action::ExePath_GetResult_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<mbf_msgs::action::ExePath_GetResult>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<mbf_msgs::action::ExePath_GetResult>,
  &mbf_msgs__action__ExePath_GetResult__get_type_hash,
  &mbf_msgs__action__ExePath_GetResult__get_type_description,
  &mbf_msgs__action__ExePath_GetResult__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<mbf_msgs::action::ExePath_GetResult>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::ExePath_GetResult_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, ExePath_GetResult)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<mbf_msgs::action::ExePath_GetResult>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__functions.h"
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.hpp"
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

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ExePath_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ExePath_FeedbackMessage_type_support_ids_t;

static const _ExePath_FeedbackMessage_type_support_ids_t _ExePath_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ExePath_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ExePath_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ExePath_FeedbackMessage_type_support_symbol_names_t _ExePath_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, ExePath_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, ExePath_FeedbackMessage)),
  }
};

typedef struct _ExePath_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _ExePath_FeedbackMessage_type_support_data_t;

static _ExePath_FeedbackMessage_type_support_data_t _ExePath_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ExePath_FeedbackMessage_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_ExePath_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_ExePath_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_ExePath_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ExePath_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ExePath_FeedbackMessage_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__action__ExePath_FeedbackMessage__get_type_hash,
  &mbf_msgs__action__ExePath_FeedbackMessage__get_type_description,
  &mbf_msgs__action__ExePath_FeedbackMessage__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::action::ExePath_FeedbackMessage>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::ExePath_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, ExePath_FeedbackMessage)() {
  return get_message_type_support_handle<mbf_msgs::action::ExePath_FeedbackMessage>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"

namespace mbf_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

static rosidl_action_type_support_t ExePath_action_type_support_handle = {
  NULL, NULL, NULL, NULL, NULL,
  &mbf_msgs__action__ExePath__get_type_hash,
  &mbf_msgs__action__ExePath__get_type_description,
  &mbf_msgs__action__ExePath__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
get_action_type_support_handle<mbf_msgs::action::ExePath>()
{
  using ::mbf_msgs::action::rosidl_typesupport_cpp::ExePath_action_type_support_handle;
  // Thread-safe by always writing the same values to the static struct
  ExePath_action_type_support_handle.goal_service_type_support = get_service_type_support_handle<::mbf_msgs::action::ExePath::Impl::SendGoalService>();
  ExePath_action_type_support_handle.result_service_type_support = get_service_type_support_handle<::mbf_msgs::action::ExePath::Impl::GetResultService>();
  ExePath_action_type_support_handle.cancel_service_type_support = get_service_type_support_handle<::mbf_msgs::action::ExePath::Impl::CancelGoalService>();
  ExePath_action_type_support_handle.feedback_message_type_support = get_message_type_support_handle<::mbf_msgs::action::ExePath::Impl::FeedbackMessage>();
  ExePath_action_type_support_handle.status_message_type_support = get_message_type_support_handle<::mbf_msgs::action::ExePath::Impl::GoalStatusMessage>();
  return &ExePath_action_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, ExePath)() {
  return ::rosidl_typesupport_cpp::get_action_type_support_handle<mbf_msgs::action::ExePath>();
}

#ifdef __cplusplus
}
#endif
