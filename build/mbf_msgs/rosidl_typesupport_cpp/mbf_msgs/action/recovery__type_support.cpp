// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from mbf_msgs:action/Recovery.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "mbf_msgs/action/detail/recovery__functions.h"
#include "mbf_msgs/action/detail/recovery__struct.hpp"
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

typedef struct _Recovery_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Recovery_Goal_type_support_ids_t;

static const _Recovery_Goal_type_support_ids_t _Recovery_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Recovery_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Recovery_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Recovery_Goal_type_support_symbol_names_t _Recovery_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, Recovery_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, Recovery_Goal)),
  }
};

typedef struct _Recovery_Goal_type_support_data_t
{
  void * data[2];
} _Recovery_Goal_type_support_data_t;

static _Recovery_Goal_type_support_data_t _Recovery_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Recovery_Goal_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_Recovery_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_Recovery_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_Recovery_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Recovery_Goal_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Recovery_Goal_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__action__Recovery_Goal__get_type_hash,
  &mbf_msgs__action__Recovery_Goal__get_type_description,
  &mbf_msgs__action__Recovery_Goal__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::action::Recovery_Goal>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::Recovery_Goal_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, Recovery_Goal)() {
  return get_message_type_support_handle<mbf_msgs::action::Recovery_Goal>();
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
// #include "mbf_msgs/action/detail/recovery__functions.h"
// already included above
// #include "mbf_msgs/action/detail/recovery__struct.hpp"
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

typedef struct _Recovery_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Recovery_Result_type_support_ids_t;

static const _Recovery_Result_type_support_ids_t _Recovery_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Recovery_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Recovery_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Recovery_Result_type_support_symbol_names_t _Recovery_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, Recovery_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, Recovery_Result)),
  }
};

typedef struct _Recovery_Result_type_support_data_t
{
  void * data[2];
} _Recovery_Result_type_support_data_t;

static _Recovery_Result_type_support_data_t _Recovery_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Recovery_Result_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_Recovery_Result_message_typesupport_ids.typesupport_identifier[0],
  &_Recovery_Result_message_typesupport_symbol_names.symbol_name[0],
  &_Recovery_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Recovery_Result_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Recovery_Result_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__action__Recovery_Result__get_type_hash,
  &mbf_msgs__action__Recovery_Result__get_type_description,
  &mbf_msgs__action__Recovery_Result__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::action::Recovery_Result>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::Recovery_Result_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, Recovery_Result)() {
  return get_message_type_support_handle<mbf_msgs::action::Recovery_Result>();
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
// #include "mbf_msgs/action/detail/recovery__functions.h"
// already included above
// #include "mbf_msgs/action/detail/recovery__struct.hpp"
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

typedef struct _Recovery_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Recovery_Feedback_type_support_ids_t;

static const _Recovery_Feedback_type_support_ids_t _Recovery_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Recovery_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Recovery_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Recovery_Feedback_type_support_symbol_names_t _Recovery_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, Recovery_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, Recovery_Feedback)),
  }
};

typedef struct _Recovery_Feedback_type_support_data_t
{
  void * data[2];
} _Recovery_Feedback_type_support_data_t;

static _Recovery_Feedback_type_support_data_t _Recovery_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Recovery_Feedback_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_Recovery_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_Recovery_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_Recovery_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Recovery_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Recovery_Feedback_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__action__Recovery_Feedback__get_type_hash,
  &mbf_msgs__action__Recovery_Feedback__get_type_description,
  &mbf_msgs__action__Recovery_Feedback__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::action::Recovery_Feedback>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::Recovery_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, Recovery_Feedback)() {
  return get_message_type_support_handle<mbf_msgs::action::Recovery_Feedback>();
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
// #include "mbf_msgs/action/detail/recovery__functions.h"
// already included above
// #include "mbf_msgs/action/detail/recovery__struct.hpp"
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

typedef struct _Recovery_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Recovery_SendGoal_Request_type_support_ids_t;

static const _Recovery_SendGoal_Request_type_support_ids_t _Recovery_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Recovery_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Recovery_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Recovery_SendGoal_Request_type_support_symbol_names_t _Recovery_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, Recovery_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, Recovery_SendGoal_Request)),
  }
};

typedef struct _Recovery_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _Recovery_SendGoal_Request_type_support_data_t;

static _Recovery_SendGoal_Request_type_support_data_t _Recovery_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Recovery_SendGoal_Request_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_Recovery_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Recovery_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Recovery_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Recovery_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Recovery_SendGoal_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__action__Recovery_SendGoal_Request__get_type_hash,
  &mbf_msgs__action__Recovery_SendGoal_Request__get_type_description,
  &mbf_msgs__action__Recovery_SendGoal_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::action::Recovery_SendGoal_Request>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::Recovery_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, Recovery_SendGoal_Request)() {
  return get_message_type_support_handle<mbf_msgs::action::Recovery_SendGoal_Request>();
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
// #include "mbf_msgs/action/detail/recovery__functions.h"
// already included above
// #include "mbf_msgs/action/detail/recovery__struct.hpp"
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

typedef struct _Recovery_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Recovery_SendGoal_Response_type_support_ids_t;

static const _Recovery_SendGoal_Response_type_support_ids_t _Recovery_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Recovery_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Recovery_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Recovery_SendGoal_Response_type_support_symbol_names_t _Recovery_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, Recovery_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, Recovery_SendGoal_Response)),
  }
};

typedef struct _Recovery_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _Recovery_SendGoal_Response_type_support_data_t;

static _Recovery_SendGoal_Response_type_support_data_t _Recovery_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Recovery_SendGoal_Response_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_Recovery_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Recovery_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Recovery_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Recovery_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Recovery_SendGoal_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__action__Recovery_SendGoal_Response__get_type_hash,
  &mbf_msgs__action__Recovery_SendGoal_Response__get_type_description,
  &mbf_msgs__action__Recovery_SendGoal_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::action::Recovery_SendGoal_Response>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::Recovery_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, Recovery_SendGoal_Response)() {
  return get_message_type_support_handle<mbf_msgs::action::Recovery_SendGoal_Response>();
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
// #include "mbf_msgs/action/detail/recovery__functions.h"
// already included above
// #include "mbf_msgs/action/detail/recovery__struct.hpp"
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

typedef struct _Recovery_SendGoal_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Recovery_SendGoal_Event_type_support_ids_t;

static const _Recovery_SendGoal_Event_type_support_ids_t _Recovery_SendGoal_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Recovery_SendGoal_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Recovery_SendGoal_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Recovery_SendGoal_Event_type_support_symbol_names_t _Recovery_SendGoal_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, Recovery_SendGoal_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, Recovery_SendGoal_Event)),
  }
};

typedef struct _Recovery_SendGoal_Event_type_support_data_t
{
  void * data[2];
} _Recovery_SendGoal_Event_type_support_data_t;

static _Recovery_SendGoal_Event_type_support_data_t _Recovery_SendGoal_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Recovery_SendGoal_Event_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_Recovery_SendGoal_Event_message_typesupport_ids.typesupport_identifier[0],
  &_Recovery_SendGoal_Event_message_typesupport_symbol_names.symbol_name[0],
  &_Recovery_SendGoal_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Recovery_SendGoal_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Recovery_SendGoal_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__action__Recovery_SendGoal_Event__get_type_hash,
  &mbf_msgs__action__Recovery_SendGoal_Event__get_type_description,
  &mbf_msgs__action__Recovery_SendGoal_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::action::Recovery_SendGoal_Event>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::Recovery_SendGoal_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, Recovery_SendGoal_Event)() {
  return get_message_type_support_handle<mbf_msgs::action::Recovery_SendGoal_Event>();
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
// #include "mbf_msgs/action/detail/recovery__struct.hpp"
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

typedef struct _Recovery_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Recovery_SendGoal_type_support_ids_t;

static const _Recovery_SendGoal_type_support_ids_t _Recovery_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Recovery_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Recovery_SendGoal_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Recovery_SendGoal_type_support_symbol_names_t _Recovery_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, Recovery_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, Recovery_SendGoal)),
  }
};

typedef struct _Recovery_SendGoal_type_support_data_t
{
  void * data[2];
} _Recovery_SendGoal_type_support_data_t;

static _Recovery_SendGoal_type_support_data_t _Recovery_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Recovery_SendGoal_service_typesupport_map = {
  2,
  "mbf_msgs",
  &_Recovery_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_Recovery_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_Recovery_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Recovery_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Recovery_SendGoal_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<mbf_msgs::action::Recovery_SendGoal_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<mbf_msgs::action::Recovery_SendGoal_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<mbf_msgs::action::Recovery_SendGoal_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<mbf_msgs::action::Recovery_SendGoal>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<mbf_msgs::action::Recovery_SendGoal>,
  &mbf_msgs__action__Recovery_SendGoal__get_type_hash,
  &mbf_msgs__action__Recovery_SendGoal__get_type_description,
  &mbf_msgs__action__Recovery_SendGoal__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<mbf_msgs::action::Recovery_SendGoal>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::Recovery_SendGoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, Recovery_SendGoal)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<mbf_msgs::action::Recovery_SendGoal>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "mbf_msgs/action/detail/recovery__functions.h"
// already included above
// #include "mbf_msgs/action/detail/recovery__struct.hpp"
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

typedef struct _Recovery_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Recovery_GetResult_Request_type_support_ids_t;

static const _Recovery_GetResult_Request_type_support_ids_t _Recovery_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Recovery_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Recovery_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Recovery_GetResult_Request_type_support_symbol_names_t _Recovery_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, Recovery_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, Recovery_GetResult_Request)),
  }
};

typedef struct _Recovery_GetResult_Request_type_support_data_t
{
  void * data[2];
} _Recovery_GetResult_Request_type_support_data_t;

static _Recovery_GetResult_Request_type_support_data_t _Recovery_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Recovery_GetResult_Request_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_Recovery_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Recovery_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Recovery_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Recovery_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Recovery_GetResult_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__action__Recovery_GetResult_Request__get_type_hash,
  &mbf_msgs__action__Recovery_GetResult_Request__get_type_description,
  &mbf_msgs__action__Recovery_GetResult_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::action::Recovery_GetResult_Request>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::Recovery_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, Recovery_GetResult_Request)() {
  return get_message_type_support_handle<mbf_msgs::action::Recovery_GetResult_Request>();
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
// #include "mbf_msgs/action/detail/recovery__functions.h"
// already included above
// #include "mbf_msgs/action/detail/recovery__struct.hpp"
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

typedef struct _Recovery_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Recovery_GetResult_Response_type_support_ids_t;

static const _Recovery_GetResult_Response_type_support_ids_t _Recovery_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Recovery_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Recovery_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Recovery_GetResult_Response_type_support_symbol_names_t _Recovery_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, Recovery_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, Recovery_GetResult_Response)),
  }
};

typedef struct _Recovery_GetResult_Response_type_support_data_t
{
  void * data[2];
} _Recovery_GetResult_Response_type_support_data_t;

static _Recovery_GetResult_Response_type_support_data_t _Recovery_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Recovery_GetResult_Response_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_Recovery_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Recovery_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Recovery_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Recovery_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Recovery_GetResult_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__action__Recovery_GetResult_Response__get_type_hash,
  &mbf_msgs__action__Recovery_GetResult_Response__get_type_description,
  &mbf_msgs__action__Recovery_GetResult_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::action::Recovery_GetResult_Response>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::Recovery_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, Recovery_GetResult_Response)() {
  return get_message_type_support_handle<mbf_msgs::action::Recovery_GetResult_Response>();
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
// #include "mbf_msgs/action/detail/recovery__functions.h"
// already included above
// #include "mbf_msgs/action/detail/recovery__struct.hpp"
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

typedef struct _Recovery_GetResult_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Recovery_GetResult_Event_type_support_ids_t;

static const _Recovery_GetResult_Event_type_support_ids_t _Recovery_GetResult_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Recovery_GetResult_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Recovery_GetResult_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Recovery_GetResult_Event_type_support_symbol_names_t _Recovery_GetResult_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, Recovery_GetResult_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, Recovery_GetResult_Event)),
  }
};

typedef struct _Recovery_GetResult_Event_type_support_data_t
{
  void * data[2];
} _Recovery_GetResult_Event_type_support_data_t;

static _Recovery_GetResult_Event_type_support_data_t _Recovery_GetResult_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Recovery_GetResult_Event_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_Recovery_GetResult_Event_message_typesupport_ids.typesupport_identifier[0],
  &_Recovery_GetResult_Event_message_typesupport_symbol_names.symbol_name[0],
  &_Recovery_GetResult_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Recovery_GetResult_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Recovery_GetResult_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__action__Recovery_GetResult_Event__get_type_hash,
  &mbf_msgs__action__Recovery_GetResult_Event__get_type_description,
  &mbf_msgs__action__Recovery_GetResult_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::action::Recovery_GetResult_Event>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::Recovery_GetResult_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, Recovery_GetResult_Event)() {
  return get_message_type_support_handle<mbf_msgs::action::Recovery_GetResult_Event>();
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
// #include "mbf_msgs/action/detail/recovery__struct.hpp"
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

typedef struct _Recovery_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Recovery_GetResult_type_support_ids_t;

static const _Recovery_GetResult_type_support_ids_t _Recovery_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Recovery_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Recovery_GetResult_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Recovery_GetResult_type_support_symbol_names_t _Recovery_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, Recovery_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, Recovery_GetResult)),
  }
};

typedef struct _Recovery_GetResult_type_support_data_t
{
  void * data[2];
} _Recovery_GetResult_type_support_data_t;

static _Recovery_GetResult_type_support_data_t _Recovery_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Recovery_GetResult_service_typesupport_map = {
  2,
  "mbf_msgs",
  &_Recovery_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_Recovery_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_Recovery_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Recovery_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Recovery_GetResult_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<mbf_msgs::action::Recovery_GetResult_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<mbf_msgs::action::Recovery_GetResult_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<mbf_msgs::action::Recovery_GetResult_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<mbf_msgs::action::Recovery_GetResult>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<mbf_msgs::action::Recovery_GetResult>,
  &mbf_msgs__action__Recovery_GetResult__get_type_hash,
  &mbf_msgs__action__Recovery_GetResult__get_type_description,
  &mbf_msgs__action__Recovery_GetResult__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<mbf_msgs::action::Recovery_GetResult>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::Recovery_GetResult_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, Recovery_GetResult)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<mbf_msgs::action::Recovery_GetResult>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "mbf_msgs/action/detail/recovery__functions.h"
// already included above
// #include "mbf_msgs/action/detail/recovery__struct.hpp"
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

typedef struct _Recovery_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Recovery_FeedbackMessage_type_support_ids_t;

static const _Recovery_FeedbackMessage_type_support_ids_t _Recovery_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Recovery_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Recovery_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Recovery_FeedbackMessage_type_support_symbol_names_t _Recovery_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbf_msgs, action, Recovery_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mbf_msgs, action, Recovery_FeedbackMessage)),
  }
};

typedef struct _Recovery_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _Recovery_FeedbackMessage_type_support_data_t;

static _Recovery_FeedbackMessage_type_support_data_t _Recovery_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Recovery_FeedbackMessage_message_typesupport_map = {
  2,
  "mbf_msgs",
  &_Recovery_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_Recovery_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_Recovery_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Recovery_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Recovery_FeedbackMessage_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &mbf_msgs__action__Recovery_FeedbackMessage__get_type_hash,
  &mbf_msgs__action__Recovery_FeedbackMessage__get_type_description,
  &mbf_msgs__action__Recovery_FeedbackMessage__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mbf_msgs::action::Recovery_FeedbackMessage>()
{
  return &::mbf_msgs::action::rosidl_typesupport_cpp::Recovery_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, Recovery_FeedbackMessage)() {
  return get_message_type_support_handle<mbf_msgs::action::Recovery_FeedbackMessage>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
// already included above
// #include "mbf_msgs/action/detail/recovery__struct.hpp"
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

static rosidl_action_type_support_t Recovery_action_type_support_handle = {
  NULL, NULL, NULL, NULL, NULL,
  &mbf_msgs__action__Recovery__get_type_hash,
  &mbf_msgs__action__Recovery__get_type_description,
  &mbf_msgs__action__Recovery__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace mbf_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
get_action_type_support_handle<mbf_msgs::action::Recovery>()
{
  using ::mbf_msgs::action::rosidl_typesupport_cpp::Recovery_action_type_support_handle;
  // Thread-safe by always writing the same values to the static struct
  Recovery_action_type_support_handle.goal_service_type_support = get_service_type_support_handle<::mbf_msgs::action::Recovery::Impl::SendGoalService>();
  Recovery_action_type_support_handle.result_service_type_support = get_service_type_support_handle<::mbf_msgs::action::Recovery::Impl::GetResultService>();
  Recovery_action_type_support_handle.cancel_service_type_support = get_service_type_support_handle<::mbf_msgs::action::Recovery::Impl::CancelGoalService>();
  Recovery_action_type_support_handle.feedback_message_type_support = get_message_type_support_handle<::mbf_msgs::action::Recovery::Impl::FeedbackMessage>();
  Recovery_action_type_support_handle.status_message_type_support = get_message_type_support_handle<::mbf_msgs::action::Recovery::Impl::GoalStatusMessage>();
  return &Recovery_action_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(rosidl_typesupport_cpp, mbf_msgs, action, Recovery)() {
  return ::rosidl_typesupport_cpp::get_action_type_support_handle<mbf_msgs::action::Recovery>();
}

#ifdef __cplusplus
}
#endif
