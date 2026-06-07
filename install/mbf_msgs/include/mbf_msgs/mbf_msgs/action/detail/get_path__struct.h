// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mbf_msgs:action/GetPath.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/action/get_path.h"


#ifndef MBF_MSGS__ACTION__DETAIL__GET_PATH__STRUCT_H_
#define MBF_MSGS__ACTION__DETAIL__GET_PATH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'start_pose'
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'planner'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/GetPath in the package mbf_msgs.
typedef struct mbf_msgs__action__GetPath_Goal
{
  /// Use start_pose or current position as the beginning of the path
  bool use_start_pose;
  /// The start pose for the path; optional, used if use_start_pose is true
  geometry_msgs__msg__PoseStamped start_pose;
  /// The pose to achieve with the path
  geometry_msgs__msg__PoseStamped target_pose;
  /// If the goal is obstructed, how many meters the planner can relax the constraint in x and y before failing
  double tolerance;
  /// Planner to use; defaults to the first one specified on "planners" parameter
  rosidl_runtime_c__String planner;
  /// use different slots for concurrency
  uint8_t concurrency_slot;
} mbf_msgs__action__GetPath_Goal;

// Struct for a sequence of mbf_msgs__action__GetPath_Goal.
typedef struct mbf_msgs__action__GetPath_Goal__Sequence
{
  mbf_msgs__action__GetPath_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__GetPath_Goal__Sequence;

// Constants defined in the message

/// Constant 'SUCCESS'.
/**
  * Predefined success codes:
 */
enum
{
  mbf_msgs__action__GetPath_Result__SUCCESS = 0
};

/// Constant 'FAILURE'.
/**
  * 1..9 are reserved as plugin specific non-error results
  * Possible error codes:
  * Unspecified failure, only used for old, non-mfb_core based plugins
 */
enum
{
  mbf_msgs__action__GetPath_Result__FAILURE = 50
};

/// Constant 'CANCELED'.
/**
  * The action has been canceled by a action client
 */
enum
{
  mbf_msgs__action__GetPath_Result__CANCELED = 51
};

/// Constant 'INVALID_START'.
/**
  * The start pose is inconsistent (e.g. frame is not valid)
 */
enum
{
  mbf_msgs__action__GetPath_Result__INVALID_START = 52
};

/// Constant 'INVALID_GOAL'.
/**
  * The goal pose is inconsistent (e.g. frame is not valid)
 */
enum
{
  mbf_msgs__action__GetPath_Result__INVALID_GOAL = 53
};

/// Constant 'BLOCKED_START'.
/**
  * The start pose is in collision
 */
enum
{
  mbf_msgs__action__GetPath_Result__BLOCKED_START = 54
};

/// Constant 'BLOCKED_GOAL'.
/**
  * The goal pose is in collision
 */
enum
{
  mbf_msgs__action__GetPath_Result__BLOCKED_GOAL = 55
};

/// Constant 'NO_PATH_FOUND'.
enum
{
  mbf_msgs__action__GetPath_Result__NO_PATH_FOUND = 56
};

/// Constant 'PAT_EXCEEDED'.
enum
{
  mbf_msgs__action__GetPath_Result__PAT_EXCEEDED = 57
};

/// Constant 'EMPTY_PATH'.
enum
{
  mbf_msgs__action__GetPath_Result__EMPTY_PATH = 58
};

/// Constant 'TF_ERROR'.
enum
{
  mbf_msgs__action__GetPath_Result__TF_ERROR = 59
};

/// Constant 'NOT_INITIALIZED'.
enum
{
  mbf_msgs__action__GetPath_Result__NOT_INITIALIZED = 60
};

/// Constant 'INVALID_PLUGIN'.
enum
{
  mbf_msgs__action__GetPath_Result__INVALID_PLUGIN = 61
};

/// Constant 'INTERNAL_ERROR'.
enum
{
  mbf_msgs__action__GetPath_Result__INTERNAL_ERROR = 62
};

/// Constant 'OUT_OF_MAP'.
/**
  * The start and / or the goal are outside the map
 */
enum
{
  mbf_msgs__action__GetPath_Result__OUT_OF_MAP = 63
};

/// Constant 'MAP_ERROR'.
/**
  * The map is not available or not running properly
 */
enum
{
  mbf_msgs__action__GetPath_Result__MAP_ERROR = 64
};

/// Constant 'STOPPED'.
/**
  * The planner execution has been stopped rigorously
 */
enum
{
  mbf_msgs__action__GetPath_Result__STOPPED = 65
};

/// Constant 'ERROR_RANGE_START'.
enum
{
  mbf_msgs__action__GetPath_Result__ERROR_RANGE_START = 50
};

/// Constant 'ERROR_RANGE_END'.
enum
{
  mbf_msgs__action__GetPath_Result__ERROR_RANGE_END = 99
};

/// Constant 'PLUGIN_ERROR_RANGE_START'.
/**
  * 71..99 are reserved as plugin specific errors:
 */
enum
{
  mbf_msgs__action__GetPath_Result__PLUGIN_ERROR_RANGE_START = 71
};

/// Constant 'PLUGIN_ERROR_RANGE_END'.
enum
{
  mbf_msgs__action__GetPath_Result__PLUGIN_ERROR_RANGE_END = 99
};

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'path'
#include "nav_msgs/msg/detail/path__struct.h"

/// Struct defined in action/GetPath in the package mbf_msgs.
typedef struct mbf_msgs__action__GetPath_Result
{
  uint32_t outcome;
  rosidl_runtime_c__String message;
  nav_msgs__msg__Path path;
  double cost;
} mbf_msgs__action__GetPath_Result;

// Struct for a sequence of mbf_msgs__action__GetPath_Result.
typedef struct mbf_msgs__action__GetPath_Result__Sequence
{
  mbf_msgs__action__GetPath_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__GetPath_Result__Sequence;

// Constants defined in the message

/// Struct defined in action/GetPath in the package mbf_msgs.
typedef struct mbf_msgs__action__GetPath_Feedback
{
  uint8_t structure_needs_at_least_one_member;
} mbf_msgs__action__GetPath_Feedback;

// Struct for a sequence of mbf_msgs__action__GetPath_Feedback.
typedef struct mbf_msgs__action__GetPath_Feedback__Sequence
{
  mbf_msgs__action__GetPath_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__GetPath_Feedback__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "mbf_msgs/action/detail/get_path__struct.h"

/// Struct defined in action/GetPath in the package mbf_msgs.
typedef struct mbf_msgs__action__GetPath_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  mbf_msgs__action__GetPath_Goal goal;
} mbf_msgs__action__GetPath_SendGoal_Request;

// Struct for a sequence of mbf_msgs__action__GetPath_SendGoal_Request.
typedef struct mbf_msgs__action__GetPath_SendGoal_Request__Sequence
{
  mbf_msgs__action__GetPath_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__GetPath_SendGoal_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/GetPath in the package mbf_msgs.
typedef struct mbf_msgs__action__GetPath_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} mbf_msgs__action__GetPath_SendGoal_Response;

// Struct for a sequence of mbf_msgs__action__GetPath_SendGoal_Response.
typedef struct mbf_msgs__action__GetPath_SendGoal_Response__Sequence
{
  mbf_msgs__action__GetPath_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__GetPath_SendGoal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  mbf_msgs__action__GetPath_SendGoal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  mbf_msgs__action__GetPath_SendGoal_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/GetPath in the package mbf_msgs.
typedef struct mbf_msgs__action__GetPath_SendGoal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  mbf_msgs__action__GetPath_SendGoal_Request__Sequence request;
  mbf_msgs__action__GetPath_SendGoal_Response__Sequence response;
} mbf_msgs__action__GetPath_SendGoal_Event;

// Struct for a sequence of mbf_msgs__action__GetPath_SendGoal_Event.
typedef struct mbf_msgs__action__GetPath_SendGoal_Event__Sequence
{
  mbf_msgs__action__GetPath_SendGoal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__GetPath_SendGoal_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/GetPath in the package mbf_msgs.
typedef struct mbf_msgs__action__GetPath_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} mbf_msgs__action__GetPath_GetResult_Request;

// Struct for a sequence of mbf_msgs__action__GetPath_GetResult_Request.
typedef struct mbf_msgs__action__GetPath_GetResult_Request__Sequence
{
  mbf_msgs__action__GetPath_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__GetPath_GetResult_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "mbf_msgs/action/detail/get_path__struct.h"

/// Struct defined in action/GetPath in the package mbf_msgs.
typedef struct mbf_msgs__action__GetPath_GetResult_Response
{
  int8_t status;
  mbf_msgs__action__GetPath_Result result;
} mbf_msgs__action__GetPath_GetResult_Response;

// Struct for a sequence of mbf_msgs__action__GetPath_GetResult_Response.
typedef struct mbf_msgs__action__GetPath_GetResult_Response__Sequence
{
  mbf_msgs__action__GetPath_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__GetPath_GetResult_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  mbf_msgs__action__GetPath_GetResult_Event__request__MAX_SIZE = 1
};
// response
enum
{
  mbf_msgs__action__GetPath_GetResult_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/GetPath in the package mbf_msgs.
typedef struct mbf_msgs__action__GetPath_GetResult_Event
{
  service_msgs__msg__ServiceEventInfo info;
  mbf_msgs__action__GetPath_GetResult_Request__Sequence request;
  mbf_msgs__action__GetPath_GetResult_Response__Sequence response;
} mbf_msgs__action__GetPath_GetResult_Event;

// Struct for a sequence of mbf_msgs__action__GetPath_GetResult_Event.
typedef struct mbf_msgs__action__GetPath_GetResult_Event__Sequence
{
  mbf_msgs__action__GetPath_GetResult_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__GetPath_GetResult_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "mbf_msgs/action/detail/get_path__struct.h"

/// Struct defined in action/GetPath in the package mbf_msgs.
typedef struct mbf_msgs__action__GetPath_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  mbf_msgs__action__GetPath_Feedback feedback;
} mbf_msgs__action__GetPath_FeedbackMessage;

// Struct for a sequence of mbf_msgs__action__GetPath_FeedbackMessage.
typedef struct mbf_msgs__action__GetPath_FeedbackMessage__Sequence
{
  mbf_msgs__action__GetPath_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__GetPath_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MBF_MSGS__ACTION__DETAIL__GET_PATH__STRUCT_H_
