// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mbf_msgs:action/ExePath.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/action/exe_path.h"


#ifndef MBF_MSGS__ACTION__DETAIL__EXE_PATH__STRUCT_H_
#define MBF_MSGS__ACTION__DETAIL__EXE_PATH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'path'
#include "nav_msgs/msg/detail/path__struct.h"
// Member 'controller'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/ExePath in the package mbf_msgs.
typedef struct mbf_msgs__action__ExePath_Goal
{
  nav_msgs__msg__Path path;
  /// Controller to use; defaults to the first one specified on "controllers" parameter
  rosidl_runtime_c__String controller;
  /// use different slots for concurrency
  uint8_t concurrency_slot;
  /// define goal tolerance for the action
  bool tolerance_from_action;
  float dist_tolerance;
  float angle_tolerance;
} mbf_msgs__action__ExePath_Goal;

// Struct for a sequence of mbf_msgs__action__ExePath_Goal.
typedef struct mbf_msgs__action__ExePath_Goal__Sequence
{
  mbf_msgs__action__ExePath_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__ExePath_Goal__Sequence;

// Constants defined in the message

/// Constant 'SUCCESS'.
/**
  * Predefined success codes:
 */
enum
{
  mbf_msgs__action__ExePath_Result__SUCCESS = 0
};

/// Constant 'FAILURE'.
/**
  * 1..9 are reserved as plugin specific non-error results
  * Predefined error codes:
  * Unspecified failure, only used for old, non-mfb_core based plugins
 */
enum
{
  mbf_msgs__action__ExePath_Result__FAILURE = 100
};

/// Constant 'CANCELED'.
enum
{
  mbf_msgs__action__ExePath_Result__CANCELED = 101
};

/// Constant 'NO_VALID_CMD'.
enum
{
  mbf_msgs__action__ExePath_Result__NO_VALID_CMD = 102
};

/// Constant 'PAT_EXCEEDED'.
enum
{
  mbf_msgs__action__ExePath_Result__PAT_EXCEEDED = 103
};

/// Constant 'COLLISION'.
enum
{
  mbf_msgs__action__ExePath_Result__COLLISION = 104
};

/// Constant 'OSCILLATION'.
enum
{
  mbf_msgs__action__ExePath_Result__OSCILLATION = 105
};

/// Constant 'ROBOT_STUCK'.
/**
  * The robot is not obeying the commanded velocity
 */
enum
{
  mbf_msgs__action__ExePath_Result__ROBOT_STUCK = 106
};

/// Constant 'MISSED_GOAL'.
/**
  * The robot has overshot the goal and cannot reach it anymore
 */
enum
{
  mbf_msgs__action__ExePath_Result__MISSED_GOAL = 107
};

/// Constant 'MISSED_PATH'.
/**
  * The robot has diverged from the path and cannot rejoin it
 */
enum
{
  mbf_msgs__action__ExePath_Result__MISSED_PATH = 108
};

/// Constant 'BLOCKED_GOAL'.
/**
  * There's an obstacle at the goal
 */
enum
{
  mbf_msgs__action__ExePath_Result__BLOCKED_GOAL = 109
};

/// Constant 'BLOCKED_PATH'.
/**
  * There's an obstacle on the path
 */
enum
{
  mbf_msgs__action__ExePath_Result__BLOCKED_PATH = 110
};

/// Constant 'INVALID_PATH'.
enum
{
  mbf_msgs__action__ExePath_Result__INVALID_PATH = 111
};

/// Constant 'TF_ERROR'.
enum
{
  mbf_msgs__action__ExePath_Result__TF_ERROR = 112
};

/// Constant 'NOT_INITIALIZED'.
enum
{
  mbf_msgs__action__ExePath_Result__NOT_INITIALIZED = 113
};

/// Constant 'INVALID_PLUGIN'.
enum
{
  mbf_msgs__action__ExePath_Result__INVALID_PLUGIN = 114
};

/// Constant 'INTERNAL_ERROR'.
enum
{
  mbf_msgs__action__ExePath_Result__INTERNAL_ERROR = 115
};

/// Constant 'OUT_OF_MAP'.
/**
  * The start and / or the goal are outside the map
 */
enum
{
  mbf_msgs__action__ExePath_Result__OUT_OF_MAP = 116
};

/// Constant 'MAP_ERROR'.
/**
  * The map is not available or not running properly
 */
enum
{
  mbf_msgs__action__ExePath_Result__MAP_ERROR = 117
};

/// Constant 'STOPPED'.
/**
  * The controller execution has been stopped rigorously
 */
enum
{
  mbf_msgs__action__ExePath_Result__STOPPED = 118
};

/// Constant 'ERROR_RANGE_START'.
enum
{
  mbf_msgs__action__ExePath_Result__ERROR_RANGE_START = 100
};

/// Constant 'ERROR_RANGE_END'.
enum
{
  mbf_msgs__action__ExePath_Result__ERROR_RANGE_END = 149
};

/// Constant 'PLUGIN_ERROR_RANGE_START'.
/**
  * 121..149 are reserved as plugin specific errors:
 */
enum
{
  mbf_msgs__action__ExePath_Result__PLUGIN_ERROR_RANGE_START = 121
};

/// Constant 'PLUGIN_ERROR_RANGE_END'.
enum
{
  mbf_msgs__action__ExePath_Result__PLUGIN_ERROR_RANGE_END = 149
};

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'final_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in action/ExePath in the package mbf_msgs.
typedef struct mbf_msgs__action__ExePath_Result
{
  uint32_t outcome;
  rosidl_runtime_c__String message;
  geometry_msgs__msg__PoseStamped final_pose;
  float dist_to_goal;
  float angle_to_goal;
} mbf_msgs__action__ExePath_Result;

// Struct for a sequence of mbf_msgs__action__ExePath_Result.
typedef struct mbf_msgs__action__ExePath_Result__Sequence
{
  mbf_msgs__action__ExePath_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__ExePath_Result__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'current_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'last_cmd_vel'
#include "geometry_msgs/msg/detail/twist_stamped__struct.h"

/// Struct defined in action/ExePath in the package mbf_msgs.
typedef struct mbf_msgs__action__ExePath_Feedback
{
  /// Outcome of most recent controller cycle. Same values as in result
  uint32_t outcome;
  rosidl_runtime_c__String message;
  float dist_to_goal;
  float angle_to_goal;
  geometry_msgs__msg__PoseStamped current_pose;
  /// last command calculated by the controller
  geometry_msgs__msg__TwistStamped last_cmd_vel;
} mbf_msgs__action__ExePath_Feedback;

// Struct for a sequence of mbf_msgs__action__ExePath_Feedback.
typedef struct mbf_msgs__action__ExePath_Feedback__Sequence
{
  mbf_msgs__action__ExePath_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__ExePath_Feedback__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "mbf_msgs/action/detail/exe_path__struct.h"

/// Struct defined in action/ExePath in the package mbf_msgs.
typedef struct mbf_msgs__action__ExePath_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  mbf_msgs__action__ExePath_Goal goal;
} mbf_msgs__action__ExePath_SendGoal_Request;

// Struct for a sequence of mbf_msgs__action__ExePath_SendGoal_Request.
typedef struct mbf_msgs__action__ExePath_SendGoal_Request__Sequence
{
  mbf_msgs__action__ExePath_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__ExePath_SendGoal_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/ExePath in the package mbf_msgs.
typedef struct mbf_msgs__action__ExePath_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} mbf_msgs__action__ExePath_SendGoal_Response;

// Struct for a sequence of mbf_msgs__action__ExePath_SendGoal_Response.
typedef struct mbf_msgs__action__ExePath_SendGoal_Response__Sequence
{
  mbf_msgs__action__ExePath_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__ExePath_SendGoal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  mbf_msgs__action__ExePath_SendGoal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  mbf_msgs__action__ExePath_SendGoal_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/ExePath in the package mbf_msgs.
typedef struct mbf_msgs__action__ExePath_SendGoal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  mbf_msgs__action__ExePath_SendGoal_Request__Sequence request;
  mbf_msgs__action__ExePath_SendGoal_Response__Sequence response;
} mbf_msgs__action__ExePath_SendGoal_Event;

// Struct for a sequence of mbf_msgs__action__ExePath_SendGoal_Event.
typedef struct mbf_msgs__action__ExePath_SendGoal_Event__Sequence
{
  mbf_msgs__action__ExePath_SendGoal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__ExePath_SendGoal_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/ExePath in the package mbf_msgs.
typedef struct mbf_msgs__action__ExePath_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} mbf_msgs__action__ExePath_GetResult_Request;

// Struct for a sequence of mbf_msgs__action__ExePath_GetResult_Request.
typedef struct mbf_msgs__action__ExePath_GetResult_Request__Sequence
{
  mbf_msgs__action__ExePath_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__ExePath_GetResult_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.h"

/// Struct defined in action/ExePath in the package mbf_msgs.
typedef struct mbf_msgs__action__ExePath_GetResult_Response
{
  int8_t status;
  mbf_msgs__action__ExePath_Result result;
} mbf_msgs__action__ExePath_GetResult_Response;

// Struct for a sequence of mbf_msgs__action__ExePath_GetResult_Response.
typedef struct mbf_msgs__action__ExePath_GetResult_Response__Sequence
{
  mbf_msgs__action__ExePath_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__ExePath_GetResult_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  mbf_msgs__action__ExePath_GetResult_Event__request__MAX_SIZE = 1
};
// response
enum
{
  mbf_msgs__action__ExePath_GetResult_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/ExePath in the package mbf_msgs.
typedef struct mbf_msgs__action__ExePath_GetResult_Event
{
  service_msgs__msg__ServiceEventInfo info;
  mbf_msgs__action__ExePath_GetResult_Request__Sequence request;
  mbf_msgs__action__ExePath_GetResult_Response__Sequence response;
} mbf_msgs__action__ExePath_GetResult_Event;

// Struct for a sequence of mbf_msgs__action__ExePath_GetResult_Event.
typedef struct mbf_msgs__action__ExePath_GetResult_Event__Sequence
{
  mbf_msgs__action__ExePath_GetResult_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__ExePath_GetResult_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "mbf_msgs/action/detail/exe_path__struct.h"

/// Struct defined in action/ExePath in the package mbf_msgs.
typedef struct mbf_msgs__action__ExePath_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  mbf_msgs__action__ExePath_Feedback feedback;
} mbf_msgs__action__ExePath_FeedbackMessage;

// Struct for a sequence of mbf_msgs__action__ExePath_FeedbackMessage.
typedef struct mbf_msgs__action__ExePath_FeedbackMessage__Sequence
{
  mbf_msgs__action__ExePath_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__ExePath_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MBF_MSGS__ACTION__DETAIL__EXE_PATH__STRUCT_H_
