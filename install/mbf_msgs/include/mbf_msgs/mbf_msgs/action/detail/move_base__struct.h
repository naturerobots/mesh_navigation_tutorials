// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mbf_msgs:action/MoveBase.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/action/move_base.h"


#ifndef MBF_MSGS__ACTION__DETAIL__MOVE_BASE__STRUCT_H_
#define MBF_MSGS__ACTION__DETAIL__MOVE_BASE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'target_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'controller'
// Member 'planner'
// Member 'recovery_behaviors'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/MoveBase in the package mbf_msgs.
typedef struct mbf_msgs__action__MoveBase_Goal
{
  geometry_msgs__msg__PoseStamped target_pose;
  /// Controller to use; defaults to the first one specified on "controllers" parameter
  rosidl_runtime_c__String controller;
  /// Planner to use; defaults to the first one specified on "planners" parameter
  rosidl_runtime_c__String planner;
  /// Recovery behaviors to try on case of failure; defaults to the "recovery_behaviors" parameter value
  rosidl_runtime_c__String__Sequence recovery_behaviors;
} mbf_msgs__action__MoveBase_Goal;

// Struct for a sequence of mbf_msgs__action__MoveBase_Goal.
typedef struct mbf_msgs__action__MoveBase_Goal__Sequence
{
  mbf_msgs__action__MoveBase_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__MoveBase_Goal__Sequence;

// Constants defined in the message

/// Constant 'SUCCESS'.
/**
  * Predefined success codes:
 */
enum
{
  mbf_msgs__action__MoveBase_Result__SUCCESS = 0
};

/// Constant 'FAILURE'.
/**
  * Predefined general error codes:
 */
enum
{
  mbf_msgs__action__MoveBase_Result__FAILURE = 10
};

/// Constant 'CANCELED'.
enum
{
  mbf_msgs__action__MoveBase_Result__CANCELED = 11
};

/// Constant 'COLLISION'.
enum
{
  mbf_msgs__action__MoveBase_Result__COLLISION = 12
};

/// Constant 'OSCILLATION'.
enum
{
  mbf_msgs__action__MoveBase_Result__OSCILLATION = 13
};

/// Constant 'START_BLOCKED'.
enum
{
  mbf_msgs__action__MoveBase_Result__START_BLOCKED = 14
};

/// Constant 'GOAL_BLOCKED'.
enum
{
  mbf_msgs__action__MoveBase_Result__GOAL_BLOCKED = 15
};

/// Constant 'TF_ERROR'.
enum
{
  mbf_msgs__action__MoveBase_Result__TF_ERROR = 16
};

/// Constant 'INTERNAL_ERROR'.
enum
{
  mbf_msgs__action__MoveBase_Result__INTERNAL_ERROR = 17
};

/// Constant 'ERROR_RANGE_START'.
enum
{
  mbf_msgs__action__MoveBase_Result__ERROR_RANGE_START = 10
};

/// Constant 'ERROR_RANGE_END'.
enum
{
  mbf_msgs__action__MoveBase_Result__ERROR_RANGE_END = 49
};

/// Constant 'PLAN_FAILURE'.
/**
  * 21..49 are reserved for future general error codes
  * Planning/controlling failures:
 */
enum
{
  mbf_msgs__action__MoveBase_Result__PLAN_FAILURE = 50
};

/// Constant 'CTRL_FAILURE'.
/**
  * 51..99 are reserved as planner specific errors
 */
enum
{
  mbf_msgs__action__MoveBase_Result__CTRL_FAILURE = 100
};

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'final_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in action/MoveBase in the package mbf_msgs.
typedef struct mbf_msgs__action__MoveBase_Result
{
  /// 101..149 are reserved as controller specific errors
  uint32_t outcome;
  rosidl_runtime_c__String message;
  /// Configuration upon action completion
  float dist_to_goal;
  float angle_to_goal;
  geometry_msgs__msg__PoseStamped final_pose;
} mbf_msgs__action__MoveBase_Result;

// Struct for a sequence of mbf_msgs__action__MoveBase_Result.
typedef struct mbf_msgs__action__MoveBase_Result__Sequence
{
  mbf_msgs__action__MoveBase_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__MoveBase_Result__Sequence;

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

/// Struct defined in action/MoveBase in the package mbf_msgs.
typedef struct mbf_msgs__action__MoveBase_Feedback
{
  /// Outcome of most recent controller cycle. Same values as in MoveBase or ExePath result
  uint32_t outcome;
  rosidl_runtime_c__String message;
  float dist_to_goal;
  float angle_to_goal;
  geometry_msgs__msg__PoseStamped current_pose;
  /// last command calculated by the controller
  geometry_msgs__msg__TwistStamped last_cmd_vel;
} mbf_msgs__action__MoveBase_Feedback;

// Struct for a sequence of mbf_msgs__action__MoveBase_Feedback.
typedef struct mbf_msgs__action__MoveBase_Feedback__Sequence
{
  mbf_msgs__action__MoveBase_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__MoveBase_Feedback__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "mbf_msgs/action/detail/move_base__struct.h"

/// Struct defined in action/MoveBase in the package mbf_msgs.
typedef struct mbf_msgs__action__MoveBase_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  mbf_msgs__action__MoveBase_Goal goal;
} mbf_msgs__action__MoveBase_SendGoal_Request;

// Struct for a sequence of mbf_msgs__action__MoveBase_SendGoal_Request.
typedef struct mbf_msgs__action__MoveBase_SendGoal_Request__Sequence
{
  mbf_msgs__action__MoveBase_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__MoveBase_SendGoal_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/MoveBase in the package mbf_msgs.
typedef struct mbf_msgs__action__MoveBase_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} mbf_msgs__action__MoveBase_SendGoal_Response;

// Struct for a sequence of mbf_msgs__action__MoveBase_SendGoal_Response.
typedef struct mbf_msgs__action__MoveBase_SendGoal_Response__Sequence
{
  mbf_msgs__action__MoveBase_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__MoveBase_SendGoal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  mbf_msgs__action__MoveBase_SendGoal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  mbf_msgs__action__MoveBase_SendGoal_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/MoveBase in the package mbf_msgs.
typedef struct mbf_msgs__action__MoveBase_SendGoal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  mbf_msgs__action__MoveBase_SendGoal_Request__Sequence request;
  mbf_msgs__action__MoveBase_SendGoal_Response__Sequence response;
} mbf_msgs__action__MoveBase_SendGoal_Event;

// Struct for a sequence of mbf_msgs__action__MoveBase_SendGoal_Event.
typedef struct mbf_msgs__action__MoveBase_SendGoal_Event__Sequence
{
  mbf_msgs__action__MoveBase_SendGoal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__MoveBase_SendGoal_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/MoveBase in the package mbf_msgs.
typedef struct mbf_msgs__action__MoveBase_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} mbf_msgs__action__MoveBase_GetResult_Request;

// Struct for a sequence of mbf_msgs__action__MoveBase_GetResult_Request.
typedef struct mbf_msgs__action__MoveBase_GetResult_Request__Sequence
{
  mbf_msgs__action__MoveBase_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__MoveBase_GetResult_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "mbf_msgs/action/detail/move_base__struct.h"

/// Struct defined in action/MoveBase in the package mbf_msgs.
typedef struct mbf_msgs__action__MoveBase_GetResult_Response
{
  int8_t status;
  mbf_msgs__action__MoveBase_Result result;
} mbf_msgs__action__MoveBase_GetResult_Response;

// Struct for a sequence of mbf_msgs__action__MoveBase_GetResult_Response.
typedef struct mbf_msgs__action__MoveBase_GetResult_Response__Sequence
{
  mbf_msgs__action__MoveBase_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__MoveBase_GetResult_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  mbf_msgs__action__MoveBase_GetResult_Event__request__MAX_SIZE = 1
};
// response
enum
{
  mbf_msgs__action__MoveBase_GetResult_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/MoveBase in the package mbf_msgs.
typedef struct mbf_msgs__action__MoveBase_GetResult_Event
{
  service_msgs__msg__ServiceEventInfo info;
  mbf_msgs__action__MoveBase_GetResult_Request__Sequence request;
  mbf_msgs__action__MoveBase_GetResult_Response__Sequence response;
} mbf_msgs__action__MoveBase_GetResult_Event;

// Struct for a sequence of mbf_msgs__action__MoveBase_GetResult_Event.
typedef struct mbf_msgs__action__MoveBase_GetResult_Event__Sequence
{
  mbf_msgs__action__MoveBase_GetResult_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__MoveBase_GetResult_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "mbf_msgs/action/detail/move_base__struct.h"

/// Struct defined in action/MoveBase in the package mbf_msgs.
typedef struct mbf_msgs__action__MoveBase_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  mbf_msgs__action__MoveBase_Feedback feedback;
} mbf_msgs__action__MoveBase_FeedbackMessage;

// Struct for a sequence of mbf_msgs__action__MoveBase_FeedbackMessage.
typedef struct mbf_msgs__action__MoveBase_FeedbackMessage__Sequence
{
  mbf_msgs__action__MoveBase_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__MoveBase_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MBF_MSGS__ACTION__DETAIL__MOVE_BASE__STRUCT_H_
