// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mbf_msgs:action/Recovery.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/action/recovery.h"


#ifndef MBF_MSGS__ACTION__DETAIL__RECOVERY__STRUCT_H_
#define MBF_MSGS__ACTION__DETAIL__RECOVERY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'behavior'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/Recovery in the package mbf_msgs.
typedef struct mbf_msgs__action__Recovery_Goal
{
  rosidl_runtime_c__String behavior;
  /// use different slots for concurrency
  uint8_t concurrency_slot;
} mbf_msgs__action__Recovery_Goal;

// Struct for a sequence of mbf_msgs__action__Recovery_Goal.
typedef struct mbf_msgs__action__Recovery_Goal__Sequence
{
  mbf_msgs__action__Recovery_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__Recovery_Goal__Sequence;

// Constants defined in the message

/// Constant 'SUCCESS'.
/**
  * Predefined success codes:
 */
enum
{
  mbf_msgs__action__Recovery_Result__SUCCESS = 0
};

/// Constant 'FAILURE'.
/**
  * Possible server codes:
 */
enum
{
  mbf_msgs__action__Recovery_Result__FAILURE = 150
};

/// Constant 'CANCELED'.
enum
{
  mbf_msgs__action__Recovery_Result__CANCELED = 151
};

/// Constant 'PAT_EXCEEDED'.
enum
{
  mbf_msgs__action__Recovery_Result__PAT_EXCEEDED = 152
};

/// Constant 'TF_ERROR'.
enum
{
  mbf_msgs__action__Recovery_Result__TF_ERROR = 153
};

/// Constant 'NOT_INITIALIZED'.
enum
{
  mbf_msgs__action__Recovery_Result__NOT_INITIALIZED = 154
};

/// Constant 'INVALID_PLUGIN'.
enum
{
  mbf_msgs__action__Recovery_Result__INVALID_PLUGIN = 155
};

/// Constant 'INTERNAL_ERROR'.
enum
{
  mbf_msgs__action__Recovery_Result__INTERNAL_ERROR = 156
};

/// Constant 'STOPPED'.
/**
  * The recovery behaviour execution has been stopped rigorously
 */
enum
{
  mbf_msgs__action__Recovery_Result__STOPPED = 157
};

/// Constant 'IMPASSABLE'.
/**
  * Further execution would lead to a collision
 */
enum
{
  mbf_msgs__action__Recovery_Result__IMPASSABLE = 158
};

/// Constant 'ERROR_RANGE_START'.
enum
{
  mbf_msgs__action__Recovery_Result__ERROR_RANGE_START = 150
};

/// Constant 'ERROR_RANGE_END'.
enum
{
  mbf_msgs__action__Recovery_Result__ERROR_RANGE_END = 199
};

/// Constant 'PLUGIN_ERROR_RANGE_START'.
/**
  * 171..199 are reserved as plugin specific errors:
 */
enum
{
  mbf_msgs__action__Recovery_Result__PLUGIN_ERROR_RANGE_START = 171
};

/// Constant 'PLUGIN_ERROR_RANGE_END'.
enum
{
  mbf_msgs__action__Recovery_Result__PLUGIN_ERROR_RANGE_END = 199
};

// Include directives for member types
// Member 'message'
// Member 'used_plugin'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/Recovery in the package mbf_msgs.
typedef struct mbf_msgs__action__Recovery_Result
{
  uint32_t outcome;
  rosidl_runtime_c__String message;
  rosidl_runtime_c__String used_plugin;
} mbf_msgs__action__Recovery_Result;

// Struct for a sequence of mbf_msgs__action__Recovery_Result.
typedef struct mbf_msgs__action__Recovery_Result__Sequence
{
  mbf_msgs__action__Recovery_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__Recovery_Result__Sequence;

// Constants defined in the message

/// Struct defined in action/Recovery in the package mbf_msgs.
typedef struct mbf_msgs__action__Recovery_Feedback
{
  uint8_t structure_needs_at_least_one_member;
} mbf_msgs__action__Recovery_Feedback;

// Struct for a sequence of mbf_msgs__action__Recovery_Feedback.
typedef struct mbf_msgs__action__Recovery_Feedback__Sequence
{
  mbf_msgs__action__Recovery_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__Recovery_Feedback__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "mbf_msgs/action/detail/recovery__struct.h"

/// Struct defined in action/Recovery in the package mbf_msgs.
typedef struct mbf_msgs__action__Recovery_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  mbf_msgs__action__Recovery_Goal goal;
} mbf_msgs__action__Recovery_SendGoal_Request;

// Struct for a sequence of mbf_msgs__action__Recovery_SendGoal_Request.
typedef struct mbf_msgs__action__Recovery_SendGoal_Request__Sequence
{
  mbf_msgs__action__Recovery_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__Recovery_SendGoal_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Recovery in the package mbf_msgs.
typedef struct mbf_msgs__action__Recovery_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} mbf_msgs__action__Recovery_SendGoal_Response;

// Struct for a sequence of mbf_msgs__action__Recovery_SendGoal_Response.
typedef struct mbf_msgs__action__Recovery_SendGoal_Response__Sequence
{
  mbf_msgs__action__Recovery_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__Recovery_SendGoal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  mbf_msgs__action__Recovery_SendGoal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  mbf_msgs__action__Recovery_SendGoal_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/Recovery in the package mbf_msgs.
typedef struct mbf_msgs__action__Recovery_SendGoal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  mbf_msgs__action__Recovery_SendGoal_Request__Sequence request;
  mbf_msgs__action__Recovery_SendGoal_Response__Sequence response;
} mbf_msgs__action__Recovery_SendGoal_Event;

// Struct for a sequence of mbf_msgs__action__Recovery_SendGoal_Event.
typedef struct mbf_msgs__action__Recovery_SendGoal_Event__Sequence
{
  mbf_msgs__action__Recovery_SendGoal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__Recovery_SendGoal_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Recovery in the package mbf_msgs.
typedef struct mbf_msgs__action__Recovery_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} mbf_msgs__action__Recovery_GetResult_Request;

// Struct for a sequence of mbf_msgs__action__Recovery_GetResult_Request.
typedef struct mbf_msgs__action__Recovery_GetResult_Request__Sequence
{
  mbf_msgs__action__Recovery_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__Recovery_GetResult_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "mbf_msgs/action/detail/recovery__struct.h"

/// Struct defined in action/Recovery in the package mbf_msgs.
typedef struct mbf_msgs__action__Recovery_GetResult_Response
{
  int8_t status;
  mbf_msgs__action__Recovery_Result result;
} mbf_msgs__action__Recovery_GetResult_Response;

// Struct for a sequence of mbf_msgs__action__Recovery_GetResult_Response.
typedef struct mbf_msgs__action__Recovery_GetResult_Response__Sequence
{
  mbf_msgs__action__Recovery_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__Recovery_GetResult_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  mbf_msgs__action__Recovery_GetResult_Event__request__MAX_SIZE = 1
};
// response
enum
{
  mbf_msgs__action__Recovery_GetResult_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/Recovery in the package mbf_msgs.
typedef struct mbf_msgs__action__Recovery_GetResult_Event
{
  service_msgs__msg__ServiceEventInfo info;
  mbf_msgs__action__Recovery_GetResult_Request__Sequence request;
  mbf_msgs__action__Recovery_GetResult_Response__Sequence response;
} mbf_msgs__action__Recovery_GetResult_Event;

// Struct for a sequence of mbf_msgs__action__Recovery_GetResult_Event.
typedef struct mbf_msgs__action__Recovery_GetResult_Event__Sequence
{
  mbf_msgs__action__Recovery_GetResult_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__Recovery_GetResult_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "mbf_msgs/action/detail/recovery__struct.h"

/// Struct defined in action/Recovery in the package mbf_msgs.
typedef struct mbf_msgs__action__Recovery_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  mbf_msgs__action__Recovery_Feedback feedback;
} mbf_msgs__action__Recovery_FeedbackMessage;

// Struct for a sequence of mbf_msgs__action__Recovery_FeedbackMessage.
typedef struct mbf_msgs__action__Recovery_FeedbackMessage__Sequence
{
  mbf_msgs__action__Recovery_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__action__Recovery_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MBF_MSGS__ACTION__DETAIL__RECOVERY__STRUCT_H_
