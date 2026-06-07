// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mbf_msgs:srv/SetTestRobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/srv/set_test_robot_state.h"


#ifndef MBF_MSGS__SRV__DETAIL__SET_TEST_ROBOT_STATE__STRUCT_H_
#define MBF_MSGS__SRV__DETAIL__SET_TEST_ROBOT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'transform'
#include "geometry_msgs/msg/detail/transform_stamped__struct.h"
// Member 'velocity_robot'
#include "geometry_msgs/msg/detail/twist_stamped__struct.h"

/// Struct defined in srv/SetTestRobotState in the package mbf_msgs.
typedef struct mbf_msgs__srv__SetTestRobotState_Request
{
  /// Set the transform that defines where the robot is located to this
  geometry_msgs__msg__TransformStamped transform;
  /// Whether to also set the robot's velocity
  bool set_velocity;
  /// Only used when set_velocity is true; Velocity needs to be given in the robot's reference frame (e.g. "base_link")
  geometry_msgs__msg__TwistStamped velocity_robot;
} mbf_msgs__srv__SetTestRobotState_Request;

// Struct for a sequence of mbf_msgs__srv__SetTestRobotState_Request.
typedef struct mbf_msgs__srv__SetTestRobotState_Request__Sequence
{
  mbf_msgs__srv__SetTestRobotState_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__srv__SetTestRobotState_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/SetTestRobotState in the package mbf_msgs.
typedef struct mbf_msgs__srv__SetTestRobotState_Response
{
  /// whether the robot's pose and velocity has been set
  bool success;
} mbf_msgs__srv__SetTestRobotState_Response;

// Struct for a sequence of mbf_msgs__srv__SetTestRobotState_Response.
typedef struct mbf_msgs__srv__SetTestRobotState_Response__Sequence
{
  mbf_msgs__srv__SetTestRobotState_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__srv__SetTestRobotState_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  mbf_msgs__srv__SetTestRobotState_Event__request__MAX_SIZE = 1
};
// response
enum
{
  mbf_msgs__srv__SetTestRobotState_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/SetTestRobotState in the package mbf_msgs.
typedef struct mbf_msgs__srv__SetTestRobotState_Event
{
  service_msgs__msg__ServiceEventInfo info;
  mbf_msgs__srv__SetTestRobotState_Request__Sequence request;
  mbf_msgs__srv__SetTestRobotState_Response__Sequence response;
} mbf_msgs__srv__SetTestRobotState_Event;

// Struct for a sequence of mbf_msgs__srv__SetTestRobotState_Event.
typedef struct mbf_msgs__srv__SetTestRobotState_Event__Sequence
{
  mbf_msgs__srv__SetTestRobotState_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__srv__SetTestRobotState_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MBF_MSGS__SRV__DETAIL__SET_TEST_ROBOT_STATE__STRUCT_H_
