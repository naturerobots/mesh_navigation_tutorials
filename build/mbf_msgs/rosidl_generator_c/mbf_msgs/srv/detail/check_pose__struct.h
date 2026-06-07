// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mbf_msgs:srv/CheckPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/srv/check_pose.h"


#ifndef MBF_MSGS__SRV__DETAIL__CHECK_POSE__STRUCT_H_
#define MBF_MSGS__SRV__DETAIL__CHECK_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'LOCAL_COSTMAP'.
enum
{
  mbf_msgs__srv__CheckPose_Request__LOCAL_COSTMAP = 1
};

/// Constant 'GLOBAL_COSTMAP'.
enum
{
  mbf_msgs__srv__CheckPose_Request__GLOBAL_COSTMAP = 2
};

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in srv/CheckPose in the package mbf_msgs.
typedef struct mbf_msgs__srv__CheckPose_Request
{
  /// the pose to be checked after transforming to costmap frame
  geometry_msgs__msg__PoseStamped pose;
  /// minimum distance allowed to the closest obstacle
  float safety_dist;
  /// cost multiplier for cells marked as lethal obstacle (zero is ignored)
  float lethal_cost_mult;
  /// cost multiplier for cells marked as inscribed obstacle (zero is ignored)
  float inscrib_cost_mult;
  /// cost multiplier for cells marked as unknown space (zero is ignored)
  float unknown_cost_mult;
  /// costmap in which to check the pose
  uint8_t costmap;
  /// check current robot pose instead (ignores pose field)
  bool current_pose;
  /// include footprint padding when checking cost; note that safety distance
  /// will be measured from the padded footprint
  bool use_padded_fp;
} mbf_msgs__srv__CheckPose_Request;

// Struct for a sequence of mbf_msgs__srv__CheckPose_Request.
typedef struct mbf_msgs__srv__CheckPose_Request__Sequence
{
  mbf_msgs__srv__CheckPose_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__srv__CheckPose_Request__Sequence;

// Constants defined in the message

/// Constant 'FREE'.
/**
  * robot is completely in traversable space
 */
enum
{
  mbf_msgs__srv__CheckPose_Response__FREE = 0
};

/// Constant 'INSCRIBED'.
/**
  * robot is partially in inscribed space
 */
enum
{
  mbf_msgs__srv__CheckPose_Response__INSCRIBED = 1
};

/// Constant 'LETHAL'.
/**
  * robot is partially in collision
 */
enum
{
  mbf_msgs__srv__CheckPose_Response__LETHAL = 2
};

/// Constant 'UNKNOWN'.
/**
  * robot is partially in unknown space
 */
enum
{
  mbf_msgs__srv__CheckPose_Response__UNKNOWN = 3
};

/// Constant 'OUTSIDE'.
/**
  * robot is partially outside the map
 */
enum
{
  mbf_msgs__srv__CheckPose_Response__OUTSIDE = 4
};

/// Struct defined in srv/CheckPose in the package mbf_msgs.
typedef struct mbf_msgs__srv__CheckPose_Response
{
  /// pose state: FREE, INSCRIBED, LETHAL, UNKNOWN or OUTSIDE
  uint8_t state;
  /// total cost of all cells within footprint padded by safety_dist
  uint32_t cost;
} mbf_msgs__srv__CheckPose_Response;

// Struct for a sequence of mbf_msgs__srv__CheckPose_Response.
typedef struct mbf_msgs__srv__CheckPose_Response__Sequence
{
  mbf_msgs__srv__CheckPose_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__srv__CheckPose_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  mbf_msgs__srv__CheckPose_Event__request__MAX_SIZE = 1
};
// response
enum
{
  mbf_msgs__srv__CheckPose_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/CheckPose in the package mbf_msgs.
typedef struct mbf_msgs__srv__CheckPose_Event
{
  service_msgs__msg__ServiceEventInfo info;
  mbf_msgs__srv__CheckPose_Request__Sequence request;
  mbf_msgs__srv__CheckPose_Response__Sequence response;
} mbf_msgs__srv__CheckPose_Event;

// Struct for a sequence of mbf_msgs__srv__CheckPose_Event.
typedef struct mbf_msgs__srv__CheckPose_Event__Sequence
{
  mbf_msgs__srv__CheckPose_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__srv__CheckPose_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MBF_MSGS__SRV__DETAIL__CHECK_POSE__STRUCT_H_
