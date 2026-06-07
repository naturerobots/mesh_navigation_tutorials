// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mbf_msgs:srv/CheckPath.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/srv/check_path.h"


#ifndef MBF_MSGS__SRV__DETAIL__CHECK_PATH__STRUCT_H_
#define MBF_MSGS__SRV__DETAIL__CHECK_PATH__STRUCT_H_

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
  mbf_msgs__srv__CheckPath_Request__LOCAL_COSTMAP = 1
};

/// Constant 'GLOBAL_COSTMAP'.
enum
{
  mbf_msgs__srv__CheckPath_Request__GLOBAL_COSTMAP = 2
};

// Include directives for member types
// Member 'path'
#include "nav_msgs/msg/detail/path__struct.h"

/// Struct defined in srv/CheckPath in the package mbf_msgs.
typedef struct mbf_msgs__srv__CheckPath_Request
{
  /// the path to be checked after transforming to costmap frame
  nav_msgs__msg__Path path;
  /// minimum distance allowed to the closest obstacle (footprint padding)
  float safety_dist;
  /// cost multiplier for cells marked as lethal obstacle (zero is ignored)
  float lethal_cost_mult;
  /// cost multiplier for cells marked as inscribed obstacle (zero is ignored)
  float inscrib_cost_mult;
  /// cost multiplier for cells marked as unknown space (zero is ignored)
  float unknown_cost_mult;
  /// costmap in which to check the pose
  uint8_t costmap;
  /// abort check on finding a pose with this state or worse (zero is ignored)
  uint8_t return_on;
  /// skip this number of poses between checks, to speedup processing
  uint8_t skip_poses;
  /// include footprint padding when checking cost; note that safety distance
  /// will be measured from the padded footprint
  bool use_padded_fp;
  /// check only cells directly traversed by the path, ignoring robot footprint
  /// (if true, both safety_dist and use_padded_fp are ignored)
  bool path_cells_only;
} mbf_msgs__srv__CheckPath_Request;

// Struct for a sequence of mbf_msgs__srv__CheckPath_Request.
typedef struct mbf_msgs__srv__CheckPath_Request__Sequence
{
  mbf_msgs__srv__CheckPath_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__srv__CheckPath_Request__Sequence;

// Constants defined in the message

/// Constant 'FREE'.
/**
  * path is completely in traversable space
 */
enum
{
  mbf_msgs__srv__CheckPath_Response__FREE = 0
};

/// Constant 'INSCRIBED'.
/**
  * path is partially in inscribed space
 */
enum
{
  mbf_msgs__srv__CheckPath_Response__INSCRIBED = 1
};

/// Constant 'LETHAL'.
/**
  * path is partially in collision
 */
enum
{
  mbf_msgs__srv__CheckPath_Response__LETHAL = 2
};

/// Constant 'UNKNOWN'.
/**
  * path is partially in unknown space
 */
enum
{
  mbf_msgs__srv__CheckPath_Response__UNKNOWN = 3
};

/// Constant 'OUTSIDE'.
/**
  * path is partially outside the map
 */
enum
{
  mbf_msgs__srv__CheckPath_Response__OUTSIDE = 4
};

/// Struct defined in srv/CheckPath in the package mbf_msgs.
typedef struct mbf_msgs__srv__CheckPath_Response
{
  /// index of the first pose along the path with return_on state or worse
  uint32_t last_checked;
  /// path worst state (until last_checked): FREE, INSCRIBED, LETHAL, UNKNOWN...
  uint8_t state;
  /// cost of all cells traversed by path within footprint padded by safety_dist
  /// or just by the path if path_cells_only is true
  uint32_t cost;
} mbf_msgs__srv__CheckPath_Response;

// Struct for a sequence of mbf_msgs__srv__CheckPath_Response.
typedef struct mbf_msgs__srv__CheckPath_Response__Sequence
{
  mbf_msgs__srv__CheckPath_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__srv__CheckPath_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  mbf_msgs__srv__CheckPath_Event__request__MAX_SIZE = 1
};
// response
enum
{
  mbf_msgs__srv__CheckPath_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/CheckPath in the package mbf_msgs.
typedef struct mbf_msgs__srv__CheckPath_Event
{
  service_msgs__msg__ServiceEventInfo info;
  mbf_msgs__srv__CheckPath_Request__Sequence request;
  mbf_msgs__srv__CheckPath_Response__Sequence response;
} mbf_msgs__srv__CheckPath_Event;

// Struct for a sequence of mbf_msgs__srv__CheckPath_Event.
typedef struct mbf_msgs__srv__CheckPath_Event__Sequence
{
  mbf_msgs__srv__CheckPath_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__srv__CheckPath_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MBF_MSGS__SRV__DETAIL__CHECK_PATH__STRUCT_H_
