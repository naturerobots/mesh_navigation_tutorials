// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mbf_msgs:srv/CheckPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/srv/check_point.h"


#ifndef MBF_MSGS__SRV__DETAIL__CHECK_POINT__STRUCT_H_
#define MBF_MSGS__SRV__DETAIL__CHECK_POINT__STRUCT_H_

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
  mbf_msgs__srv__CheckPoint_Request__LOCAL_COSTMAP = 1
};

/// Constant 'GLOBAL_COSTMAP'.
enum
{
  mbf_msgs__srv__CheckPoint_Request__GLOBAL_COSTMAP = 2
};

// Include directives for member types
// Member 'point'
#include "geometry_msgs/msg/detail/point_stamped__struct.h"

/// Struct defined in srv/CheckPoint in the package mbf_msgs.
typedef struct mbf_msgs__srv__CheckPoint_Request
{
  /// the point to be checked after transforming to costmap frame
  geometry_msgs__msg__PointStamped point;
  /// costmap in which to check the point
  uint8_t costmap;
} mbf_msgs__srv__CheckPoint_Request;

// Struct for a sequence of mbf_msgs__srv__CheckPoint_Request.
typedef struct mbf_msgs__srv__CheckPoint_Request__Sequence
{
  mbf_msgs__srv__CheckPoint_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__srv__CheckPoint_Request__Sequence;

// Constants defined in the message

/// Constant 'FREE'.
/**
  * point is in traversable space
 */
enum
{
  mbf_msgs__srv__CheckPoint_Response__FREE = 0
};

/// Constant 'INSCRIBED'.
/**
  * point is in inscribed space
 */
enum
{
  mbf_msgs__srv__CheckPoint_Response__INSCRIBED = 1
};

/// Constant 'LETHAL'.
/**
  * point is in collision
 */
enum
{
  mbf_msgs__srv__CheckPoint_Response__LETHAL = 2
};

/// Constant 'UNKNOWN'.
/**
  * point is in unknown space
 */
enum
{
  mbf_msgs__srv__CheckPoint_Response__UNKNOWN = 3
};

/// Constant 'OUTSIDE'.
/**
  * point is outside the map
 */
enum
{
  mbf_msgs__srv__CheckPoint_Response__OUTSIDE = 4
};

/// Struct defined in srv/CheckPoint in the package mbf_msgs.
typedef struct mbf_msgs__srv__CheckPoint_Response
{
  /// point state: FREE, INSCRIBED, LETHAL, UNKNOWN or OUTSIDE
  uint8_t state;
  /// cost of the cell at point
  uint32_t cost;
} mbf_msgs__srv__CheckPoint_Response;

// Struct for a sequence of mbf_msgs__srv__CheckPoint_Response.
typedef struct mbf_msgs__srv__CheckPoint_Response__Sequence
{
  mbf_msgs__srv__CheckPoint_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__srv__CheckPoint_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  mbf_msgs__srv__CheckPoint_Event__request__MAX_SIZE = 1
};
// response
enum
{
  mbf_msgs__srv__CheckPoint_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/CheckPoint in the package mbf_msgs.
typedef struct mbf_msgs__srv__CheckPoint_Event
{
  service_msgs__msg__ServiceEventInfo info;
  mbf_msgs__srv__CheckPoint_Request__Sequence request;
  mbf_msgs__srv__CheckPoint_Response__Sequence response;
} mbf_msgs__srv__CheckPoint_Event;

// Struct for a sequence of mbf_msgs__srv__CheckPoint_Event.
typedef struct mbf_msgs__srv__CheckPoint_Event__Sequence
{
  mbf_msgs__srv__CheckPoint_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbf_msgs__srv__CheckPoint_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MBF_MSGS__SRV__DETAIL__CHECK_POINT__STRUCT_H_
