// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from mbf_msgs:srv/SetTestRobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/srv/set_test_robot_state.h"


#ifndef MBF_MSGS__SRV__DETAIL__SET_TEST_ROBOT_STATE__FUNCTIONS_H_
#define MBF_MSGS__SRV__DETAIL__SET_TEST_ROBOT_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "mbf_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "mbf_msgs/srv/detail/set_test_robot_state__struct.h"

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
const rosidl_type_hash_t *
mbf_msgs__srv__SetTestRobotState__get_type_hash(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
const rosidl_runtime_c__type_description__TypeDescription *
mbf_msgs__srv__SetTestRobotState__get_type_description(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
const rosidl_runtime_c__type_description__TypeSource *
mbf_msgs__srv__SetTestRobotState__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
mbf_msgs__srv__SetTestRobotState__get_type_description_sources(
  const rosidl_service_type_support_t * type_support);

/// Initialize srv/SetTestRobotState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mbf_msgs__srv__SetTestRobotState_Request
 * )) before or use
 * mbf_msgs__srv__SetTestRobotState_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
bool
mbf_msgs__srv__SetTestRobotState_Request__init(mbf_msgs__srv__SetTestRobotState_Request * msg);

/// Finalize srv/SetTestRobotState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
void
mbf_msgs__srv__SetTestRobotState_Request__fini(mbf_msgs__srv__SetTestRobotState_Request * msg);

/// Create srv/SetTestRobotState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mbf_msgs__srv__SetTestRobotState_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
mbf_msgs__srv__SetTestRobotState_Request *
mbf_msgs__srv__SetTestRobotState_Request__create(void);

/// Destroy srv/SetTestRobotState message.
/**
 * It calls
 * mbf_msgs__srv__SetTestRobotState_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
void
mbf_msgs__srv__SetTestRobotState_Request__destroy(mbf_msgs__srv__SetTestRobotState_Request * msg);

/// Check for srv/SetTestRobotState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
bool
mbf_msgs__srv__SetTestRobotState_Request__are_equal(const mbf_msgs__srv__SetTestRobotState_Request * lhs, const mbf_msgs__srv__SetTestRobotState_Request * rhs);

/// Copy a srv/SetTestRobotState message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
bool
mbf_msgs__srv__SetTestRobotState_Request__copy(
  const mbf_msgs__srv__SetTestRobotState_Request * input,
  mbf_msgs__srv__SetTestRobotState_Request * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
const rosidl_type_hash_t *
mbf_msgs__srv__SetTestRobotState_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
const rosidl_runtime_c__type_description__TypeDescription *
mbf_msgs__srv__SetTestRobotState_Request__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
const rosidl_runtime_c__type_description__TypeSource *
mbf_msgs__srv__SetTestRobotState_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
mbf_msgs__srv__SetTestRobotState_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/SetTestRobotState messages.
/**
 * It allocates the memory for the number of elements and calls
 * mbf_msgs__srv__SetTestRobotState_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
bool
mbf_msgs__srv__SetTestRobotState_Request__Sequence__init(mbf_msgs__srv__SetTestRobotState_Request__Sequence * array, size_t size);

/// Finalize array of srv/SetTestRobotState messages.
/**
 * It calls
 * mbf_msgs__srv__SetTestRobotState_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
void
mbf_msgs__srv__SetTestRobotState_Request__Sequence__fini(mbf_msgs__srv__SetTestRobotState_Request__Sequence * array);

/// Create array of srv/SetTestRobotState messages.
/**
 * It allocates the memory for the array and calls
 * mbf_msgs__srv__SetTestRobotState_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
mbf_msgs__srv__SetTestRobotState_Request__Sequence *
mbf_msgs__srv__SetTestRobotState_Request__Sequence__create(size_t size);

/// Destroy array of srv/SetTestRobotState messages.
/**
 * It calls
 * mbf_msgs__srv__SetTestRobotState_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
void
mbf_msgs__srv__SetTestRobotState_Request__Sequence__destroy(mbf_msgs__srv__SetTestRobotState_Request__Sequence * array);

/// Check for srv/SetTestRobotState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
bool
mbf_msgs__srv__SetTestRobotState_Request__Sequence__are_equal(const mbf_msgs__srv__SetTestRobotState_Request__Sequence * lhs, const mbf_msgs__srv__SetTestRobotState_Request__Sequence * rhs);

/// Copy an array of srv/SetTestRobotState messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
bool
mbf_msgs__srv__SetTestRobotState_Request__Sequence__copy(
  const mbf_msgs__srv__SetTestRobotState_Request__Sequence * input,
  mbf_msgs__srv__SetTestRobotState_Request__Sequence * output);

/// Initialize srv/SetTestRobotState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mbf_msgs__srv__SetTestRobotState_Response
 * )) before or use
 * mbf_msgs__srv__SetTestRobotState_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
bool
mbf_msgs__srv__SetTestRobotState_Response__init(mbf_msgs__srv__SetTestRobotState_Response * msg);

/// Finalize srv/SetTestRobotState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
void
mbf_msgs__srv__SetTestRobotState_Response__fini(mbf_msgs__srv__SetTestRobotState_Response * msg);

/// Create srv/SetTestRobotState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mbf_msgs__srv__SetTestRobotState_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
mbf_msgs__srv__SetTestRobotState_Response *
mbf_msgs__srv__SetTestRobotState_Response__create(void);

/// Destroy srv/SetTestRobotState message.
/**
 * It calls
 * mbf_msgs__srv__SetTestRobotState_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
void
mbf_msgs__srv__SetTestRobotState_Response__destroy(mbf_msgs__srv__SetTestRobotState_Response * msg);

/// Check for srv/SetTestRobotState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
bool
mbf_msgs__srv__SetTestRobotState_Response__are_equal(const mbf_msgs__srv__SetTestRobotState_Response * lhs, const mbf_msgs__srv__SetTestRobotState_Response * rhs);

/// Copy a srv/SetTestRobotState message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
bool
mbf_msgs__srv__SetTestRobotState_Response__copy(
  const mbf_msgs__srv__SetTestRobotState_Response * input,
  mbf_msgs__srv__SetTestRobotState_Response * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
const rosidl_type_hash_t *
mbf_msgs__srv__SetTestRobotState_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
const rosidl_runtime_c__type_description__TypeDescription *
mbf_msgs__srv__SetTestRobotState_Response__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
const rosidl_runtime_c__type_description__TypeSource *
mbf_msgs__srv__SetTestRobotState_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
mbf_msgs__srv__SetTestRobotState_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/SetTestRobotState messages.
/**
 * It allocates the memory for the number of elements and calls
 * mbf_msgs__srv__SetTestRobotState_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
bool
mbf_msgs__srv__SetTestRobotState_Response__Sequence__init(mbf_msgs__srv__SetTestRobotState_Response__Sequence * array, size_t size);

/// Finalize array of srv/SetTestRobotState messages.
/**
 * It calls
 * mbf_msgs__srv__SetTestRobotState_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
void
mbf_msgs__srv__SetTestRobotState_Response__Sequence__fini(mbf_msgs__srv__SetTestRobotState_Response__Sequence * array);

/// Create array of srv/SetTestRobotState messages.
/**
 * It allocates the memory for the array and calls
 * mbf_msgs__srv__SetTestRobotState_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
mbf_msgs__srv__SetTestRobotState_Response__Sequence *
mbf_msgs__srv__SetTestRobotState_Response__Sequence__create(size_t size);

/// Destroy array of srv/SetTestRobotState messages.
/**
 * It calls
 * mbf_msgs__srv__SetTestRobotState_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
void
mbf_msgs__srv__SetTestRobotState_Response__Sequence__destroy(mbf_msgs__srv__SetTestRobotState_Response__Sequence * array);

/// Check for srv/SetTestRobotState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
bool
mbf_msgs__srv__SetTestRobotState_Response__Sequence__are_equal(const mbf_msgs__srv__SetTestRobotState_Response__Sequence * lhs, const mbf_msgs__srv__SetTestRobotState_Response__Sequence * rhs);

/// Copy an array of srv/SetTestRobotState messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
bool
mbf_msgs__srv__SetTestRobotState_Response__Sequence__copy(
  const mbf_msgs__srv__SetTestRobotState_Response__Sequence * input,
  mbf_msgs__srv__SetTestRobotState_Response__Sequence * output);

/// Initialize srv/SetTestRobotState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mbf_msgs__srv__SetTestRobotState_Event
 * )) before or use
 * mbf_msgs__srv__SetTestRobotState_Event__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
bool
mbf_msgs__srv__SetTestRobotState_Event__init(mbf_msgs__srv__SetTestRobotState_Event * msg);

/// Finalize srv/SetTestRobotState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
void
mbf_msgs__srv__SetTestRobotState_Event__fini(mbf_msgs__srv__SetTestRobotState_Event * msg);

/// Create srv/SetTestRobotState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mbf_msgs__srv__SetTestRobotState_Event__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
mbf_msgs__srv__SetTestRobotState_Event *
mbf_msgs__srv__SetTestRobotState_Event__create(void);

/// Destroy srv/SetTestRobotState message.
/**
 * It calls
 * mbf_msgs__srv__SetTestRobotState_Event__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
void
mbf_msgs__srv__SetTestRobotState_Event__destroy(mbf_msgs__srv__SetTestRobotState_Event * msg);

/// Check for srv/SetTestRobotState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
bool
mbf_msgs__srv__SetTestRobotState_Event__are_equal(const mbf_msgs__srv__SetTestRobotState_Event * lhs, const mbf_msgs__srv__SetTestRobotState_Event * rhs);

/// Copy a srv/SetTestRobotState message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
bool
mbf_msgs__srv__SetTestRobotState_Event__copy(
  const mbf_msgs__srv__SetTestRobotState_Event * input,
  mbf_msgs__srv__SetTestRobotState_Event * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
const rosidl_type_hash_t *
mbf_msgs__srv__SetTestRobotState_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
const rosidl_runtime_c__type_description__TypeDescription *
mbf_msgs__srv__SetTestRobotState_Event__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
const rosidl_runtime_c__type_description__TypeSource *
mbf_msgs__srv__SetTestRobotState_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
mbf_msgs__srv__SetTestRobotState_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/SetTestRobotState messages.
/**
 * It allocates the memory for the number of elements and calls
 * mbf_msgs__srv__SetTestRobotState_Event__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
bool
mbf_msgs__srv__SetTestRobotState_Event__Sequence__init(mbf_msgs__srv__SetTestRobotState_Event__Sequence * array, size_t size);

/// Finalize array of srv/SetTestRobotState messages.
/**
 * It calls
 * mbf_msgs__srv__SetTestRobotState_Event__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
void
mbf_msgs__srv__SetTestRobotState_Event__Sequence__fini(mbf_msgs__srv__SetTestRobotState_Event__Sequence * array);

/// Create array of srv/SetTestRobotState messages.
/**
 * It allocates the memory for the array and calls
 * mbf_msgs__srv__SetTestRobotState_Event__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
mbf_msgs__srv__SetTestRobotState_Event__Sequence *
mbf_msgs__srv__SetTestRobotState_Event__Sequence__create(size_t size);

/// Destroy array of srv/SetTestRobotState messages.
/**
 * It calls
 * mbf_msgs__srv__SetTestRobotState_Event__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
void
mbf_msgs__srv__SetTestRobotState_Event__Sequence__destroy(mbf_msgs__srv__SetTestRobotState_Event__Sequence * array);

/// Check for srv/SetTestRobotState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
bool
mbf_msgs__srv__SetTestRobotState_Event__Sequence__are_equal(const mbf_msgs__srv__SetTestRobotState_Event__Sequence * lhs, const mbf_msgs__srv__SetTestRobotState_Event__Sequence * rhs);

/// Copy an array of srv/SetTestRobotState messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mbf_msgs
bool
mbf_msgs__srv__SetTestRobotState_Event__Sequence__copy(
  const mbf_msgs__srv__SetTestRobotState_Event__Sequence * input,
  mbf_msgs__srv__SetTestRobotState_Event__Sequence * output);
#ifdef __cplusplus
}
#endif

#endif  // MBF_MSGS__SRV__DETAIL__SET_TEST_ROBOT_STATE__FUNCTIONS_H_
