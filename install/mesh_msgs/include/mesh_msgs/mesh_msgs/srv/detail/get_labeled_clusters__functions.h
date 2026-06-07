// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from mesh_msgs:srv/GetLabeledClusters.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/srv/get_labeled_clusters.h"


#ifndef MESH_MSGS__SRV__DETAIL__GET_LABELED_CLUSTERS__FUNCTIONS_H_
#define MESH_MSGS__SRV__DETAIL__GET_LABELED_CLUSTERS__FUNCTIONS_H_

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
#include "mesh_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "mesh_msgs/srv/detail/get_labeled_clusters__struct.h"

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetLabeledClusters__get_type_hash(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__srv__GetLabeledClusters__get_type_description(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetLabeledClusters__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetLabeledClusters__get_type_description_sources(
  const rosidl_service_type_support_t * type_support);

/// Initialize srv/GetLabeledClusters message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mesh_msgs__srv__GetLabeledClusters_Request
 * )) before or use
 * mesh_msgs__srv__GetLabeledClusters_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
bool
mesh_msgs__srv__GetLabeledClusters_Request__init(mesh_msgs__srv__GetLabeledClusters_Request * msg);

/// Finalize srv/GetLabeledClusters message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
void
mesh_msgs__srv__GetLabeledClusters_Request__fini(mesh_msgs__srv__GetLabeledClusters_Request * msg);

/// Create srv/GetLabeledClusters message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mesh_msgs__srv__GetLabeledClusters_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
mesh_msgs__srv__GetLabeledClusters_Request *
mesh_msgs__srv__GetLabeledClusters_Request__create(void);

/// Destroy srv/GetLabeledClusters message.
/**
 * It calls
 * mesh_msgs__srv__GetLabeledClusters_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
void
mesh_msgs__srv__GetLabeledClusters_Request__destroy(mesh_msgs__srv__GetLabeledClusters_Request * msg);

/// Check for srv/GetLabeledClusters message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
bool
mesh_msgs__srv__GetLabeledClusters_Request__are_equal(const mesh_msgs__srv__GetLabeledClusters_Request * lhs, const mesh_msgs__srv__GetLabeledClusters_Request * rhs);

/// Copy a srv/GetLabeledClusters message.
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
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
bool
mesh_msgs__srv__GetLabeledClusters_Request__copy(
  const mesh_msgs__srv__GetLabeledClusters_Request * input,
  mesh_msgs__srv__GetLabeledClusters_Request * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetLabeledClusters_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__srv__GetLabeledClusters_Request__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetLabeledClusters_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetLabeledClusters_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/GetLabeledClusters messages.
/**
 * It allocates the memory for the number of elements and calls
 * mesh_msgs__srv__GetLabeledClusters_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
bool
mesh_msgs__srv__GetLabeledClusters_Request__Sequence__init(mesh_msgs__srv__GetLabeledClusters_Request__Sequence * array, size_t size);

/// Finalize array of srv/GetLabeledClusters messages.
/**
 * It calls
 * mesh_msgs__srv__GetLabeledClusters_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
void
mesh_msgs__srv__GetLabeledClusters_Request__Sequence__fini(mesh_msgs__srv__GetLabeledClusters_Request__Sequence * array);

/// Create array of srv/GetLabeledClusters messages.
/**
 * It allocates the memory for the array and calls
 * mesh_msgs__srv__GetLabeledClusters_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
mesh_msgs__srv__GetLabeledClusters_Request__Sequence *
mesh_msgs__srv__GetLabeledClusters_Request__Sequence__create(size_t size);

/// Destroy array of srv/GetLabeledClusters messages.
/**
 * It calls
 * mesh_msgs__srv__GetLabeledClusters_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
void
mesh_msgs__srv__GetLabeledClusters_Request__Sequence__destroy(mesh_msgs__srv__GetLabeledClusters_Request__Sequence * array);

/// Check for srv/GetLabeledClusters message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
bool
mesh_msgs__srv__GetLabeledClusters_Request__Sequence__are_equal(const mesh_msgs__srv__GetLabeledClusters_Request__Sequence * lhs, const mesh_msgs__srv__GetLabeledClusters_Request__Sequence * rhs);

/// Copy an array of srv/GetLabeledClusters messages.
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
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
bool
mesh_msgs__srv__GetLabeledClusters_Request__Sequence__copy(
  const mesh_msgs__srv__GetLabeledClusters_Request__Sequence * input,
  mesh_msgs__srv__GetLabeledClusters_Request__Sequence * output);

/// Initialize srv/GetLabeledClusters message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mesh_msgs__srv__GetLabeledClusters_Response
 * )) before or use
 * mesh_msgs__srv__GetLabeledClusters_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
bool
mesh_msgs__srv__GetLabeledClusters_Response__init(mesh_msgs__srv__GetLabeledClusters_Response * msg);

/// Finalize srv/GetLabeledClusters message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
void
mesh_msgs__srv__GetLabeledClusters_Response__fini(mesh_msgs__srv__GetLabeledClusters_Response * msg);

/// Create srv/GetLabeledClusters message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mesh_msgs__srv__GetLabeledClusters_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
mesh_msgs__srv__GetLabeledClusters_Response *
mesh_msgs__srv__GetLabeledClusters_Response__create(void);

/// Destroy srv/GetLabeledClusters message.
/**
 * It calls
 * mesh_msgs__srv__GetLabeledClusters_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
void
mesh_msgs__srv__GetLabeledClusters_Response__destroy(mesh_msgs__srv__GetLabeledClusters_Response * msg);

/// Check for srv/GetLabeledClusters message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
bool
mesh_msgs__srv__GetLabeledClusters_Response__are_equal(const mesh_msgs__srv__GetLabeledClusters_Response * lhs, const mesh_msgs__srv__GetLabeledClusters_Response * rhs);

/// Copy a srv/GetLabeledClusters message.
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
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
bool
mesh_msgs__srv__GetLabeledClusters_Response__copy(
  const mesh_msgs__srv__GetLabeledClusters_Response * input,
  mesh_msgs__srv__GetLabeledClusters_Response * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetLabeledClusters_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__srv__GetLabeledClusters_Response__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetLabeledClusters_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetLabeledClusters_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/GetLabeledClusters messages.
/**
 * It allocates the memory for the number of elements and calls
 * mesh_msgs__srv__GetLabeledClusters_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
bool
mesh_msgs__srv__GetLabeledClusters_Response__Sequence__init(mesh_msgs__srv__GetLabeledClusters_Response__Sequence * array, size_t size);

/// Finalize array of srv/GetLabeledClusters messages.
/**
 * It calls
 * mesh_msgs__srv__GetLabeledClusters_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
void
mesh_msgs__srv__GetLabeledClusters_Response__Sequence__fini(mesh_msgs__srv__GetLabeledClusters_Response__Sequence * array);

/// Create array of srv/GetLabeledClusters messages.
/**
 * It allocates the memory for the array and calls
 * mesh_msgs__srv__GetLabeledClusters_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
mesh_msgs__srv__GetLabeledClusters_Response__Sequence *
mesh_msgs__srv__GetLabeledClusters_Response__Sequence__create(size_t size);

/// Destroy array of srv/GetLabeledClusters messages.
/**
 * It calls
 * mesh_msgs__srv__GetLabeledClusters_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
void
mesh_msgs__srv__GetLabeledClusters_Response__Sequence__destroy(mesh_msgs__srv__GetLabeledClusters_Response__Sequence * array);

/// Check for srv/GetLabeledClusters message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
bool
mesh_msgs__srv__GetLabeledClusters_Response__Sequence__are_equal(const mesh_msgs__srv__GetLabeledClusters_Response__Sequence * lhs, const mesh_msgs__srv__GetLabeledClusters_Response__Sequence * rhs);

/// Copy an array of srv/GetLabeledClusters messages.
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
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
bool
mesh_msgs__srv__GetLabeledClusters_Response__Sequence__copy(
  const mesh_msgs__srv__GetLabeledClusters_Response__Sequence * input,
  mesh_msgs__srv__GetLabeledClusters_Response__Sequence * output);

/// Initialize srv/GetLabeledClusters message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mesh_msgs__srv__GetLabeledClusters_Event
 * )) before or use
 * mesh_msgs__srv__GetLabeledClusters_Event__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
bool
mesh_msgs__srv__GetLabeledClusters_Event__init(mesh_msgs__srv__GetLabeledClusters_Event * msg);

/// Finalize srv/GetLabeledClusters message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
void
mesh_msgs__srv__GetLabeledClusters_Event__fini(mesh_msgs__srv__GetLabeledClusters_Event * msg);

/// Create srv/GetLabeledClusters message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mesh_msgs__srv__GetLabeledClusters_Event__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
mesh_msgs__srv__GetLabeledClusters_Event *
mesh_msgs__srv__GetLabeledClusters_Event__create(void);

/// Destroy srv/GetLabeledClusters message.
/**
 * It calls
 * mesh_msgs__srv__GetLabeledClusters_Event__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
void
mesh_msgs__srv__GetLabeledClusters_Event__destroy(mesh_msgs__srv__GetLabeledClusters_Event * msg);

/// Check for srv/GetLabeledClusters message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
bool
mesh_msgs__srv__GetLabeledClusters_Event__are_equal(const mesh_msgs__srv__GetLabeledClusters_Event * lhs, const mesh_msgs__srv__GetLabeledClusters_Event * rhs);

/// Copy a srv/GetLabeledClusters message.
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
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
bool
mesh_msgs__srv__GetLabeledClusters_Event__copy(
  const mesh_msgs__srv__GetLabeledClusters_Event * input,
  mesh_msgs__srv__GetLabeledClusters_Event * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_type_hash_t *
mesh_msgs__srv__GetLabeledClusters_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_runtime_c__type_description__TypeDescription *
mesh_msgs__srv__GetLabeledClusters_Event__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_runtime_c__type_description__TypeSource *
mesh_msgs__srv__GetLabeledClusters_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
mesh_msgs__srv__GetLabeledClusters_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of srv/GetLabeledClusters messages.
/**
 * It allocates the memory for the number of elements and calls
 * mesh_msgs__srv__GetLabeledClusters_Event__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
bool
mesh_msgs__srv__GetLabeledClusters_Event__Sequence__init(mesh_msgs__srv__GetLabeledClusters_Event__Sequence * array, size_t size);

/// Finalize array of srv/GetLabeledClusters messages.
/**
 * It calls
 * mesh_msgs__srv__GetLabeledClusters_Event__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
void
mesh_msgs__srv__GetLabeledClusters_Event__Sequence__fini(mesh_msgs__srv__GetLabeledClusters_Event__Sequence * array);

/// Create array of srv/GetLabeledClusters messages.
/**
 * It allocates the memory for the array and calls
 * mesh_msgs__srv__GetLabeledClusters_Event__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
mesh_msgs__srv__GetLabeledClusters_Event__Sequence *
mesh_msgs__srv__GetLabeledClusters_Event__Sequence__create(size_t size);

/// Destroy array of srv/GetLabeledClusters messages.
/**
 * It calls
 * mesh_msgs__srv__GetLabeledClusters_Event__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
void
mesh_msgs__srv__GetLabeledClusters_Event__Sequence__destroy(mesh_msgs__srv__GetLabeledClusters_Event__Sequence * array);

/// Check for srv/GetLabeledClusters message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
bool
mesh_msgs__srv__GetLabeledClusters_Event__Sequence__are_equal(const mesh_msgs__srv__GetLabeledClusters_Event__Sequence * lhs, const mesh_msgs__srv__GetLabeledClusters_Event__Sequence * rhs);

/// Copy an array of srv/GetLabeledClusters messages.
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
ROSIDL_GENERATOR_C_PUBLIC_mesh_msgs
bool
mesh_msgs__srv__GetLabeledClusters_Event__Sequence__copy(
  const mesh_msgs__srv__GetLabeledClusters_Event__Sequence * input,
  mesh_msgs__srv__GetLabeledClusters_Event__Sequence * output);
#ifdef __cplusplus
}
#endif

#endif  // MESH_MSGS__SRV__DETAIL__GET_LABELED_CLUSTERS__FUNCTIONS_H_
