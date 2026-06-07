// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mbf_msgs:srv/CheckPath.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "mbf_msgs/srv/detail/check_path__struct.h"
#include "mbf_msgs/srv/detail/check_path__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool nav_msgs__msg__path__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * nav_msgs__msg__path__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool mbf_msgs__srv__check_path__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[43];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("mbf_msgs.srv._check_path.CheckPath_Request", full_classname_dest, 42) == 0);
  }
  mbf_msgs__srv__CheckPath_Request * ros_message = _ros_message;
  {  // path
    PyObject * field = PyObject_GetAttrString(_pymsg, "path");
    if (!field) {
      return false;
    }
    if (!nav_msgs__msg__path__convert_from_py(field, &ros_message->path)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // safety_dist
    PyObject * field = PyObject_GetAttrString(_pymsg, "safety_dist");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->safety_dist = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // lethal_cost_mult
    PyObject * field = PyObject_GetAttrString(_pymsg, "lethal_cost_mult");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->lethal_cost_mult = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // inscrib_cost_mult
    PyObject * field = PyObject_GetAttrString(_pymsg, "inscrib_cost_mult");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->inscrib_cost_mult = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // unknown_cost_mult
    PyObject * field = PyObject_GetAttrString(_pymsg, "unknown_cost_mult");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->unknown_cost_mult = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // costmap
    PyObject * field = PyObject_GetAttrString(_pymsg, "costmap");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->costmap = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // return_on
    PyObject * field = PyObject_GetAttrString(_pymsg, "return_on");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->return_on = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // skip_poses
    PyObject * field = PyObject_GetAttrString(_pymsg, "skip_poses");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->skip_poses = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // use_padded_fp
    PyObject * field = PyObject_GetAttrString(_pymsg, "use_padded_fp");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->use_padded_fp = (Py_True == field);
    Py_DECREF(field);
  }
  {  // path_cells_only
    PyObject * field = PyObject_GetAttrString(_pymsg, "path_cells_only");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->path_cells_only = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mbf_msgs__srv__check_path__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of CheckPath_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mbf_msgs.srv._check_path");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "CheckPath_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mbf_msgs__srv__CheckPath_Request * ros_message = (mbf_msgs__srv__CheckPath_Request *)raw_ros_message;
  {  // path
    PyObject * field = NULL;
    field = nav_msgs__msg__path__convert_to_py(&ros_message->path);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "path", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // safety_dist
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->safety_dist);
    {
      int rc = PyObject_SetAttrString(_pymessage, "safety_dist", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lethal_cost_mult
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->lethal_cost_mult);
    {
      int rc = PyObject_SetAttrString(_pymessage, "lethal_cost_mult", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // inscrib_cost_mult
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->inscrib_cost_mult);
    {
      int rc = PyObject_SetAttrString(_pymessage, "inscrib_cost_mult", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // unknown_cost_mult
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->unknown_cost_mult);
    {
      int rc = PyObject_SetAttrString(_pymessage, "unknown_cost_mult", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // costmap
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->costmap);
    {
      int rc = PyObject_SetAttrString(_pymessage, "costmap", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // return_on
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->return_on);
    {
      int rc = PyObject_SetAttrString(_pymessage, "return_on", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // skip_poses
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->skip_poses);
    {
      int rc = PyObject_SetAttrString(_pymessage, "skip_poses", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // use_padded_fp
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->use_padded_fp ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "use_padded_fp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // path_cells_only
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->path_cells_only ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "path_cells_only", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "mbf_msgs/srv/detail/check_path__struct.h"
// already included above
// #include "mbf_msgs/srv/detail/check_path__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool mbf_msgs__srv__check_path__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[44];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("mbf_msgs.srv._check_path.CheckPath_Response", full_classname_dest, 43) == 0);
  }
  mbf_msgs__srv__CheckPath_Response * ros_message = _ros_message;
  {  // last_checked
    PyObject * field = PyObject_GetAttrString(_pymsg, "last_checked");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->last_checked = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // state
    PyObject * field = PyObject_GetAttrString(_pymsg, "state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->state = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // cost
    PyObject * field = PyObject_GetAttrString(_pymsg, "cost");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->cost = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mbf_msgs__srv__check_path__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of CheckPath_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mbf_msgs.srv._check_path");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "CheckPath_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mbf_msgs__srv__CheckPath_Response * ros_message = (mbf_msgs__srv__CheckPath_Response *)raw_ros_message;
  {  // last_checked
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->last_checked);
    {
      int rc = PyObject_SetAttrString(_pymessage, "last_checked", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // state
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cost
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->cost);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cost", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "mbf_msgs/srv/detail/check_path__struct.h"
// already included above
// #include "mbf_msgs/srv/detail/check_path__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes


// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool service_msgs__msg__service_event_info__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * service_msgs__msg__service_event_info__convert_to_py(void * raw_ros_message);
bool mbf_msgs__srv__check_path__request__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * mbf_msgs__srv__check_path__request__convert_to_py(void * raw_ros_message);
bool mbf_msgs__srv__check_path__response__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * mbf_msgs__srv__check_path__response__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool mbf_msgs__srv__check_path__event__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[41];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("mbf_msgs.srv._check_path.CheckPath_Event", full_classname_dest, 40) == 0);
  }
  mbf_msgs__srv__CheckPath_Event * ros_message = _ros_message;
  {  // info
    PyObject * field = PyObject_GetAttrString(_pymsg, "info");
    if (!field) {
      return false;
    }
    if (!service_msgs__msg__service_event_info__convert_from_py(field, &ros_message->info)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // request
    PyObject * field = PyObject_GetAttrString(_pymsg, "request");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'request'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!mbf_msgs__srv__CheckPath_Request__Sequence__init(&(ros_message->request), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create mbf_msgs__srv__CheckPath_Request__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    mbf_msgs__srv__CheckPath_Request * dest = ros_message->request.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!mbf_msgs__srv__check_path__request__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // response
    PyObject * field = PyObject_GetAttrString(_pymsg, "response");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'response'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!mbf_msgs__srv__CheckPath_Response__Sequence__init(&(ros_message->response), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create mbf_msgs__srv__CheckPath_Response__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    mbf_msgs__srv__CheckPath_Response * dest = ros_message->response.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!mbf_msgs__srv__check_path__response__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mbf_msgs__srv__check_path__event__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of CheckPath_Event */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mbf_msgs.srv._check_path");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "CheckPath_Event");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mbf_msgs__srv__CheckPath_Event * ros_message = (mbf_msgs__srv__CheckPath_Event *)raw_ros_message;
  {  // info
    PyObject * field = NULL;
    field = service_msgs__msg__service_event_info__convert_to_py(&ros_message->info);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "info", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // request
    PyObject * field = NULL;
    size_t size = ros_message->request.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    mbf_msgs__srv__CheckPath_Request * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->request.data[i]);
      PyObject * pyitem = mbf_msgs__srv__check_path__request__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "request", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // response
    PyObject * field = NULL;
    size_t size = ros_message->response.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    mbf_msgs__srv__CheckPath_Response * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->response.data[i]);
      PyObject * pyitem = mbf_msgs__srv__check_path__response__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "response", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
