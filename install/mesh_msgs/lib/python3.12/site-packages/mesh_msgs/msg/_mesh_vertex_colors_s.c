// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mesh_msgs:msg/MeshVertexColors.idl
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
#include "mesh_msgs/msg/detail/mesh_vertex_colors__struct.h"
#include "mesh_msgs/msg/detail/mesh_vertex_colors__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "std_msgs/msg/detail/color_rgba__functions.h"
// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__color_rgba__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__color_rgba__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool mesh_msgs__msg__mesh_vertex_colors__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[51];
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
    assert(strncmp("mesh_msgs.msg._mesh_vertex_colors.MeshVertexColors", full_classname_dest, 50) == 0);
  }
  mesh_msgs__msg__MeshVertexColors * ros_message = _ros_message;
  {  // vertex_colors
    PyObject * field = PyObject_GetAttrString(_pymsg, "vertex_colors");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'vertex_colors'");
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
    if (!std_msgs__msg__ColorRGBA__Sequence__init(&(ros_message->vertex_colors), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create std_msgs__msg__ColorRGBA__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    std_msgs__msg__ColorRGBA * dest = ros_message->vertex_colors.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!std_msgs__msg__color_rgba__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
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
PyObject * mesh_msgs__msg__mesh_vertex_colors__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MeshVertexColors */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mesh_msgs.msg._mesh_vertex_colors");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MeshVertexColors");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mesh_msgs__msg__MeshVertexColors * ros_message = (mesh_msgs__msg__MeshVertexColors *)raw_ros_message;
  {  // vertex_colors
    PyObject * field = NULL;
    size_t size = ros_message->vertex_colors.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    std_msgs__msg__ColorRGBA * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->vertex_colors.data[i]);
      PyObject * pyitem = std_msgs__msg__color_rgba__convert_to_py(item);
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
      int rc = PyObject_SetAttrString(_pymessage, "vertex_colors", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
