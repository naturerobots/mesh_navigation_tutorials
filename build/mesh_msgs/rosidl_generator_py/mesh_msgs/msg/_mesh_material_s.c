// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mesh_msgs:msg/MeshMaterial.idl
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
#include "mesh_msgs/msg/detail/mesh_material__struct.h"
#include "mesh_msgs/msg/detail/mesh_material__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__color_rgba__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__color_rgba__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool mesh_msgs__msg__mesh_material__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[42];
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
    assert(strncmp("mesh_msgs.msg._mesh_material.MeshMaterial", full_classname_dest, 41) == 0);
  }
  mesh_msgs__msg__MeshMaterial * ros_message = _ros_message;
  {  // texture_index
    PyObject * field = PyObject_GetAttrString(_pymsg, "texture_index");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->texture_index = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // color
    PyObject * field = PyObject_GetAttrString(_pymsg, "color");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__color_rgba__convert_from_py(field, &ros_message->color)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // has_texture
    PyObject * field = PyObject_GetAttrString(_pymsg, "has_texture");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->has_texture = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mesh_msgs__msg__mesh_material__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MeshMaterial */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mesh_msgs.msg._mesh_material");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MeshMaterial");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mesh_msgs__msg__MeshMaterial * ros_message = (mesh_msgs__msg__MeshMaterial *)raw_ros_message;
  {  // texture_index
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->texture_index);
    {
      int rc = PyObject_SetAttrString(_pymessage, "texture_index", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // color
    PyObject * field = NULL;
    field = std_msgs__msg__color_rgba__convert_to_py(&ros_message->color);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "color", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // has_texture
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->has_texture ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "has_texture", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
