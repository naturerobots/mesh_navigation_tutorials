// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mesh_msgs:msg/MeshMaterials.idl
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
#include "mesh_msgs/msg/detail/mesh_materials__struct.h"
#include "mesh_msgs/msg/detail/mesh_materials__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "mesh_msgs/msg/detail/mesh_face_cluster__functions.h"
#include "mesh_msgs/msg/detail/mesh_material__functions.h"
#include "mesh_msgs/msg/detail/mesh_vertex_tex_coords__functions.h"
// end nested array functions include
bool mesh_msgs__msg__mesh_face_cluster__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * mesh_msgs__msg__mesh_face_cluster__convert_to_py(void * raw_ros_message);
bool mesh_msgs__msg__mesh_material__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * mesh_msgs__msg__mesh_material__convert_to_py(void * raw_ros_message);
bool mesh_msgs__msg__mesh_vertex_tex_coords__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * mesh_msgs__msg__mesh_vertex_tex_coords__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool mesh_msgs__msg__mesh_materials__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("mesh_msgs.msg._mesh_materials.MeshMaterials", full_classname_dest, 43) == 0);
  }
  mesh_msgs__msg__MeshMaterials * ros_message = _ros_message;
  {  // clusters
    PyObject * field = PyObject_GetAttrString(_pymsg, "clusters");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'clusters'");
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
    if (!mesh_msgs__msg__MeshFaceCluster__Sequence__init(&(ros_message->clusters), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create mesh_msgs__msg__MeshFaceCluster__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    mesh_msgs__msg__MeshFaceCluster * dest = ros_message->clusters.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!mesh_msgs__msg__mesh_face_cluster__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // materials
    PyObject * field = PyObject_GetAttrString(_pymsg, "materials");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'materials'");
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
    if (!mesh_msgs__msg__MeshMaterial__Sequence__init(&(ros_message->materials), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create mesh_msgs__msg__MeshMaterial__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    mesh_msgs__msg__MeshMaterial * dest = ros_message->materials.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!mesh_msgs__msg__mesh_material__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // cluster_materials
    PyObject * field = PyObject_GetAttrString(_pymsg, "cluster_materials");
    if (!field) {
      return false;
    }
    if (PyObject_CheckBuffer(field)) {
      // Optimization for converting arrays of primitives
      Py_buffer view;
      int rc = PyObject_GetBuffer(field, &view, PyBUF_SIMPLE);
      if (rc < 0) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = view.len / sizeof(uint32_t);
      if (!rosidl_runtime_c__uint32__Sequence__init(&(ros_message->cluster_materials), size)) {
        PyErr_SetString(PyExc_RuntimeError, "unable to create uint32__Sequence ros_message");
        PyBuffer_Release(&view);
        Py_DECREF(field);
        return false;
      }
      uint32_t * dest = ros_message->cluster_materials.data;
      rc = PyBuffer_ToContiguous(dest, &view, view.len, 'C');
      if (rc < 0) {
        PyBuffer_Release(&view);
        Py_DECREF(field);
        return false;
      }
      PyBuffer_Release(&view);
    } else {
      PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'cluster_materials'");
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
      if (!rosidl_runtime_c__uint32__Sequence__init(&(ros_message->cluster_materials), size)) {
        PyErr_SetString(PyExc_RuntimeError, "unable to create uint32__Sequence ros_message");
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
      uint32_t * dest = ros_message->cluster_materials.data;
      for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject * item = PySequence_Fast_GET_ITEM(seq_field, i);
        if (!item) {
          Py_DECREF(seq_field);
          Py_DECREF(field);
          return false;
        }
        assert(PyLong_Check(item));
        uint32_t tmp = PyLong_AsUnsignedLong(item);

        memcpy(&dest[i], &tmp, sizeof(uint32_t));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // vertex_tex_coords
    PyObject * field = PyObject_GetAttrString(_pymsg, "vertex_tex_coords");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'vertex_tex_coords'");
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
    if (!mesh_msgs__msg__MeshVertexTexCoords__Sequence__init(&(ros_message->vertex_tex_coords), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create mesh_msgs__msg__MeshVertexTexCoords__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    mesh_msgs__msg__MeshVertexTexCoords * dest = ros_message->vertex_tex_coords.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!mesh_msgs__msg__mesh_vertex_tex_coords__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
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
PyObject * mesh_msgs__msg__mesh_materials__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MeshMaterials */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mesh_msgs.msg._mesh_materials");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MeshMaterials");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mesh_msgs__msg__MeshMaterials * ros_message = (mesh_msgs__msg__MeshMaterials *)raw_ros_message;
  {  // clusters
    PyObject * field = NULL;
    size_t size = ros_message->clusters.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    mesh_msgs__msg__MeshFaceCluster * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->clusters.data[i]);
      PyObject * pyitem = mesh_msgs__msg__mesh_face_cluster__convert_to_py(item);
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
      int rc = PyObject_SetAttrString(_pymessage, "clusters", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // materials
    PyObject * field = NULL;
    size_t size = ros_message->materials.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    mesh_msgs__msg__MeshMaterial * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->materials.data[i]);
      PyObject * pyitem = mesh_msgs__msg__mesh_material__convert_to_py(item);
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
      int rc = PyObject_SetAttrString(_pymessage, "materials", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cluster_materials
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "cluster_materials");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "array.array") == 0);
    // ensure that itemsize matches the sizeof of the ROS message field
    PyObject * itemsize_attr = PyObject_GetAttrString(field, "itemsize");
    assert(itemsize_attr != NULL);
    size_t itemsize = PyLong_AsSize_t(itemsize_attr);
    Py_DECREF(itemsize_attr);
    if (itemsize != sizeof(uint32_t)) {
      PyErr_SetString(PyExc_RuntimeError, "itemsize doesn't match expectation");
      Py_DECREF(field);
      return NULL;
    }
    // clear the array, poor approach to remove potential default values
    Py_ssize_t length = PyObject_Length(field);
    if (-1 == length) {
      Py_DECREF(field);
      return NULL;
    }
    if (length > 0) {
      PyObject * pop = PyObject_GetAttrString(field, "pop");
      assert(pop != NULL);
      for (Py_ssize_t i = 0; i < length; ++i) {
        PyObject * ret = PyObject_CallFunctionObjArgs(pop, NULL);
        if (!ret) {
          Py_DECREF(pop);
          Py_DECREF(field);
          return NULL;
        }
        Py_DECREF(ret);
      }
      Py_DECREF(pop);
    }
    if (ros_message->cluster_materials.size > 0) {
      // populating the array.array using the frombytes method
      PyObject * frombytes = PyObject_GetAttrString(field, "frombytes");
      assert(frombytes != NULL);
      uint32_t * src = &(ros_message->cluster_materials.data[0]);
      PyObject * data = PyBytes_FromStringAndSize((const char *)src, ros_message->cluster_materials.size * sizeof(uint32_t));
      assert(data != NULL);
      PyObject * ret = PyObject_CallFunctionObjArgs(frombytes, data, NULL);
      Py_DECREF(data);
      Py_DECREF(frombytes);
      if (!ret) {
        Py_DECREF(field);
        return NULL;
      }
      Py_DECREF(ret);
    }
    Py_DECREF(field);
  }
  {  // vertex_tex_coords
    PyObject * field = NULL;
    size_t size = ros_message->vertex_tex_coords.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    mesh_msgs__msg__MeshVertexTexCoords * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->vertex_tex_coords.data[i]);
      PyObject * pyitem = mesh_msgs__msg__mesh_vertex_tex_coords__convert_to_py(item);
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
      int rc = PyObject_SetAttrString(_pymessage, "vertex_tex_coords", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
