# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mesh_msgs:msg/MeshGeometry.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MeshGeometry(type):
    """Metaclass of message 'MeshGeometry'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('mesh_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mesh_msgs.msg.MeshGeometry')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__mesh_geometry
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__mesh_geometry
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__mesh_geometry
            cls._TYPE_SUPPORT = module.type_support_msg__msg__mesh_geometry
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__mesh_geometry

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

            from mesh_msgs.msg import MeshTriangleIndices
            if MeshTriangleIndices.__class__._TYPE_SUPPORT is None:
                MeshTriangleIndices.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MeshGeometry(metaclass=Metaclass_MeshGeometry):
    """Message class 'MeshGeometry'."""

    __slots__ = [
        '_vertices',
        '_vertex_normals',
        '_faces',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'vertices': 'sequence<geometry_msgs/Point>',
        'vertex_normals': 'sequence<geometry_msgs/Point>',
        'faces': 'sequence<mesh_msgs/MeshTriangleIndices>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['mesh_msgs', 'msg'], 'MeshTriangleIndices')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.vertices = kwargs.get('vertices', [])
        self.vertex_normals = kwargs.get('vertex_normals', [])
        self.faces = kwargs.get('faces', [])

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.vertices != other.vertices:
            return False
        if self.vertex_normals != other.vertex_normals:
            return False
        if self.faces != other.faces:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def vertices(self):
        """Message field 'vertices'."""
        return self._vertices

    @vertices.setter
    def vertices(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Point
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, Point) for v in value) and
                 True), \
                "The 'vertices' field must be a set or sequence and each value of type 'Point'"
        self._vertices = value

    @builtins.property
    def vertex_normals(self):
        """Message field 'vertex_normals'."""
        return self._vertex_normals

    @vertex_normals.setter
    def vertex_normals(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Point
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, Point) for v in value) and
                 True), \
                "The 'vertex_normals' field must be a set or sequence and each value of type 'Point'"
        self._vertex_normals = value

    @builtins.property
    def faces(self):
        """Message field 'faces'."""
        return self._faces

    @faces.setter
    def faces(self, value):
        if self._check_fields:
            from mesh_msgs.msg import MeshTriangleIndices
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, MeshTriangleIndices) for v in value) and
                 True), \
                "The 'faces' field must be a set or sequence and each value of type 'MeshTriangleIndices'"
        self._faces = value
