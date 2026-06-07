# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mesh_msgs:msg/MeshMaterials.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

# Member 'cluster_materials'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MeshMaterials(type):
    """Metaclass of message 'MeshMaterials'."""

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
                'mesh_msgs.msg.MeshMaterials')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__mesh_materials
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__mesh_materials
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__mesh_materials
            cls._TYPE_SUPPORT = module.type_support_msg__msg__mesh_materials
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__mesh_materials

            from mesh_msgs.msg import MeshFaceCluster
            if MeshFaceCluster.__class__._TYPE_SUPPORT is None:
                MeshFaceCluster.__class__.__import_type_support__()

            from mesh_msgs.msg import MeshMaterial
            if MeshMaterial.__class__._TYPE_SUPPORT is None:
                MeshMaterial.__class__.__import_type_support__()

            from mesh_msgs.msg import MeshVertexTexCoords
            if MeshVertexTexCoords.__class__._TYPE_SUPPORT is None:
                MeshVertexTexCoords.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MeshMaterials(metaclass=Metaclass_MeshMaterials):
    """Message class 'MeshMaterials'."""

    __slots__ = [
        '_clusters',
        '_materials',
        '_cluster_materials',
        '_vertex_tex_coords',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'clusters': 'sequence<mesh_msgs/MeshFaceCluster>',
        'materials': 'sequence<mesh_msgs/MeshMaterial>',
        'cluster_materials': 'sequence<uint32>',
        'vertex_tex_coords': 'sequence<mesh_msgs/MeshVertexTexCoords>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['mesh_msgs', 'msg'], 'MeshFaceCluster')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['mesh_msgs', 'msg'], 'MeshMaterial')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['mesh_msgs', 'msg'], 'MeshVertexTexCoords')),  # noqa: E501
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
        self.clusters = kwargs.get('clusters', [])
        self.materials = kwargs.get('materials', [])
        self.cluster_materials = array.array('I', kwargs.get('cluster_materials', []))
        self.vertex_tex_coords = kwargs.get('vertex_tex_coords', [])

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
        if self.clusters != other.clusters:
            return False
        if self.materials != other.materials:
            return False
        if self.cluster_materials != other.cluster_materials:
            return False
        if self.vertex_tex_coords != other.vertex_tex_coords:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def clusters(self):
        """Message field 'clusters'."""
        return self._clusters

    @clusters.setter
    def clusters(self, value):
        if self._check_fields:
            from mesh_msgs.msg import MeshFaceCluster
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
                 all(isinstance(v, MeshFaceCluster) for v in value) and
                 True), \
                "The 'clusters' field must be a set or sequence and each value of type 'MeshFaceCluster'"
        self._clusters = value

    @builtins.property
    def materials(self):
        """Message field 'materials'."""
        return self._materials

    @materials.setter
    def materials(self, value):
        if self._check_fields:
            from mesh_msgs.msg import MeshMaterial
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
                 all(isinstance(v, MeshMaterial) for v in value) and
                 True), \
                "The 'materials' field must be a set or sequence and each value of type 'MeshMaterial'"
        self._materials = value

    @builtins.property
    def cluster_materials(self):
        """Message field 'cluster_materials'."""
        return self._cluster_materials

    @cluster_materials.setter
    def cluster_materials(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'I', \
                    "The 'cluster_materials' array.array() must have the type code of 'I'"
                self._cluster_materials = value
                return
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 4294967296 for val in value)), \
                "The 'cluster_materials' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 4294967295]"
        self._cluster_materials = array.array('I', value)

    @builtins.property
    def vertex_tex_coords(self):
        """Message field 'vertex_tex_coords'."""
        return self._vertex_tex_coords

    @vertex_tex_coords.setter
    def vertex_tex_coords(self, value):
        if self._check_fields:
            from mesh_msgs.msg import MeshVertexTexCoords
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
                 all(isinstance(v, MeshVertexTexCoords) for v in value) and
                 True), \
                "The 'vertex_tex_coords' field must be a set or sequence and each value of type 'MeshVertexTexCoords'"
        self._vertex_tex_coords = value
