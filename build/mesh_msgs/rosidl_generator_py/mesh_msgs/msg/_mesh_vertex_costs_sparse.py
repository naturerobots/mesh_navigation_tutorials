# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mesh_msgs:msg/MeshVertexCostsSparse.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

# Member 'vertices'
# Member 'costs'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MeshVertexCostsSparse(type):
    """Metaclass of message 'MeshVertexCostsSparse'."""

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
                'mesh_msgs.msg.MeshVertexCostsSparse')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__mesh_vertex_costs_sparse
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__mesh_vertex_costs_sparse
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__mesh_vertex_costs_sparse
            cls._TYPE_SUPPORT = module.type_support_msg__msg__mesh_vertex_costs_sparse
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__mesh_vertex_costs_sparse

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MeshVertexCostsSparse(metaclass=Metaclass_MeshVertexCostsSparse):
    """Message class 'MeshVertexCostsSparse'."""

    __slots__ = [
        '_vertices',
        '_costs',
        '_default_value',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'vertices': 'sequence<uint32>',
        'costs': 'sequence<float>',
        'default_value': 'float',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
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
        self.vertices = array.array('I', kwargs.get('vertices', []))
        self.costs = array.array('f', kwargs.get('costs', []))
        self.default_value = kwargs.get('default_value', float())

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
        if self.costs != other.costs:
            return False
        if self.default_value != other.default_value:
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
            if isinstance(value, array.array):
                assert value.typecode == 'I', \
                    "The 'vertices' array.array() must have the type code of 'I'"
                self._vertices = value
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
                "The 'vertices' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 4294967295]"
        self._vertices = array.array('I', value)

    @builtins.property
    def costs(self):
        """Message field 'costs'."""
        return self._costs

    @costs.setter
    def costs(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'f', \
                    "The 'costs' array.array() must have the type code of 'f'"
                self._costs = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'costs' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._costs = array.array('f', value)

    @builtins.property
    def default_value(self):
        """Message field 'default_value'."""
        return self._default_value

    @default_value.setter
    def default_value(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'default_value' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'default_value' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._default_value = value
