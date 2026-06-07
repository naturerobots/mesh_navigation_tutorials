# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mbf_msgs:action/ExePath.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ExePath_Goal(type):
    """Metaclass of message 'ExePath_Goal'."""

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
            module = import_type_support('mbf_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mbf_msgs.action.ExePath_Goal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__exe_path__goal
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__exe_path__goal
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__exe_path__goal
            cls._TYPE_SUPPORT = module.type_support_msg__action__exe_path__goal
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__exe_path__goal

            from nav_msgs.msg import Path
            if Path.__class__._TYPE_SUPPORT is None:
                Path.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExePath_Goal(metaclass=Metaclass_ExePath_Goal):
    """Message class 'ExePath_Goal'."""

    __slots__ = [
        '_path',
        '_controller',
        '_concurrency_slot',
        '_tolerance_from_action',
        '_dist_tolerance',
        '_angle_tolerance',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'path': 'nav_msgs/Path',
        'controller': 'string',
        'concurrency_slot': 'uint8',
        'tolerance_from_action': 'boolean',
        'dist_tolerance': 'float',
        'angle_tolerance': 'float',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['nav_msgs', 'msg'], 'Path'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
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
        from nav_msgs.msg import Path
        self.path = kwargs.get('path', Path())
        self.controller = kwargs.get('controller', str())
        self.concurrency_slot = kwargs.get('concurrency_slot', int())
        self.tolerance_from_action = kwargs.get('tolerance_from_action', bool())
        self.dist_tolerance = kwargs.get('dist_tolerance', float())
        self.angle_tolerance = kwargs.get('angle_tolerance', float())

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
        if self.path != other.path:
            return False
        if self.controller != other.controller:
            return False
        if self.concurrency_slot != other.concurrency_slot:
            return False
        if self.tolerance_from_action != other.tolerance_from_action:
            return False
        if self.dist_tolerance != other.dist_tolerance:
            return False
        if self.angle_tolerance != other.angle_tolerance:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def path(self):
        """Message field 'path'."""
        return self._path

    @path.setter
    def path(self, value):
        if self._check_fields:
            from nav_msgs.msg import Path
            assert \
                isinstance(value, Path), \
                "The 'path' field must be a sub message of type 'Path'"
        self._path = value

    @builtins.property
    def controller(self):
        """Message field 'controller'."""
        return self._controller

    @controller.setter
    def controller(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'controller' field must be of type 'str'"
        self._controller = value

    @builtins.property
    def concurrency_slot(self):
        """Message field 'concurrency_slot'."""
        return self._concurrency_slot

    @concurrency_slot.setter
    def concurrency_slot(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'concurrency_slot' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'concurrency_slot' field must be an unsigned integer in [0, 255]"
        self._concurrency_slot = value

    @builtins.property
    def tolerance_from_action(self):
        """Message field 'tolerance_from_action'."""
        return self._tolerance_from_action

    @tolerance_from_action.setter
    def tolerance_from_action(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'tolerance_from_action' field must be of type 'bool'"
        self._tolerance_from_action = value

    @builtins.property
    def dist_tolerance(self):
        """Message field 'dist_tolerance'."""
        return self._dist_tolerance

    @dist_tolerance.setter
    def dist_tolerance(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'dist_tolerance' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'dist_tolerance' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._dist_tolerance = value

    @builtins.property
    def angle_tolerance(self):
        """Message field 'angle_tolerance'."""
        return self._angle_tolerance

    @angle_tolerance.setter
    def angle_tolerance(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'angle_tolerance' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'angle_tolerance' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._angle_tolerance = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_ExePath_Result(type):
    """Metaclass of message 'ExePath_Result'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'SUCCESS': 0,
        'FAILURE': 100,
        'CANCELED': 101,
        'NO_VALID_CMD': 102,
        'PAT_EXCEEDED': 103,
        'COLLISION': 104,
        'OSCILLATION': 105,
        'ROBOT_STUCK': 106,
        'MISSED_GOAL': 107,
        'MISSED_PATH': 108,
        'BLOCKED_GOAL': 109,
        'BLOCKED_PATH': 110,
        'INVALID_PATH': 111,
        'TF_ERROR': 112,
        'NOT_INITIALIZED': 113,
        'INVALID_PLUGIN': 114,
        'INTERNAL_ERROR': 115,
        'OUT_OF_MAP': 116,
        'MAP_ERROR': 117,
        'STOPPED': 118,
        'ERROR_RANGE_START': 100,
        'ERROR_RANGE_END': 149,
        'PLUGIN_ERROR_RANGE_START': 121,
        'PLUGIN_ERROR_RANGE_END': 149,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('mbf_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mbf_msgs.action.ExePath_Result')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__exe_path__result
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__exe_path__result
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__exe_path__result
            cls._TYPE_SUPPORT = module.type_support_msg__action__exe_path__result
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__exe_path__result

            from geometry_msgs.msg import PoseStamped
            if PoseStamped.__class__._TYPE_SUPPORT is None:
                PoseStamped.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'SUCCESS': cls.__constants['SUCCESS'],
            'FAILURE': cls.__constants['FAILURE'],
            'CANCELED': cls.__constants['CANCELED'],
            'NO_VALID_CMD': cls.__constants['NO_VALID_CMD'],
            'PAT_EXCEEDED': cls.__constants['PAT_EXCEEDED'],
            'COLLISION': cls.__constants['COLLISION'],
            'OSCILLATION': cls.__constants['OSCILLATION'],
            'ROBOT_STUCK': cls.__constants['ROBOT_STUCK'],
            'MISSED_GOAL': cls.__constants['MISSED_GOAL'],
            'MISSED_PATH': cls.__constants['MISSED_PATH'],
            'BLOCKED_GOAL': cls.__constants['BLOCKED_GOAL'],
            'BLOCKED_PATH': cls.__constants['BLOCKED_PATH'],
            'INVALID_PATH': cls.__constants['INVALID_PATH'],
            'TF_ERROR': cls.__constants['TF_ERROR'],
            'NOT_INITIALIZED': cls.__constants['NOT_INITIALIZED'],
            'INVALID_PLUGIN': cls.__constants['INVALID_PLUGIN'],
            'INTERNAL_ERROR': cls.__constants['INTERNAL_ERROR'],
            'OUT_OF_MAP': cls.__constants['OUT_OF_MAP'],
            'MAP_ERROR': cls.__constants['MAP_ERROR'],
            'STOPPED': cls.__constants['STOPPED'],
            'ERROR_RANGE_START': cls.__constants['ERROR_RANGE_START'],
            'ERROR_RANGE_END': cls.__constants['ERROR_RANGE_END'],
            'PLUGIN_ERROR_RANGE_START': cls.__constants['PLUGIN_ERROR_RANGE_START'],
            'PLUGIN_ERROR_RANGE_END': cls.__constants['PLUGIN_ERROR_RANGE_END'],
        }

    @property
    def SUCCESS(self):
        """Message constant 'SUCCESS'."""
        return Metaclass_ExePath_Result.__constants['SUCCESS']

    @property
    def FAILURE(self):
        """Message constant 'FAILURE'."""
        return Metaclass_ExePath_Result.__constants['FAILURE']

    @property
    def CANCELED(self):
        """Message constant 'CANCELED'."""
        return Metaclass_ExePath_Result.__constants['CANCELED']

    @property
    def NO_VALID_CMD(self):
        """Message constant 'NO_VALID_CMD'."""
        return Metaclass_ExePath_Result.__constants['NO_VALID_CMD']

    @property
    def PAT_EXCEEDED(self):
        """Message constant 'PAT_EXCEEDED'."""
        return Metaclass_ExePath_Result.__constants['PAT_EXCEEDED']

    @property
    def COLLISION(self):
        """Message constant 'COLLISION'."""
        return Metaclass_ExePath_Result.__constants['COLLISION']

    @property
    def OSCILLATION(self):
        """Message constant 'OSCILLATION'."""
        return Metaclass_ExePath_Result.__constants['OSCILLATION']

    @property
    def ROBOT_STUCK(self):
        """Message constant 'ROBOT_STUCK'."""
        return Metaclass_ExePath_Result.__constants['ROBOT_STUCK']

    @property
    def MISSED_GOAL(self):
        """Message constant 'MISSED_GOAL'."""
        return Metaclass_ExePath_Result.__constants['MISSED_GOAL']

    @property
    def MISSED_PATH(self):
        """Message constant 'MISSED_PATH'."""
        return Metaclass_ExePath_Result.__constants['MISSED_PATH']

    @property
    def BLOCKED_GOAL(self):
        """Message constant 'BLOCKED_GOAL'."""
        return Metaclass_ExePath_Result.__constants['BLOCKED_GOAL']

    @property
    def BLOCKED_PATH(self):
        """Message constant 'BLOCKED_PATH'."""
        return Metaclass_ExePath_Result.__constants['BLOCKED_PATH']

    @property
    def INVALID_PATH(self):
        """Message constant 'INVALID_PATH'."""
        return Metaclass_ExePath_Result.__constants['INVALID_PATH']

    @property
    def TF_ERROR(self):
        """Message constant 'TF_ERROR'."""
        return Metaclass_ExePath_Result.__constants['TF_ERROR']

    @property
    def NOT_INITIALIZED(self):
        """Message constant 'NOT_INITIALIZED'."""
        return Metaclass_ExePath_Result.__constants['NOT_INITIALIZED']

    @property
    def INVALID_PLUGIN(self):
        """Message constant 'INVALID_PLUGIN'."""
        return Metaclass_ExePath_Result.__constants['INVALID_PLUGIN']

    @property
    def INTERNAL_ERROR(self):
        """Message constant 'INTERNAL_ERROR'."""
        return Metaclass_ExePath_Result.__constants['INTERNAL_ERROR']

    @property
    def OUT_OF_MAP(self):
        """Message constant 'OUT_OF_MAP'."""
        return Metaclass_ExePath_Result.__constants['OUT_OF_MAP']

    @property
    def MAP_ERROR(self):
        """Message constant 'MAP_ERROR'."""
        return Metaclass_ExePath_Result.__constants['MAP_ERROR']

    @property
    def STOPPED(self):
        """Message constant 'STOPPED'."""
        return Metaclass_ExePath_Result.__constants['STOPPED']

    @property
    def ERROR_RANGE_START(self):
        """Message constant 'ERROR_RANGE_START'."""
        return Metaclass_ExePath_Result.__constants['ERROR_RANGE_START']

    @property
    def ERROR_RANGE_END(self):
        """Message constant 'ERROR_RANGE_END'."""
        return Metaclass_ExePath_Result.__constants['ERROR_RANGE_END']

    @property
    def PLUGIN_ERROR_RANGE_START(self):
        """Message constant 'PLUGIN_ERROR_RANGE_START'."""
        return Metaclass_ExePath_Result.__constants['PLUGIN_ERROR_RANGE_START']

    @property
    def PLUGIN_ERROR_RANGE_END(self):
        """Message constant 'PLUGIN_ERROR_RANGE_END'."""
        return Metaclass_ExePath_Result.__constants['PLUGIN_ERROR_RANGE_END']


class ExePath_Result(metaclass=Metaclass_ExePath_Result):
    """
    Message class 'ExePath_Result'.

    Constants:
      SUCCESS
      FAILURE
      CANCELED
      NO_VALID_CMD
      PAT_EXCEEDED
      COLLISION
      OSCILLATION
      ROBOT_STUCK
      MISSED_GOAL
      MISSED_PATH
      BLOCKED_GOAL
      BLOCKED_PATH
      INVALID_PATH
      TF_ERROR
      NOT_INITIALIZED
      INVALID_PLUGIN
      INTERNAL_ERROR
      OUT_OF_MAP
      MAP_ERROR
      STOPPED
      ERROR_RANGE_START
      ERROR_RANGE_END
      PLUGIN_ERROR_RANGE_START
      PLUGIN_ERROR_RANGE_END
    """

    __slots__ = [
        '_outcome',
        '_message',
        '_final_pose',
        '_dist_to_goal',
        '_angle_to_goal',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'outcome': 'uint32',
        'message': 'string',
        'final_pose': 'geometry_msgs/PoseStamped',
        'dist_to_goal': 'float',
        'angle_to_goal': 'float',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
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
        self.outcome = kwargs.get('outcome', int())
        self.message = kwargs.get('message', str())
        from geometry_msgs.msg import PoseStamped
        self.final_pose = kwargs.get('final_pose', PoseStamped())
        self.dist_to_goal = kwargs.get('dist_to_goal', float())
        self.angle_to_goal = kwargs.get('angle_to_goal', float())

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
        if self.outcome != other.outcome:
            return False
        if self.message != other.message:
            return False
        if self.final_pose != other.final_pose:
            return False
        if self.dist_to_goal != other.dist_to_goal:
            return False
        if self.angle_to_goal != other.angle_to_goal:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def outcome(self):
        """Message field 'outcome'."""
        return self._outcome

    @outcome.setter
    def outcome(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'outcome' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'outcome' field must be an unsigned integer in [0, 4294967295]"
        self._outcome = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value

    @builtins.property
    def final_pose(self):
        """Message field 'final_pose'."""
        return self._final_pose

    @final_pose.setter
    def final_pose(self, value):
        if self._check_fields:
            from geometry_msgs.msg import PoseStamped
            assert \
                isinstance(value, PoseStamped), \
                "The 'final_pose' field must be a sub message of type 'PoseStamped'"
        self._final_pose = value

    @builtins.property
    def dist_to_goal(self):
        """Message field 'dist_to_goal'."""
        return self._dist_to_goal

    @dist_to_goal.setter
    def dist_to_goal(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'dist_to_goal' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'dist_to_goal' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._dist_to_goal = value

    @builtins.property
    def angle_to_goal(self):
        """Message field 'angle_to_goal'."""
        return self._angle_to_goal

    @angle_to_goal.setter
    def angle_to_goal(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'angle_to_goal' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'angle_to_goal' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._angle_to_goal = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_ExePath_Feedback(type):
    """Metaclass of message 'ExePath_Feedback'."""

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
            module = import_type_support('mbf_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mbf_msgs.action.ExePath_Feedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__exe_path__feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__exe_path__feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__exe_path__feedback
            cls._TYPE_SUPPORT = module.type_support_msg__action__exe_path__feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__exe_path__feedback

            from geometry_msgs.msg import PoseStamped
            if PoseStamped.__class__._TYPE_SUPPORT is None:
                PoseStamped.__class__.__import_type_support__()

            from geometry_msgs.msg import TwistStamped
            if TwistStamped.__class__._TYPE_SUPPORT is None:
                TwistStamped.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExePath_Feedback(metaclass=Metaclass_ExePath_Feedback):
    """Message class 'ExePath_Feedback'."""

    __slots__ = [
        '_outcome',
        '_message',
        '_dist_to_goal',
        '_angle_to_goal',
        '_current_pose',
        '_last_cmd_vel',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'outcome': 'uint32',
        'message': 'string',
        'dist_to_goal': 'float',
        'angle_to_goal': 'float',
        'current_pose': 'geometry_msgs/PoseStamped',
        'last_cmd_vel': 'geometry_msgs/TwistStamped',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'TwistStamped'),  # noqa: E501
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
        self.outcome = kwargs.get('outcome', int())
        self.message = kwargs.get('message', str())
        self.dist_to_goal = kwargs.get('dist_to_goal', float())
        self.angle_to_goal = kwargs.get('angle_to_goal', float())
        from geometry_msgs.msg import PoseStamped
        self.current_pose = kwargs.get('current_pose', PoseStamped())
        from geometry_msgs.msg import TwistStamped
        self.last_cmd_vel = kwargs.get('last_cmd_vel', TwistStamped())

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
        if self.outcome != other.outcome:
            return False
        if self.message != other.message:
            return False
        if self.dist_to_goal != other.dist_to_goal:
            return False
        if self.angle_to_goal != other.angle_to_goal:
            return False
        if self.current_pose != other.current_pose:
            return False
        if self.last_cmd_vel != other.last_cmd_vel:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def outcome(self):
        """Message field 'outcome'."""
        return self._outcome

    @outcome.setter
    def outcome(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'outcome' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'outcome' field must be an unsigned integer in [0, 4294967295]"
        self._outcome = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value

    @builtins.property
    def dist_to_goal(self):
        """Message field 'dist_to_goal'."""
        return self._dist_to_goal

    @dist_to_goal.setter
    def dist_to_goal(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'dist_to_goal' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'dist_to_goal' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._dist_to_goal = value

    @builtins.property
    def angle_to_goal(self):
        """Message field 'angle_to_goal'."""
        return self._angle_to_goal

    @angle_to_goal.setter
    def angle_to_goal(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'angle_to_goal' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'angle_to_goal' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._angle_to_goal = value

    @builtins.property
    def current_pose(self):
        """Message field 'current_pose'."""
        return self._current_pose

    @current_pose.setter
    def current_pose(self, value):
        if self._check_fields:
            from geometry_msgs.msg import PoseStamped
            assert \
                isinstance(value, PoseStamped), \
                "The 'current_pose' field must be a sub message of type 'PoseStamped'"
        self._current_pose = value

    @builtins.property
    def last_cmd_vel(self):
        """Message field 'last_cmd_vel'."""
        return self._last_cmd_vel

    @last_cmd_vel.setter
    def last_cmd_vel(self, value):
        if self._check_fields:
            from geometry_msgs.msg import TwistStamped
            assert \
                isinstance(value, TwistStamped), \
                "The 'last_cmd_vel' field must be a sub message of type 'TwistStamped'"
        self._last_cmd_vel = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ExePath_SendGoal_Request(type):
    """Metaclass of message 'ExePath_SendGoal_Request'."""

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
            module = import_type_support('mbf_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mbf_msgs.action.ExePath_SendGoal_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__exe_path__send_goal__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__exe_path__send_goal__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__exe_path__send_goal__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__exe_path__send_goal__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__exe_path__send_goal__request

            from mbf_msgs.action import ExePath
            if ExePath.Goal.__class__._TYPE_SUPPORT is None:
                ExePath.Goal.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExePath_SendGoal_Request(metaclass=Metaclass_ExePath_SendGoal_Request):
    """Message class 'ExePath_SendGoal_Request'."""

    __slots__ = [
        '_goal_id',
        '_goal',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'goal': 'mbf_msgs/ExePath_Goal',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['mbf_msgs', 'action'], 'ExePath_Goal'),  # noqa: E501
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
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from mbf_msgs.action._exe_path import ExePath_Goal
        self.goal = kwargs.get('goal', ExePath_Goal())

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
        if self.goal_id != other.goal_id:
            return False
        if self.goal != other.goal:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if self._check_fields:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def goal(self):
        """Message field 'goal'."""
        return self._goal

    @goal.setter
    def goal(self, value):
        if self._check_fields:
            from mbf_msgs.action._exe_path import ExePath_Goal
            assert \
                isinstance(value, ExePath_Goal), \
                "The 'goal' field must be a sub message of type 'ExePath_Goal'"
        self._goal = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ExePath_SendGoal_Response(type):
    """Metaclass of message 'ExePath_SendGoal_Response'."""

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
            module = import_type_support('mbf_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mbf_msgs.action.ExePath_SendGoal_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__exe_path__send_goal__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__exe_path__send_goal__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__exe_path__send_goal__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__exe_path__send_goal__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__exe_path__send_goal__response

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExePath_SendGoal_Response(metaclass=Metaclass_ExePath_SendGoal_Response):
    """Message class 'ExePath_SendGoal_Response'."""

    __slots__ = [
        '_accepted',
        '_stamp',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'accepted': 'boolean',
        'stamp': 'builtin_interfaces/Time',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
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
        self.accepted = kwargs.get('accepted', bool())
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())

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
        if self.accepted != other.accepted:
            return False
        if self.stamp != other.stamp:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def accepted(self):
        """Message field 'accepted'."""
        return self._accepted

    @accepted.setter
    def accepted(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'accepted' field must be of type 'bool'"
        self._accepted = value

    @builtins.property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if self._check_fields:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ExePath_SendGoal_Event(type):
    """Metaclass of message 'ExePath_SendGoal_Event'."""

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
            module = import_type_support('mbf_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mbf_msgs.action.ExePath_SendGoal_Event')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__exe_path__send_goal__event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__exe_path__send_goal__event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__exe_path__send_goal__event
            cls._TYPE_SUPPORT = module.type_support_msg__action__exe_path__send_goal__event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__exe_path__send_goal__event

            from service_msgs.msg import ServiceEventInfo
            if ServiceEventInfo.__class__._TYPE_SUPPORT is None:
                ServiceEventInfo.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExePath_SendGoal_Event(metaclass=Metaclass_ExePath_SendGoal_Event):
    """Message class 'ExePath_SendGoal_Event'."""

    __slots__ = [
        '_info',
        '_request',
        '_response',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'info': 'service_msgs/ServiceEventInfo',
        'request': 'sequence<mbf_msgs/ExePath_SendGoal_Request, 1>',
        'response': 'sequence<mbf_msgs/ExePath_SendGoal_Response, 1>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['service_msgs', 'msg'], 'ServiceEventInfo'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['mbf_msgs', 'action'], 'ExePath_SendGoal_Request'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['mbf_msgs', 'action'], 'ExePath_SendGoal_Response'), 1),  # noqa: E501
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
        from service_msgs.msg import ServiceEventInfo
        self.info = kwargs.get('info', ServiceEventInfo())
        self.request = kwargs.get('request', [])
        self.response = kwargs.get('response', [])

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
        if self.info != other.info:
            return False
        if self.request != other.request:
            return False
        if self.response != other.response:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def info(self):
        """Message field 'info'."""
        return self._info

    @info.setter
    def info(self, value):
        if self._check_fields:
            from service_msgs.msg import ServiceEventInfo
            assert \
                isinstance(value, ServiceEventInfo), \
                "The 'info' field must be a sub message of type 'ServiceEventInfo'"
        self._info = value

    @builtins.property
    def request(self):
        """Message field 'request'."""
        return self._request

    @request.setter
    def request(self, value):
        if self._check_fields:
            from mbf_msgs.action import ExePath_SendGoal_Request
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
                 len(value) <= 1 and
                 all(isinstance(v, ExePath_SendGoal_Request) for v in value) and
                 True), \
                "The 'request' field must be a set or sequence with length <= 1 and each value of type 'ExePath_SendGoal_Request'"
        self._request = value

    @builtins.property
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if self._check_fields:
            from mbf_msgs.action import ExePath_SendGoal_Response
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
                 len(value) <= 1 and
                 all(isinstance(v, ExePath_SendGoal_Response) for v in value) and
                 True), \
                "The 'response' field must be a set or sequence with length <= 1 and each value of type 'ExePath_SendGoal_Response'"
        self._response = value


class Metaclass_ExePath_SendGoal(type):
    """Metaclass of service 'ExePath_SendGoal'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('mbf_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mbf_msgs.action.ExePath_SendGoal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__exe_path__send_goal

            from mbf_msgs.action import _exe_path
            if _exe_path.Metaclass_ExePath_SendGoal_Request._TYPE_SUPPORT is None:
                _exe_path.Metaclass_ExePath_SendGoal_Request.__import_type_support__()
            if _exe_path.Metaclass_ExePath_SendGoal_Response._TYPE_SUPPORT is None:
                _exe_path.Metaclass_ExePath_SendGoal_Response.__import_type_support__()
            if _exe_path.Metaclass_ExePath_SendGoal_Event._TYPE_SUPPORT is None:
                _exe_path.Metaclass_ExePath_SendGoal_Event.__import_type_support__()


class ExePath_SendGoal(metaclass=Metaclass_ExePath_SendGoal):
    from mbf_msgs.action._exe_path import ExePath_SendGoal_Request as Request
    from mbf_msgs.action._exe_path import ExePath_SendGoal_Response as Response
    from mbf_msgs.action._exe_path import ExePath_SendGoal_Event as Event

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ExePath_GetResult_Request(type):
    """Metaclass of message 'ExePath_GetResult_Request'."""

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
            module = import_type_support('mbf_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mbf_msgs.action.ExePath_GetResult_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__exe_path__get_result__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__exe_path__get_result__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__exe_path__get_result__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__exe_path__get_result__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__exe_path__get_result__request

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExePath_GetResult_Request(metaclass=Metaclass_ExePath_GetResult_Request):
    """Message class 'ExePath_GetResult_Request'."""

    __slots__ = [
        '_goal_id',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
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
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())

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
        if self.goal_id != other.goal_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if self._check_fields:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ExePath_GetResult_Response(type):
    """Metaclass of message 'ExePath_GetResult_Response'."""

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
            module = import_type_support('mbf_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mbf_msgs.action.ExePath_GetResult_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__exe_path__get_result__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__exe_path__get_result__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__exe_path__get_result__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__exe_path__get_result__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__exe_path__get_result__response

            from mbf_msgs.action import ExePath
            if ExePath.Result.__class__._TYPE_SUPPORT is None:
                ExePath.Result.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExePath_GetResult_Response(metaclass=Metaclass_ExePath_GetResult_Response):
    """Message class 'ExePath_GetResult_Response'."""

    __slots__ = [
        '_status',
        '_result',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'status': 'int8',
        'result': 'mbf_msgs/ExePath_Result',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['mbf_msgs', 'action'], 'ExePath_Result'),  # noqa: E501
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
        self.status = kwargs.get('status', int())
        from mbf_msgs.action._exe_path import ExePath_Result
        self.result = kwargs.get('result', ExePath_Result())

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
        if self.status != other.status:
            return False
        if self.result != other.result:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'status' field must be an integer in [-128, 127]"
        self._status = value

    @builtins.property
    def result(self):
        """Message field 'result'."""
        return self._result

    @result.setter
    def result(self, value):
        if self._check_fields:
            from mbf_msgs.action._exe_path import ExePath_Result
            assert \
                isinstance(value, ExePath_Result), \
                "The 'result' field must be a sub message of type 'ExePath_Result'"
        self._result = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ExePath_GetResult_Event(type):
    """Metaclass of message 'ExePath_GetResult_Event'."""

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
            module = import_type_support('mbf_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mbf_msgs.action.ExePath_GetResult_Event')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__exe_path__get_result__event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__exe_path__get_result__event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__exe_path__get_result__event
            cls._TYPE_SUPPORT = module.type_support_msg__action__exe_path__get_result__event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__exe_path__get_result__event

            from service_msgs.msg import ServiceEventInfo
            if ServiceEventInfo.__class__._TYPE_SUPPORT is None:
                ServiceEventInfo.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExePath_GetResult_Event(metaclass=Metaclass_ExePath_GetResult_Event):
    """Message class 'ExePath_GetResult_Event'."""

    __slots__ = [
        '_info',
        '_request',
        '_response',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'info': 'service_msgs/ServiceEventInfo',
        'request': 'sequence<mbf_msgs/ExePath_GetResult_Request, 1>',
        'response': 'sequence<mbf_msgs/ExePath_GetResult_Response, 1>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['service_msgs', 'msg'], 'ServiceEventInfo'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['mbf_msgs', 'action'], 'ExePath_GetResult_Request'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['mbf_msgs', 'action'], 'ExePath_GetResult_Response'), 1),  # noqa: E501
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
        from service_msgs.msg import ServiceEventInfo
        self.info = kwargs.get('info', ServiceEventInfo())
        self.request = kwargs.get('request', [])
        self.response = kwargs.get('response', [])

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
        if self.info != other.info:
            return False
        if self.request != other.request:
            return False
        if self.response != other.response:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def info(self):
        """Message field 'info'."""
        return self._info

    @info.setter
    def info(self, value):
        if self._check_fields:
            from service_msgs.msg import ServiceEventInfo
            assert \
                isinstance(value, ServiceEventInfo), \
                "The 'info' field must be a sub message of type 'ServiceEventInfo'"
        self._info = value

    @builtins.property
    def request(self):
        """Message field 'request'."""
        return self._request

    @request.setter
    def request(self, value):
        if self._check_fields:
            from mbf_msgs.action import ExePath_GetResult_Request
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
                 len(value) <= 1 and
                 all(isinstance(v, ExePath_GetResult_Request) for v in value) and
                 True), \
                "The 'request' field must be a set or sequence with length <= 1 and each value of type 'ExePath_GetResult_Request'"
        self._request = value

    @builtins.property
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if self._check_fields:
            from mbf_msgs.action import ExePath_GetResult_Response
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
                 len(value) <= 1 and
                 all(isinstance(v, ExePath_GetResult_Response) for v in value) and
                 True), \
                "The 'response' field must be a set or sequence with length <= 1 and each value of type 'ExePath_GetResult_Response'"
        self._response = value


class Metaclass_ExePath_GetResult(type):
    """Metaclass of service 'ExePath_GetResult'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('mbf_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mbf_msgs.action.ExePath_GetResult')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__exe_path__get_result

            from mbf_msgs.action import _exe_path
            if _exe_path.Metaclass_ExePath_GetResult_Request._TYPE_SUPPORT is None:
                _exe_path.Metaclass_ExePath_GetResult_Request.__import_type_support__()
            if _exe_path.Metaclass_ExePath_GetResult_Response._TYPE_SUPPORT is None:
                _exe_path.Metaclass_ExePath_GetResult_Response.__import_type_support__()
            if _exe_path.Metaclass_ExePath_GetResult_Event._TYPE_SUPPORT is None:
                _exe_path.Metaclass_ExePath_GetResult_Event.__import_type_support__()


class ExePath_GetResult(metaclass=Metaclass_ExePath_GetResult):
    from mbf_msgs.action._exe_path import ExePath_GetResult_Request as Request
    from mbf_msgs.action._exe_path import ExePath_GetResult_Response as Response
    from mbf_msgs.action._exe_path import ExePath_GetResult_Event as Event

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ExePath_FeedbackMessage(type):
    """Metaclass of message 'ExePath_FeedbackMessage'."""

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
            module = import_type_support('mbf_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mbf_msgs.action.ExePath_FeedbackMessage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__exe_path__feedback_message
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__exe_path__feedback_message
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__exe_path__feedback_message
            cls._TYPE_SUPPORT = module.type_support_msg__action__exe_path__feedback_message
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__exe_path__feedback_message

            from mbf_msgs.action import ExePath
            if ExePath.Feedback.__class__._TYPE_SUPPORT is None:
                ExePath.Feedback.__class__.__import_type_support__()

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExePath_FeedbackMessage(metaclass=Metaclass_ExePath_FeedbackMessage):
    """Message class 'ExePath_FeedbackMessage'."""

    __slots__ = [
        '_goal_id',
        '_feedback',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'feedback': 'mbf_msgs/ExePath_Feedback',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['mbf_msgs', 'action'], 'ExePath_Feedback'),  # noqa: E501
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
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from mbf_msgs.action._exe_path import ExePath_Feedback
        self.feedback = kwargs.get('feedback', ExePath_Feedback())

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
        if self.goal_id != other.goal_id:
            return False
        if self.feedback != other.feedback:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if self._check_fields:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def feedback(self):
        """Message field 'feedback'."""
        return self._feedback

    @feedback.setter
    def feedback(self, value):
        if self._check_fields:
            from mbf_msgs.action._exe_path import ExePath_Feedback
            assert \
                isinstance(value, ExePath_Feedback), \
                "The 'feedback' field must be a sub message of type 'ExePath_Feedback'"
        self._feedback = value


class Metaclass_ExePath(type):
    """Metaclass of action 'ExePath'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('mbf_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mbf_msgs.action.ExePath')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_action__action__exe_path

            from action_msgs.msg import _goal_status_array
            if _goal_status_array.Metaclass_GoalStatusArray._TYPE_SUPPORT is None:
                _goal_status_array.Metaclass_GoalStatusArray.__import_type_support__()
            from action_msgs.srv import _cancel_goal
            if _cancel_goal.Metaclass_CancelGoal._TYPE_SUPPORT is None:
                _cancel_goal.Metaclass_CancelGoal.__import_type_support__()

            from mbf_msgs.action import _exe_path
            if _exe_path.Metaclass_ExePath_SendGoal._TYPE_SUPPORT is None:
                _exe_path.Metaclass_ExePath_SendGoal.__import_type_support__()
            if _exe_path.Metaclass_ExePath_GetResult._TYPE_SUPPORT is None:
                _exe_path.Metaclass_ExePath_GetResult.__import_type_support__()
            if _exe_path.Metaclass_ExePath_FeedbackMessage._TYPE_SUPPORT is None:
                _exe_path.Metaclass_ExePath_FeedbackMessage.__import_type_support__()


class ExePath(metaclass=Metaclass_ExePath):

    # The goal message defined in the action definition.
    from mbf_msgs.action._exe_path import ExePath_Goal as Goal
    # The result message defined in the action definition.
    from mbf_msgs.action._exe_path import ExePath_Result as Result
    # The feedback message defined in the action definition.
    from mbf_msgs.action._exe_path import ExePath_Feedback as Feedback

    class Impl:

        # The send_goal service using a wrapped version of the goal message as a request.
        from mbf_msgs.action._exe_path import ExePath_SendGoal as SendGoalService
        # The get_result service using a wrapped version of the result message as a response.
        from mbf_msgs.action._exe_path import ExePath_GetResult as GetResultService
        # The feedback message with generic fields which wraps the feedback message.
        from mbf_msgs.action._exe_path import ExePath_FeedbackMessage as FeedbackMessage

        # The generic service to cancel a goal.
        from action_msgs.srv._cancel_goal import CancelGoal as CancelGoalService
        # The generic message for get the status of a goal.
        from action_msgs.msg._goal_status_array import GoalStatusArray as GoalStatusMessage

    def __init__(self):
        raise NotImplementedError('Action classes can not be instantiated')
