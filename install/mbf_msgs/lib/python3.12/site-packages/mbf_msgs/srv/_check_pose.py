# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mbf_msgs:srv/CheckPose.idl
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


class Metaclass_CheckPose_Request(type):
    """Metaclass of message 'CheckPose_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'LOCAL_COSTMAP': 1,
        'GLOBAL_COSTMAP': 2,
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
                'mbf_msgs.srv.CheckPose_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__check_pose__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__check_pose__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__check_pose__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__check_pose__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__check_pose__request

            from geometry_msgs.msg import PoseStamped
            if PoseStamped.__class__._TYPE_SUPPORT is None:
                PoseStamped.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'LOCAL_COSTMAP': cls.__constants['LOCAL_COSTMAP'],
            'GLOBAL_COSTMAP': cls.__constants['GLOBAL_COSTMAP'],
        }

    @property
    def LOCAL_COSTMAP(self):
        """Message constant 'LOCAL_COSTMAP'."""
        return Metaclass_CheckPose_Request.__constants['LOCAL_COSTMAP']

    @property
    def GLOBAL_COSTMAP(self):
        """Message constant 'GLOBAL_COSTMAP'."""
        return Metaclass_CheckPose_Request.__constants['GLOBAL_COSTMAP']


class CheckPose_Request(metaclass=Metaclass_CheckPose_Request):
    """
    Message class 'CheckPose_Request'.

    Constants:
      LOCAL_COSTMAP
      GLOBAL_COSTMAP
    """

    __slots__ = [
        '_pose',
        '_safety_dist',
        '_lethal_cost_mult',
        '_inscrib_cost_mult',
        '_unknown_cost_mult',
        '_costmap',
        '_current_pose',
        '_use_padded_fp',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'pose': 'geometry_msgs/PoseStamped',
        'safety_dist': 'float',
        'lethal_cost_mult': 'float',
        'inscrib_cost_mult': 'float',
        'unknown_cost_mult': 'float',
        'costmap': 'uint8',
        'current_pose': 'boolean',
        'use_padded_fp': 'boolean',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
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
        from geometry_msgs.msg import PoseStamped
        self.pose = kwargs.get('pose', PoseStamped())
        self.safety_dist = kwargs.get('safety_dist', float())
        self.lethal_cost_mult = kwargs.get('lethal_cost_mult', float())
        self.inscrib_cost_mult = kwargs.get('inscrib_cost_mult', float())
        self.unknown_cost_mult = kwargs.get('unknown_cost_mult', float())
        self.costmap = kwargs.get('costmap', int())
        self.current_pose = kwargs.get('current_pose', bool())
        self.use_padded_fp = kwargs.get('use_padded_fp', bool())

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
        if self.pose != other.pose:
            return False
        if self.safety_dist != other.safety_dist:
            return False
        if self.lethal_cost_mult != other.lethal_cost_mult:
            return False
        if self.inscrib_cost_mult != other.inscrib_cost_mult:
            return False
        if self.unknown_cost_mult != other.unknown_cost_mult:
            return False
        if self.costmap != other.costmap:
            return False
        if self.current_pose != other.current_pose:
            return False
        if self.use_padded_fp != other.use_padded_fp:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def pose(self):
        """Message field 'pose'."""
        return self._pose

    @pose.setter
    def pose(self, value):
        if self._check_fields:
            from geometry_msgs.msg import PoseStamped
            assert \
                isinstance(value, PoseStamped), \
                "The 'pose' field must be a sub message of type 'PoseStamped'"
        self._pose = value

    @builtins.property
    def safety_dist(self):
        """Message field 'safety_dist'."""
        return self._safety_dist

    @safety_dist.setter
    def safety_dist(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'safety_dist' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'safety_dist' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._safety_dist = value

    @builtins.property
    def lethal_cost_mult(self):
        """Message field 'lethal_cost_mult'."""
        return self._lethal_cost_mult

    @lethal_cost_mult.setter
    def lethal_cost_mult(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'lethal_cost_mult' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'lethal_cost_mult' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._lethal_cost_mult = value

    @builtins.property
    def inscrib_cost_mult(self):
        """Message field 'inscrib_cost_mult'."""
        return self._inscrib_cost_mult

    @inscrib_cost_mult.setter
    def inscrib_cost_mult(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'inscrib_cost_mult' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'inscrib_cost_mult' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._inscrib_cost_mult = value

    @builtins.property
    def unknown_cost_mult(self):
        """Message field 'unknown_cost_mult'."""
        return self._unknown_cost_mult

    @unknown_cost_mult.setter
    def unknown_cost_mult(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'unknown_cost_mult' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'unknown_cost_mult' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._unknown_cost_mult = value

    @builtins.property
    def costmap(self):
        """Message field 'costmap'."""
        return self._costmap

    @costmap.setter
    def costmap(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'costmap' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'costmap' field must be an unsigned integer in [0, 255]"
        self._costmap = value

    @builtins.property
    def current_pose(self):
        """Message field 'current_pose'."""
        return self._current_pose

    @current_pose.setter
    def current_pose(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'current_pose' field must be of type 'bool'"
        self._current_pose = value

    @builtins.property
    def use_padded_fp(self):
        """Message field 'use_padded_fp'."""
        return self._use_padded_fp

    @use_padded_fp.setter
    def use_padded_fp(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'use_padded_fp' field must be of type 'bool'"
        self._use_padded_fp = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_CheckPose_Response(type):
    """Metaclass of message 'CheckPose_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'FREE': 0,
        'INSCRIBED': 1,
        'LETHAL': 2,
        'UNKNOWN': 3,
        'OUTSIDE': 4,
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
                'mbf_msgs.srv.CheckPose_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__check_pose__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__check_pose__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__check_pose__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__check_pose__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__check_pose__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'FREE': cls.__constants['FREE'],
            'INSCRIBED': cls.__constants['INSCRIBED'],
            'LETHAL': cls.__constants['LETHAL'],
            'UNKNOWN': cls.__constants['UNKNOWN'],
            'OUTSIDE': cls.__constants['OUTSIDE'],
        }

    @property
    def FREE(self):
        """Message constant 'FREE'."""
        return Metaclass_CheckPose_Response.__constants['FREE']

    @property
    def INSCRIBED(self):
        """Message constant 'INSCRIBED'."""
        return Metaclass_CheckPose_Response.__constants['INSCRIBED']

    @property
    def LETHAL(self):
        """Message constant 'LETHAL'."""
        return Metaclass_CheckPose_Response.__constants['LETHAL']

    @property
    def UNKNOWN(self):
        """Message constant 'UNKNOWN'."""
        return Metaclass_CheckPose_Response.__constants['UNKNOWN']

    @property
    def OUTSIDE(self):
        """Message constant 'OUTSIDE'."""
        return Metaclass_CheckPose_Response.__constants['OUTSIDE']


class CheckPose_Response(metaclass=Metaclass_CheckPose_Response):
    """
    Message class 'CheckPose_Response'.

    Constants:
      FREE
      INSCRIBED
      LETHAL
      UNKNOWN
      OUTSIDE
    """

    __slots__ = [
        '_state',
        '_cost',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'state': 'uint8',
        'cost': 'uint32',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
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
        self.state = kwargs.get('state', int())
        self.cost = kwargs.get('cost', int())

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
        if self.state != other.state:
            return False
        if self.cost != other.cost:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'state' field must be an unsigned integer in [0, 255]"
        self._state = value

    @builtins.property
    def cost(self):
        """Message field 'cost'."""
        return self._cost

    @cost.setter
    def cost(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'cost' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'cost' field must be an unsigned integer in [0, 4294967295]"
        self._cost = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_CheckPose_Event(type):
    """Metaclass of message 'CheckPose_Event'."""

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
                'mbf_msgs.srv.CheckPose_Event')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__check_pose__event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__check_pose__event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__check_pose__event
            cls._TYPE_SUPPORT = module.type_support_msg__srv__check_pose__event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__check_pose__event

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


class CheckPose_Event(metaclass=Metaclass_CheckPose_Event):
    """Message class 'CheckPose_Event'."""

    __slots__ = [
        '_info',
        '_request',
        '_response',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'info': 'service_msgs/ServiceEventInfo',
        'request': 'sequence<mbf_msgs/CheckPose_Request, 1>',
        'response': 'sequence<mbf_msgs/CheckPose_Response, 1>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['service_msgs', 'msg'], 'ServiceEventInfo'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['mbf_msgs', 'srv'], 'CheckPose_Request'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['mbf_msgs', 'srv'], 'CheckPose_Response'), 1),  # noqa: E501
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
            from mbf_msgs.srv import CheckPose_Request
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
                 all(isinstance(v, CheckPose_Request) for v in value) and
                 True), \
                "The 'request' field must be a set or sequence with length <= 1 and each value of type 'CheckPose_Request'"
        self._request = value

    @builtins.property
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if self._check_fields:
            from mbf_msgs.srv import CheckPose_Response
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
                 all(isinstance(v, CheckPose_Response) for v in value) and
                 True), \
                "The 'response' field must be a set or sequence with length <= 1 and each value of type 'CheckPose_Response'"
        self._response = value


class Metaclass_CheckPose(type):
    """Metaclass of service 'CheckPose'."""

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
                'mbf_msgs.srv.CheckPose')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__check_pose

            from mbf_msgs.srv import _check_pose
            if _check_pose.Metaclass_CheckPose_Request._TYPE_SUPPORT is None:
                _check_pose.Metaclass_CheckPose_Request.__import_type_support__()
            if _check_pose.Metaclass_CheckPose_Response._TYPE_SUPPORT is None:
                _check_pose.Metaclass_CheckPose_Response.__import_type_support__()
            if _check_pose.Metaclass_CheckPose_Event._TYPE_SUPPORT is None:
                _check_pose.Metaclass_CheckPose_Event.__import_type_support__()


class CheckPose(metaclass=Metaclass_CheckPose):
    from mbf_msgs.srv._check_pose import CheckPose_Request as Request
    from mbf_msgs.srv._check_pose import CheckPose_Response as Response
    from mbf_msgs.srv._check_pose import CheckPose_Event as Event

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
