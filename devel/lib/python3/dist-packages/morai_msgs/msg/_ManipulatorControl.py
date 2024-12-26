# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from morai_msgs/ManipulatorControl.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class ManipulatorControl(genpy.Message):
  _md5sum = "2949cc268ce29a2b5dafef91eaf5bd1c"
  _type = "morai_msgs/ManipulatorControl"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int32 ControlType
float32 x1
float32 x2
float32 x3
float32 x4
float32 x5
float32 x6
bool GripperStatus

"""
  __slots__ = ['ControlType','x1','x2','x3','x4','x5','x6','GripperStatus']
  _slot_types = ['int32','float32','float32','float32','float32','float32','float32','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       ControlType,x1,x2,x3,x4,x5,x6,GripperStatus

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ManipulatorControl, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.ControlType is None:
        self.ControlType = 0
      if self.x1 is None:
        self.x1 = 0.
      if self.x2 is None:
        self.x2 = 0.
      if self.x3 is None:
        self.x3 = 0.
      if self.x4 is None:
        self.x4 = 0.
      if self.x5 is None:
        self.x5 = 0.
      if self.x6 is None:
        self.x6 = 0.
      if self.GripperStatus is None:
        self.GripperStatus = False
    else:
      self.ControlType = 0
      self.x1 = 0.
      self.x2 = 0.
      self.x3 = 0.
      self.x4 = 0.
      self.x5 = 0.
      self.x6 = 0.
      self.GripperStatus = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_i6fB().pack(_x.ControlType, _x.x1, _x.x2, _x.x3, _x.x4, _x.x5, _x.x6, _x.GripperStatus))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 29
      (_x.ControlType, _x.x1, _x.x2, _x.x3, _x.x4, _x.x5, _x.x6, _x.GripperStatus,) = _get_struct_i6fB().unpack(str[start:end])
      self.GripperStatus = bool(self.GripperStatus)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_i6fB().pack(_x.ControlType, _x.x1, _x.x2, _x.x3, _x.x4, _x.x5, _x.x6, _x.GripperStatus))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 29
      (_x.ControlType, _x.x1, _x.x2, _x.x3, _x.x4, _x.x5, _x.x6, _x.GripperStatus,) = _get_struct_i6fB().unpack(str[start:end])
      self.GripperStatus = bool(self.GripperStatus)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_i6fB = None
def _get_struct_i6fB():
    global _struct_i6fB
    if _struct_i6fB is None:
        _struct_i6fB = struct.Struct("<i6fB")
    return _struct_i6fB