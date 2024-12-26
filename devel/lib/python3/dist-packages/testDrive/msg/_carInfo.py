# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from testDrive/carInfo.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class carInfo(genpy.Message):
  _md5sum = "e597445d8085e6b97cea6f509d947ddb"
  _type = "testDrive/carInfo"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# 모드
string mode  # 사용할 좌표계: "ENU" 또는 "UTM"

# ENU 좌표계
float64 enu_east
float64 enu_north
float64 enu_up
float64 yaw

# WGS84 좌표계
float64 utm_x
float64 utm_y
float64 heading

"""
  __slots__ = ['mode','enu_east','enu_north','enu_up','yaw','utm_x','utm_y','heading']
  _slot_types = ['string','float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       mode,enu_east,enu_north,enu_up,yaw,utm_x,utm_y,heading

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(carInfo, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.mode is None:
        self.mode = ''
      if self.enu_east is None:
        self.enu_east = 0.
      if self.enu_north is None:
        self.enu_north = 0.
      if self.enu_up is None:
        self.enu_up = 0.
      if self.yaw is None:
        self.yaw = 0.
      if self.utm_x is None:
        self.utm_x = 0.
      if self.utm_y is None:
        self.utm_y = 0.
      if self.heading is None:
        self.heading = 0.
    else:
      self.mode = ''
      self.enu_east = 0.
      self.enu_north = 0.
      self.enu_up = 0.
      self.yaw = 0.
      self.utm_x = 0.
      self.utm_y = 0.
      self.heading = 0.

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
      _x = self.mode
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_7d().pack(_x.enu_east, _x.enu_north, _x.enu_up, _x.yaw, _x.utm_x, _x.utm_y, _x.heading))
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
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.mode = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.mode = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.enu_east, _x.enu_north, _x.enu_up, _x.yaw, _x.utm_x, _x.utm_y, _x.heading,) = _get_struct_7d().unpack(str[start:end])
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
      _x = self.mode
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_7d().pack(_x.enu_east, _x.enu_north, _x.enu_up, _x.yaw, _x.utm_x, _x.utm_y, _x.heading))
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
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.mode = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.mode = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.enu_east, _x.enu_north, _x.enu_up, _x.yaw, _x.utm_x, _x.utm_y, _x.heading,) = _get_struct_7d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_7d = None
def _get_struct_7d():
    global _struct_7d
    if _struct_7d is None:
        _struct_7d = struct.Struct("<7d")
    return _struct_7d
