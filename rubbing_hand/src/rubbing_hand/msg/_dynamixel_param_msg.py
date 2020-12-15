# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from rubbing_hand/dynamixel_param_msg.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class dynamixel_param_msg(genpy.Message):
  _md5sum = "cdac21b3e022c4862c8d9fd54fe39a8b"
  _type = "rubbing_hand/dynamixel_param_msg"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int32 surface_pos
float64 interval
float64 fps
int32[] trg_pos
"""
  __slots__ = ['surface_pos','interval','fps','trg_pos']
  _slot_types = ['int32','float64','float64','int32[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       surface_pos,interval,fps,trg_pos

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(dynamixel_param_msg, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.surface_pos is None:
        self.surface_pos = 0
      if self.interval is None:
        self.interval = 0.
      if self.fps is None:
        self.fps = 0.
      if self.trg_pos is None:
        self.trg_pos = []
    else:
      self.surface_pos = 0
      self.interval = 0.
      self.fps = 0.
      self.trg_pos = []

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
      buff.write(_get_struct_i2d().pack(_x.surface_pos, _x.interval, _x.fps))
      length = len(self.trg_pos)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(struct.pack(pattern, *self.trg_pos))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 20
      (_x.surface_pos, _x.interval, _x.fps,) = _get_struct_i2d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.trg_pos = struct.unpack(pattern, str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_i2d().pack(_x.surface_pos, _x.interval, _x.fps))
      length = len(self.trg_pos)
      buff.write(_struct_I.pack(length))
      pattern = '<%si'%length
      buff.write(self.trg_pos.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 20
      (_x.surface_pos, _x.interval, _x.fps,) = _get_struct_i2d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%si'%length
      start = end
      end += struct.calcsize(pattern)
      self.trg_pos = numpy.frombuffer(str[start:end], dtype=numpy.int32, count=length)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_i2d = None
def _get_struct_i2d():
    global _struct_i2d
    if _struct_i2d is None:
        _struct_i2d = struct.Struct("<i2d")
    return _struct_i2d