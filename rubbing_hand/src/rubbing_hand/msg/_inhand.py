# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from rubbing_hand/inhand.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class inhand(genpy.Message):
  _md5sum = "266b6584c02ea6d87499d928f5c0eec9"
  _type = "rubbing_hand/inhand"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """Header header

# --------dynamixel--------

# ダイナミクセルへ渡した目標指間距離
# 取得した指間距離へ操作量を加えている
# interval = got_interval + MV
float64 interval

# 指間距離へ加える操作量(Manipulated Variable)
float64 MV

# -------------------------

# --------fingervision-----

# Array of slips (slip distribution), which is a serialized list of 3x3 matrix.
# Each cell in the 3x3 matrix is the sum of moving pixels in the cell.
float32[] mv_s

# objectの角度 [deg]
float64 obj_orientation

# objectの角度 [deg]  obj_orientationをsmaでフィルターしてる（filter_node参照）
float64 obj_orientation_filtered

# objectの角速度 [deg/s]　obj_orientation_filteredの差分をとったものをsmaでフィルターしてる（filter_node参照）
float64 d_obj_orientation_filtered

# -------------------------

# ---------inhand----------

# 目標角度
float64 target_obj_orientation

# 目標角速度
float64 target_d_obj_orientation

# 目標角速度と取得した角速度の差
# d_obj_orientation_filtered - target_d_obj_orientation
float64 omega_d

# mv_sの和からすべり判定をする際のしきい値
float64 th_slip

# 取得した角速度d_obj_orientation_filteredと目標角速度target_d_obj_orientationとの差から操作量MVを決めるパラメータ
# MV_input  = [neutral_min, neutral_max , drop]
# MV_output = [open, close, quick_close]
# drop < d_obj_orientation_filtered : quick_close
# d_omega <= neutral_min : open
# neutral_min < d_omega <= neutral_max : 0
# neutral_max < d_omega : close
float64[] MV_i
float64[] MV_o

# マニピュレーション実行区間を表すフラグ
# 0: してない， 1:マニピュレーション終了後の数秒間， 2:マニピュレーション中, 3:途中終了した
int32 process_f

# -------------------------

# なんでも入れていいよ
float64[] debag
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
"""
  __slots__ = ['header','interval','MV','mv_s','obj_orientation','obj_orientation_filtered','d_obj_orientation_filtered','target_obj_orientation','target_d_obj_orientation','omega_d','th_slip','MV_i','MV_o','process_f','debag']
  _slot_types = ['std_msgs/Header','float64','float64','float32[]','float64','float64','float64','float64','float64','float64','float64','float64[]','float64[]','int32','float64[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,interval,MV,mv_s,obj_orientation,obj_orientation_filtered,d_obj_orientation_filtered,target_obj_orientation,target_d_obj_orientation,omega_d,th_slip,MV_i,MV_o,process_f,debag

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(inhand, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.interval is None:
        self.interval = 0.
      if self.MV is None:
        self.MV = 0.
      if self.mv_s is None:
        self.mv_s = []
      if self.obj_orientation is None:
        self.obj_orientation = 0.
      if self.obj_orientation_filtered is None:
        self.obj_orientation_filtered = 0.
      if self.d_obj_orientation_filtered is None:
        self.d_obj_orientation_filtered = 0.
      if self.target_obj_orientation is None:
        self.target_obj_orientation = 0.
      if self.target_d_obj_orientation is None:
        self.target_d_obj_orientation = 0.
      if self.omega_d is None:
        self.omega_d = 0.
      if self.th_slip is None:
        self.th_slip = 0.
      if self.MV_i is None:
        self.MV_i = []
      if self.MV_o is None:
        self.MV_o = []
      if self.process_f is None:
        self.process_f = 0
      if self.debag is None:
        self.debag = []
    else:
      self.header = std_msgs.msg.Header()
      self.interval = 0.
      self.MV = 0.
      self.mv_s = []
      self.obj_orientation = 0.
      self.obj_orientation_filtered = 0.
      self.d_obj_orientation_filtered = 0.
      self.target_obj_orientation = 0.
      self.target_d_obj_orientation = 0.
      self.omega_d = 0.
      self.th_slip = 0.
      self.MV_i = []
      self.MV_o = []
      self.process_f = 0
      self.debag = []

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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2d().pack(_x.interval, _x.MV))
      length = len(self.mv_s)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.Struct(pattern).pack(*self.mv_s))
      _x = self
      buff.write(_get_struct_7d().pack(_x.obj_orientation, _x.obj_orientation_filtered, _x.d_obj_orientation_filtered, _x.target_obj_orientation, _x.target_d_obj_orientation, _x.omega_d, _x.th_slip))
      length = len(self.MV_i)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.MV_i))
      length = len(self.MV_o)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.MV_o))
      _x = self.process_f
      buff.write(_get_struct_i().pack(_x))
      length = len(self.debag)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.debag))
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
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 16
      (_x.interval, _x.MV,) = _get_struct_2d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.mv_s = s.unpack(str[start:end])
      _x = self
      start = end
      end += 56
      (_x.obj_orientation, _x.obj_orientation_filtered, _x.d_obj_orientation_filtered, _x.target_obj_orientation, _x.target_d_obj_orientation, _x.omega_d, _x.th_slip,) = _get_struct_7d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.MV_i = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.MV_o = s.unpack(str[start:end])
      start = end
      end += 4
      (self.process_f,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.debag = s.unpack(str[start:end])
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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2d().pack(_x.interval, _x.MV))
      length = len(self.mv_s)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.mv_s.tostring())
      _x = self
      buff.write(_get_struct_7d().pack(_x.obj_orientation, _x.obj_orientation_filtered, _x.d_obj_orientation_filtered, _x.target_obj_orientation, _x.target_d_obj_orientation, _x.omega_d, _x.th_slip))
      length = len(self.MV_i)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.MV_i.tostring())
      length = len(self.MV_o)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.MV_o.tostring())
      _x = self.process_f
      buff.write(_get_struct_i().pack(_x))
      length = len(self.debag)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.debag.tostring())
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
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 16
      (_x.interval, _x.MV,) = _get_struct_2d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.mv_s = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      _x = self
      start = end
      end += 56
      (_x.obj_orientation, _x.obj_orientation_filtered, _x.d_obj_orientation_filtered, _x.target_obj_orientation, _x.target_d_obj_orientation, _x.omega_d, _x.th_slip,) = _get_struct_7d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.MV_i = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.MV_o = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (self.process_f,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.debag = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2d = None
def _get_struct_2d():
    global _struct_2d
    if _struct_2d is None:
        _struct_2d = struct.Struct("<2d")
    return _struct_2d
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_7d = None
def _get_struct_7d():
    global _struct_7d
    if _struct_7d is None:
        _struct_7d = struct.Struct("<7d")
    return _struct_7d
_struct_i = None
def _get_struct_i():
    global _struct_i
    if _struct_i is None:
        _struct_i = struct.Struct("<i")
    return _struct_i
