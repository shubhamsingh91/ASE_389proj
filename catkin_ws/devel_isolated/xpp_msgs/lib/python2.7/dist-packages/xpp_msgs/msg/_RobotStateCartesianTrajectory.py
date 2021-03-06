# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from xpp_msgs/RobotStateCartesianTrajectory.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy
import geometry_msgs.msg
import std_msgs.msg
import xpp_msgs.msg

class RobotStateCartesianTrajectory(genpy.Message):
  _md5sum = "cf81e80e883d7cf1f8652d3cf7985437"
  _type = "xpp_msgs/RobotStateCartesianTrajectory"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """# The state of a robot expressed in the cartesian frame

# The header is used to specify the coordinate frame and the reference time for the trajectory durations
std_msgs/Header header

# A representation of a Cartesian trajectory
RobotStateCartesian[] points

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
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: xpp_msgs/RobotStateCartesian
# The state of a robot expressed in the Cartesian frame

duration                time_from_start   # global time along trajectory

# Position, velocity and acceleration of the base expressed in world frame
# The orientation quaternion maps base to world frame.
State6d                 base              # base pos/vel/acc in world

StateLin3d[]            ee_motion         # endeffector pos/vel/acc in world
geometry_msgs/Vector3[] ee_forces         # endeffector forces expressed in world
bool[]                  ee_contact        # True if the foot is touching the environment




================================================================================
MSG: xpp_msgs/State6d
# The state of the 6D base of a system

geometry_msgs/Pose     pose         # The 6D linear and angular position, orientation maps base to world
geometry_msgs/Twist    twist        # The 6D linear and angular velocity 
geometry_msgs/Accel    accel        # The 6D linear and angular acceleration
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Accel
# This expresses acceleration in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: xpp_msgs/StateLin3d
# This contains the 3D representation of a linear state, including:
# position, velocity, acceleration

geometry_msgs/Point pos
geometry_msgs/Vector3 vel
geometry_msgs/Vector3 acc"""
  __slots__ = ['header','points']
  _slot_types = ['std_msgs/Header','xpp_msgs/RobotStateCartesian[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,points

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(RobotStateCartesianTrajectory, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.points is None:
        self.points = []
    else:
      self.header = std_msgs.msg.Header()
      self.points = []

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
      length = len(self.points)
      buff.write(_struct_I.pack(length))
      for val1 in self.points:
        _v1 = val1.time_from_start
        _x = _v1
        buff.write(_get_struct_2i().pack(_x.secs, _x.nsecs))
        _v2 = val1.base
        _v3 = _v2.pose
        _v4 = _v3.position
        _x = _v4
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v5 = _v3.orientation
        _x = _v5
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
        _v6 = _v2.twist
        _v7 = _v6.linear
        _x = _v7
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v8 = _v6.angular
        _x = _v8
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v9 = _v2.accel
        _v10 = _v9.linear
        _x = _v10
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v11 = _v9.angular
        _x = _v11
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        length = len(val1.ee_motion)
        buff.write(_struct_I.pack(length))
        for val2 in val1.ee_motion:
          _v12 = val2.pos
          _x = _v12
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
          _v13 = val2.vel
          _x = _v13
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
          _v14 = val2.acc
          _x = _v14
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        length = len(val1.ee_forces)
        buff.write(_struct_I.pack(length))
        for val2 in val1.ee_forces:
          _x = val2
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        length = len(val1.ee_contact)
        buff.write(_struct_I.pack(length))
        pattern = '<%sB'%length
        buff.write(struct.Struct(pattern).pack(*val1.ee_contact))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.points is None:
        self.points = None
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
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.points = []
      for i in range(0, length):
        val1 = xpp_msgs.msg.RobotStateCartesian()
        _v15 = val1.time_from_start
        _x = _v15
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2i().unpack(str[start:end])
        _v16 = val1.base
        _v17 = _v16.pose
        _v18 = _v17.position
        _x = _v18
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v19 = _v17.orientation
        _x = _v19
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
        _v20 = _v16.twist
        _v21 = _v20.linear
        _x = _v21
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v22 = _v20.angular
        _x = _v22
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v23 = _v16.accel
        _v24 = _v23.linear
        _x = _v24
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v25 = _v23.angular
        _x = _v25
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.ee_motion = []
        for i in range(0, length):
          val2 = xpp_msgs.msg.StateLin3d()
          _v26 = val2.pos
          _x = _v26
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          _v27 = val2.vel
          _x = _v27
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          _v28 = val2.acc
          _x = _v28
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          val1.ee_motion.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.ee_forces = []
        for i in range(0, length):
          val2 = geometry_msgs.msg.Vector3()
          _x = val2
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          val1.ee_forces.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sB'%length
        start = end
        s = struct.Struct(pattern)
        end += s.size
        val1.ee_contact = s.unpack(str[start:end])
        val1.ee_contact = list(map(bool, val1.ee_contact))
        self.points.append(val1)
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
      length = len(self.points)
      buff.write(_struct_I.pack(length))
      for val1 in self.points:
        _v29 = val1.time_from_start
        _x = _v29
        buff.write(_get_struct_2i().pack(_x.secs, _x.nsecs))
        _v30 = val1.base
        _v31 = _v30.pose
        _v32 = _v31.position
        _x = _v32
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v33 = _v31.orientation
        _x = _v33
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
        _v34 = _v30.twist
        _v35 = _v34.linear
        _x = _v35
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v36 = _v34.angular
        _x = _v36
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v37 = _v30.accel
        _v38 = _v37.linear
        _x = _v38
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v39 = _v37.angular
        _x = _v39
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        length = len(val1.ee_motion)
        buff.write(_struct_I.pack(length))
        for val2 in val1.ee_motion:
          _v40 = val2.pos
          _x = _v40
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
          _v41 = val2.vel
          _x = _v41
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
          _v42 = val2.acc
          _x = _v42
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        length = len(val1.ee_forces)
        buff.write(_struct_I.pack(length))
        for val2 in val1.ee_forces:
          _x = val2
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        length = len(val1.ee_contact)
        buff.write(_struct_I.pack(length))
        pattern = '<%sB'%length
        buff.write(val1.ee_contact.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.points is None:
        self.points = None
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
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.points = []
      for i in range(0, length):
        val1 = xpp_msgs.msg.RobotStateCartesian()
        _v43 = val1.time_from_start
        _x = _v43
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2i().unpack(str[start:end])
        _v44 = val1.base
        _v45 = _v44.pose
        _v46 = _v45.position
        _x = _v46
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v47 = _v45.orientation
        _x = _v47
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
        _v48 = _v44.twist
        _v49 = _v48.linear
        _x = _v49
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v50 = _v48.angular
        _x = _v50
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v51 = _v44.accel
        _v52 = _v51.linear
        _x = _v52
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v53 = _v51.angular
        _x = _v53
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.ee_motion = []
        for i in range(0, length):
          val2 = xpp_msgs.msg.StateLin3d()
          _v54 = val2.pos
          _x = _v54
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          _v55 = val2.vel
          _x = _v55
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          _v56 = val2.acc
          _x = _v56
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          val1.ee_motion.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.ee_forces = []
        for i in range(0, length):
          val2 = geometry_msgs.msg.Vector3()
          _x = val2
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          val1.ee_forces.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sB'%length
        start = end
        s = struct.Struct(pattern)
        end += s.size
        val1.ee_contact = numpy.frombuffer(str[start:end], dtype=numpy.bool, count=length)
        val1.ee_contact = list(map(bool, val1.ee_contact))
        self.points.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2i = None
def _get_struct_2i():
    global _struct_2i
    if _struct_2i is None:
        _struct_2i = struct.Struct("<2i")
    return _struct_2i
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
_struct_4d = None
def _get_struct_4d():
    global _struct_4d
    if _struct_4d is None:
        _struct_4d = struct.Struct("<4d")
    return _struct_4d
