"""autogenerated by genpy from icp_service/LSTransformRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg

class LSTransformRequest(genpy.Message):
  _md5sum = "4707d4fc95a0f9320111fb3fefd7c572"
  _type = "icp_service/LSTransformRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """sensor_msgs/PointCloud2 pc1
sensor_msgs/PointCloud2 pc2
geometry_msgs/Pose guess
geometry_msgs/Pose arPose1
geometry_msgs/Pose arPose2

================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the "fields" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of postion and orientation. 
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

"""
  __slots__ = ['pc1','pc2','guess','arPose1','arPose2']
  _slot_types = ['sensor_msgs/PointCloud2','sensor_msgs/PointCloud2','geometry_msgs/Pose','geometry_msgs/Pose','geometry_msgs/Pose']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       pc1,pc2,guess,arPose1,arPose2

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(LSTransformRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.pc1 is None:
        self.pc1 = sensor_msgs.msg.PointCloud2()
      if self.pc2 is None:
        self.pc2 = sensor_msgs.msg.PointCloud2()
      if self.guess is None:
        self.guess = geometry_msgs.msg.Pose()
      if self.arPose1 is None:
        self.arPose1 = geometry_msgs.msg.Pose()
      if self.arPose2 is None:
        self.arPose2 = geometry_msgs.msg.Pose()
    else:
      self.pc1 = sensor_msgs.msg.PointCloud2()
      self.pc2 = sensor_msgs.msg.PointCloud2()
      self.guess = geometry_msgs.msg.Pose()
      self.arPose1 = geometry_msgs.msg.Pose()
      self.arPose2 = geometry_msgs.msg.Pose()

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
      buff.write(_struct_3I.pack(_x.pc1.header.seq, _x.pc1.header.stamp.secs, _x.pc1.header.stamp.nsecs))
      _x = self.pc1.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.pc1.height, _x.pc1.width))
      length = len(self.pc1.fields)
      buff.write(_struct_I.pack(length))
      for val1 in self.pc1.fields:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_IBI.pack(_x.offset, _x.datatype, _x.count))
      _x = self
      buff.write(_struct_B2I.pack(_x.pc1.is_bigendian, _x.pc1.point_step, _x.pc1.row_step))
      _x = self.pc1.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_B3I.pack(_x.pc1.is_dense, _x.pc2.header.seq, _x.pc2.header.stamp.secs, _x.pc2.header.stamp.nsecs))
      _x = self.pc2.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.pc2.height, _x.pc2.width))
      length = len(self.pc2.fields)
      buff.write(_struct_I.pack(length))
      for val1 in self.pc2.fields:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_IBI.pack(_x.offset, _x.datatype, _x.count))
      _x = self
      buff.write(_struct_B2I.pack(_x.pc2.is_bigendian, _x.pc2.point_step, _x.pc2.row_step))
      _x = self.pc2.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_B21d.pack(_x.pc2.is_dense, _x.guess.position.x, _x.guess.position.y, _x.guess.position.z, _x.guess.orientation.x, _x.guess.orientation.y, _x.guess.orientation.z, _x.guess.orientation.w, _x.arPose1.position.x, _x.arPose1.position.y, _x.arPose1.position.z, _x.arPose1.orientation.x, _x.arPose1.orientation.y, _x.arPose1.orientation.z, _x.arPose1.orientation.w, _x.arPose2.position.x, _x.arPose2.position.y, _x.arPose2.position.z, _x.arPose2.orientation.x, _x.arPose2.orientation.y, _x.arPose2.orientation.z, _x.arPose2.orientation.w))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.pc1 is None:
        self.pc1 = sensor_msgs.msg.PointCloud2()
      if self.pc2 is None:
        self.pc2 = sensor_msgs.msg.PointCloud2()
      if self.guess is None:
        self.guess = geometry_msgs.msg.Pose()
      if self.arPose1 is None:
        self.arPose1 = geometry_msgs.msg.Pose()
      if self.arPose2 is None:
        self.arPose2 = geometry_msgs.msg.Pose()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.pc1.header.seq, _x.pc1.header.stamp.secs, _x.pc1.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.pc1.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.pc1.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.pc1.height, _x.pc1.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.pc1.fields = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.PointField()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 9
        (_x.offset, _x.datatype, _x.count,) = _struct_IBI.unpack(str[start:end])
        self.pc1.fields.append(val1)
      _x = self
      start = end
      end += 9
      (_x.pc1.is_bigendian, _x.pc1.point_step, _x.pc1.row_step,) = _struct_B2I.unpack(str[start:end])
      self.pc1.is_bigendian = bool(self.pc1.is_bigendian)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.pc1.data = str[start:end].decode('utf-8')
      else:
        self.pc1.data = str[start:end]
      _x = self
      start = end
      end += 13
      (_x.pc1.is_dense, _x.pc2.header.seq, _x.pc2.header.stamp.secs, _x.pc2.header.stamp.nsecs,) = _struct_B3I.unpack(str[start:end])
      self.pc1.is_dense = bool(self.pc1.is_dense)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.pc2.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.pc2.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.pc2.height, _x.pc2.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.pc2.fields = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.PointField()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 9
        (_x.offset, _x.datatype, _x.count,) = _struct_IBI.unpack(str[start:end])
        self.pc2.fields.append(val1)
      _x = self
      start = end
      end += 9
      (_x.pc2.is_bigendian, _x.pc2.point_step, _x.pc2.row_step,) = _struct_B2I.unpack(str[start:end])
      self.pc2.is_bigendian = bool(self.pc2.is_bigendian)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.pc2.data = str[start:end].decode('utf-8')
      else:
        self.pc2.data = str[start:end]
      _x = self
      start = end
      end += 169
      (_x.pc2.is_dense, _x.guess.position.x, _x.guess.position.y, _x.guess.position.z, _x.guess.orientation.x, _x.guess.orientation.y, _x.guess.orientation.z, _x.guess.orientation.w, _x.arPose1.position.x, _x.arPose1.position.y, _x.arPose1.position.z, _x.arPose1.orientation.x, _x.arPose1.orientation.y, _x.arPose1.orientation.z, _x.arPose1.orientation.w, _x.arPose2.position.x, _x.arPose2.position.y, _x.arPose2.position.z, _x.arPose2.orientation.x, _x.arPose2.orientation.y, _x.arPose2.orientation.z, _x.arPose2.orientation.w,) = _struct_B21d.unpack(str[start:end])
      self.pc2.is_dense = bool(self.pc2.is_dense)
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
      buff.write(_struct_3I.pack(_x.pc1.header.seq, _x.pc1.header.stamp.secs, _x.pc1.header.stamp.nsecs))
      _x = self.pc1.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.pc1.height, _x.pc1.width))
      length = len(self.pc1.fields)
      buff.write(_struct_I.pack(length))
      for val1 in self.pc1.fields:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_IBI.pack(_x.offset, _x.datatype, _x.count))
      _x = self
      buff.write(_struct_B2I.pack(_x.pc1.is_bigendian, _x.pc1.point_step, _x.pc1.row_step))
      _x = self.pc1.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_B3I.pack(_x.pc1.is_dense, _x.pc2.header.seq, _x.pc2.header.stamp.secs, _x.pc2.header.stamp.nsecs))
      _x = self.pc2.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2I.pack(_x.pc2.height, _x.pc2.width))
      length = len(self.pc2.fields)
      buff.write(_struct_I.pack(length))
      for val1 in self.pc2.fields:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_IBI.pack(_x.offset, _x.datatype, _x.count))
      _x = self
      buff.write(_struct_B2I.pack(_x.pc2.is_bigendian, _x.pc2.point_step, _x.pc2.row_step))
      _x = self.pc2.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_B21d.pack(_x.pc2.is_dense, _x.guess.position.x, _x.guess.position.y, _x.guess.position.z, _x.guess.orientation.x, _x.guess.orientation.y, _x.guess.orientation.z, _x.guess.orientation.w, _x.arPose1.position.x, _x.arPose1.position.y, _x.arPose1.position.z, _x.arPose1.orientation.x, _x.arPose1.orientation.y, _x.arPose1.orientation.z, _x.arPose1.orientation.w, _x.arPose2.position.x, _x.arPose2.position.y, _x.arPose2.position.z, _x.arPose2.orientation.x, _x.arPose2.orientation.y, _x.arPose2.orientation.z, _x.arPose2.orientation.w))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.pc1 is None:
        self.pc1 = sensor_msgs.msg.PointCloud2()
      if self.pc2 is None:
        self.pc2 = sensor_msgs.msg.PointCloud2()
      if self.guess is None:
        self.guess = geometry_msgs.msg.Pose()
      if self.arPose1 is None:
        self.arPose1 = geometry_msgs.msg.Pose()
      if self.arPose2 is None:
        self.arPose2 = geometry_msgs.msg.Pose()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.pc1.header.seq, _x.pc1.header.stamp.secs, _x.pc1.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.pc1.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.pc1.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.pc1.height, _x.pc1.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.pc1.fields = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.PointField()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 9
        (_x.offset, _x.datatype, _x.count,) = _struct_IBI.unpack(str[start:end])
        self.pc1.fields.append(val1)
      _x = self
      start = end
      end += 9
      (_x.pc1.is_bigendian, _x.pc1.point_step, _x.pc1.row_step,) = _struct_B2I.unpack(str[start:end])
      self.pc1.is_bigendian = bool(self.pc1.is_bigendian)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.pc1.data = str[start:end].decode('utf-8')
      else:
        self.pc1.data = str[start:end]
      _x = self
      start = end
      end += 13
      (_x.pc1.is_dense, _x.pc2.header.seq, _x.pc2.header.stamp.secs, _x.pc2.header.stamp.nsecs,) = _struct_B3I.unpack(str[start:end])
      self.pc1.is_dense = bool(self.pc1.is_dense)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.pc2.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.pc2.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.pc2.height, _x.pc2.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.pc2.fields = []
      for i in range(0, length):
        val1 = sensor_msgs.msg.PointField()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 9
        (_x.offset, _x.datatype, _x.count,) = _struct_IBI.unpack(str[start:end])
        self.pc2.fields.append(val1)
      _x = self
      start = end
      end += 9
      (_x.pc2.is_bigendian, _x.pc2.point_step, _x.pc2.row_step,) = _struct_B2I.unpack(str[start:end])
      self.pc2.is_bigendian = bool(self.pc2.is_bigendian)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.pc2.data = str[start:end].decode('utf-8')
      else:
        self.pc2.data = str[start:end]
      _x = self
      start = end
      end += 169
      (_x.pc2.is_dense, _x.guess.position.x, _x.guess.position.y, _x.guess.position.z, _x.guess.orientation.x, _x.guess.orientation.y, _x.guess.orientation.z, _x.guess.orientation.w, _x.arPose1.position.x, _x.arPose1.position.y, _x.arPose1.position.z, _x.arPose1.orientation.x, _x.arPose1.orientation.y, _x.arPose1.orientation.z, _x.arPose1.orientation.w, _x.arPose2.position.x, _x.arPose2.position.y, _x.arPose2.position.z, _x.arPose2.orientation.x, _x.arPose2.orientation.y, _x.arPose2.orientation.z, _x.arPose2.orientation.w,) = _struct_B21d.unpack(str[start:end])
      self.pc2.is_dense = bool(self.pc2.is_dense)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_IBI = struct.Struct("<IBI")
_struct_B21d = struct.Struct("<B21d")
_struct_3I = struct.Struct("<3I")
_struct_B3I = struct.Struct("<B3I")
_struct_B2I = struct.Struct("<B2I")
_struct_2I = struct.Struct("<2I")
"""autogenerated by genpy from icp_service/LSTransformResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class LSTransformResponse(genpy.Message):
  _md5sum = "f192399f711a48924df9a394d37edd67"
  _type = "icp_service/LSTransformResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """geometry_msgs/Pose pose



================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of postion and orientation. 
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

"""
  __slots__ = ['pose']
  _slot_types = ['geometry_msgs/Pose']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       pose

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(LSTransformResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose()
    else:
      self.pose = geometry_msgs.msg.Pose()

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
      buff.write(_struct_7d.pack(_x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose()
      end = 0
      _x = self
      start = end
      end += 56
      (_x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w,) = _struct_7d.unpack(str[start:end])
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
      buff.write(_struct_7d.pack(_x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose()
      end = 0
      _x = self
      start = end
      end += 56
      (_x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w,) = _struct_7d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_7d = struct.Struct("<7d")
class LSTransform(object):
  _type          = 'icp_service/LSTransform'
  _md5sum = 'a5520d4a29ce7d12a0a953be6631c61f'
  _request_class  = LSTransformRequest
  _response_class = LSTransformResponse