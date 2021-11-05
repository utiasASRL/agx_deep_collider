; Auto-generated. Do not edit!


(cl:in-package msg_interfaces-msg)


;//! \htmlinclude VoxGrid.msg.html

(cl:defclass <VoxGrid> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (height
    :reader height
    :initarg :height
    :type cl:integer
    :initform 0)
   (width
    :reader width
    :initarg :width
    :type cl:integer
    :initform 0)
   (depth
    :reader depth
    :initarg :depth
    :type cl:integer
    :initform 0)
   (dl
    :reader dl
    :initarg :dl
    :type cl:float
    :initform 0.0)
   (dt
    :reader dt
    :initarg :dt
    :type cl:float
    :initform 0.0)
   (origin
    :reader origin
    :initarg :origin
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (theta
    :reader theta
    :initarg :theta
    :type cl:float
    :initform 0.0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass VoxGrid (<VoxGrid>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VoxGrid>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VoxGrid)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name msg_interfaces-msg:<VoxGrid> is deprecated: use msg_interfaces-msg:VoxGrid instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <VoxGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_interfaces-msg:header-val is deprecated.  Use msg_interfaces-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <VoxGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_interfaces-msg:height-val is deprecated.  Use msg_interfaces-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <VoxGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_interfaces-msg:width-val is deprecated.  Use msg_interfaces-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'depth-val :lambda-list '(m))
(cl:defmethod depth-val ((m <VoxGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_interfaces-msg:depth-val is deprecated.  Use msg_interfaces-msg:depth instead.")
  (depth m))

(cl:ensure-generic-function 'dl-val :lambda-list '(m))
(cl:defmethod dl-val ((m <VoxGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_interfaces-msg:dl-val is deprecated.  Use msg_interfaces-msg:dl instead.")
  (dl m))

(cl:ensure-generic-function 'dt-val :lambda-list '(m))
(cl:defmethod dt-val ((m <VoxGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_interfaces-msg:dt-val is deprecated.  Use msg_interfaces-msg:dt instead.")
  (dt m))

(cl:ensure-generic-function 'origin-val :lambda-list '(m))
(cl:defmethod origin-val ((m <VoxGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_interfaces-msg:origin-val is deprecated.  Use msg_interfaces-msg:origin instead.")
  (origin m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <VoxGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_interfaces-msg:theta-val is deprecated.  Use msg_interfaces-msg:theta instead.")
  (theta m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <VoxGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_interfaces-msg:data-val is deprecated.  Use msg_interfaces-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VoxGrid>) ostream)
  "Serializes a message object of type '<VoxGrid>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'depth)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'depth)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'depth)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'depth)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dl))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'origin) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VoxGrid>) istream)
  "Deserializes a message object of type '<VoxGrid>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'depth)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'depth)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'depth)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'depth)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dl) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dt) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'origin) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VoxGrid>)))
  "Returns string type for a message object of type '<VoxGrid>"
  "msg_interfaces/VoxGrid")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VoxGrid)))
  "Returns string type for a message object of type 'VoxGrid"
  "msg_interfaces/VoxGrid")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VoxGrid>)))
  "Returns md5sum for a message object of type '<VoxGrid>"
  "1d25ba0a8468b05233fc2cee91e19528")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VoxGrid)))
  "Returns md5sum for a message object of type 'VoxGrid"
  "1d25ba0a8468b05233fc2cee91e19528")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VoxGrid>)))
  "Returns full string definition for message of type '<VoxGrid>"
  (cl:format cl:nil "# This represents a 3-D grid map, in which each cell represents the probability of~%# occupancy at different times.~%std_msgs/Header  header ~%# Dimensions of the grid, depth for time. [cells]~%uint32 height~%uint32 width~%uint32 depth~%# The map resolutions in meters and seconds [m/cell] [s/cell]~%float32 dl~%float32 dt~%# Origin of the grid in the world: position of the corner of voxel and time origin of the first prediction (0,0,0). [m, m, s]~%geometry_msgs/Point origin~%# Orientation of the map in the 2D plane [rad]~%float32 theta~%# The map data, in row-width-depth order, starting with (0,0,0).  Occupancy~%# probabilities are in the range [0,255].~%uint8[] data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VoxGrid)))
  "Returns full string definition for message of type 'VoxGrid"
  (cl:format cl:nil "# This represents a 3-D grid map, in which each cell represents the probability of~%# occupancy at different times.~%std_msgs/Header  header ~%# Dimensions of the grid, depth for time. [cells]~%uint32 height~%uint32 width~%uint32 depth~%# The map resolutions in meters and seconds [m/cell] [s/cell]~%float32 dl~%float32 dt~%# Origin of the grid in the world: position of the corner of voxel and time origin of the first prediction (0,0,0). [m, m, s]~%geometry_msgs/Point origin~%# Orientation of the map in the 2D plane [rad]~%float32 theta~%# The map data, in row-width-depth order, starting with (0,0,0).  Occupancy~%# probabilities are in the range [0,255].~%uint8[] data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VoxGrid>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'origin))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VoxGrid>))
  "Converts a ROS message object to a list"
  (cl:list 'VoxGrid
    (cl:cons ':header (header msg))
    (cl:cons ':height (height msg))
    (cl:cons ':width (width msg))
    (cl:cons ':depth (depth msg))
    (cl:cons ':dl (dl msg))
    (cl:cons ':dt (dt msg))
    (cl:cons ':origin (origin msg))
    (cl:cons ':theta (theta msg))
    (cl:cons ':data (data msg))
))
