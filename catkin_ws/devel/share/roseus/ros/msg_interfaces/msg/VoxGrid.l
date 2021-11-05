;; Auto-generated. Do not edit!


(when (boundp 'msg_interfaces::VoxGrid)
  (if (not (find-package "MSG_INTERFACES"))
    (make-package "MSG_INTERFACES"))
  (shadow 'VoxGrid (find-package "MSG_INTERFACES")))
(unless (find-package "MSG_INTERFACES::VOXGRID")
  (make-package "MSG_INTERFACES::VOXGRID"))

(in-package "ROS")
;;//! \htmlinclude VoxGrid.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass msg_interfaces::VoxGrid
  :super ros::object
  :slots (_header _height _width _depth _dl _dt _origin _theta _data ))

(defmethod msg_interfaces::VoxGrid
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:height __height) 0)
    ((:width __width) 0)
    ((:depth __depth) 0)
    ((:dl __dl) 0.0)
    ((:dt __dt) 0.0)
    ((:origin __origin) (instance geometry_msgs::Point :init))
    ((:theta __theta) 0.0)
    ((:data __data) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _header __header)
   (setq _height (round __height))
   (setq _width (round __width))
   (setq _depth (round __depth))
   (setq _dl (float __dl))
   (setq _dt (float __dt))
   (setq _origin __origin)
   (setq _theta (float __theta))
   (setq _data __data)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:depth
   (&optional __depth)
   (if __depth (setq _depth __depth)) _depth)
  (:dl
   (&optional __dl)
   (if __dl (setq _dl __dl)) _dl)
  (:dt
   (&optional __dt)
   (if __dt (setq _dt __dt)) _dt)
  (:origin
   (&rest __origin)
   (if (keywordp (car __origin))
       (send* _origin __origin)
     (progn
       (if __origin (setq _origin (car __origin)))
       _origin)))
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _height
    4
    ;; uint32 _width
    4
    ;; uint32 _depth
    4
    ;; float32 _dl
    4
    ;; float32 _dt
    4
    ;; geometry_msgs/Point _origin
    (send _origin :serialization-length)
    ;; float32 _theta
    4
    ;; uint8[] _data
    (* 1    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _height
       (write-long _height s)
     ;; uint32 _width
       (write-long _width s)
     ;; uint32 _depth
       (write-long _depth s)
     ;; float32 _dl
       (sys::poke _dl (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dt
       (sys::poke _dt (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Point _origin
       (send _origin :serialize s)
     ;; float32 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8[] _data
     (write-long (length _data) s)
     (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _height
     (setq _height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _width
     (setq _width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _depth
     (setq _depth (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _dl
     (setq _dl (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dt
     (setq _dt (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Point _origin
     (send _origin :deserialize buf ptr-) (incf ptr- (send _origin :serialization-length))
   ;; float32 _theta
     (setq _theta (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8[] _data
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _data (make-array n :element-type :char))
     (replace _data buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get msg_interfaces::VoxGrid :md5sum-) "1d25ba0a8468b05233fc2cee91e19528")
(setf (get msg_interfaces::VoxGrid :datatype-) "msg_interfaces/VoxGrid")
(setf (get msg_interfaces::VoxGrid :definition-)
      "# This represents a 3-D grid map, in which each cell represents the probability of
# occupancy at different times.
std_msgs/Header  header 
# Dimensions of the grid, depth for time. [cells]
uint32 height
uint32 width
uint32 depth
# The map resolutions in meters and seconds [m/cell] [s/cell]
float32 dl
float32 dt
# Origin of the grid in the world: position of the corner of voxel and time origin of the first prediction (0,0,0). [m, m, s]
geometry_msgs/Point origin
# Orientation of the map in the 2D plane [rad]
float32 theta
# The map data, in row-width-depth order, starting with (0,0,0).  Occupancy
# probabilities are in the range [0,255].
uint8[] data
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

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :msg_interfaces/VoxGrid "1d25ba0a8468b05233fc2cee91e19528")


