; Auto-generated. Do not edit!


(cl:in-package rubbing_hand-msg)


;//! \htmlinclude dynamixel_param_msg.msg.html

(cl:defclass <dynamixel_param_msg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (surface_pos
    :reader surface_pos
    :initarg :surface_pos
    :type cl:integer
    :initform 0)
   (interval
    :reader interval
    :initarg :interval
    :type cl:float
    :initform 0.0)
   (fps
    :reader fps
    :initarg :fps
    :type cl:float
    :initform 0.0)
   (trg_pos
    :reader trg_pos
    :initarg :trg_pos
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (degree_of_finger
    :reader degree_of_finger
    :initarg :degree_of_finger
    :type cl:float
    :initform 0.0)
   (debug
    :reader debug
    :initarg :debug
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass dynamixel_param_msg (<dynamixel_param_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <dynamixel_param_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'dynamixel_param_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rubbing_hand-msg:<dynamixel_param_msg> is deprecated: use rubbing_hand-msg:dynamixel_param_msg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <dynamixel_param_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rubbing_hand-msg:header-val is deprecated.  Use rubbing_hand-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'surface_pos-val :lambda-list '(m))
(cl:defmethod surface_pos-val ((m <dynamixel_param_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rubbing_hand-msg:surface_pos-val is deprecated.  Use rubbing_hand-msg:surface_pos instead.")
  (surface_pos m))

(cl:ensure-generic-function 'interval-val :lambda-list '(m))
(cl:defmethod interval-val ((m <dynamixel_param_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rubbing_hand-msg:interval-val is deprecated.  Use rubbing_hand-msg:interval instead.")
  (interval m))

(cl:ensure-generic-function 'fps-val :lambda-list '(m))
(cl:defmethod fps-val ((m <dynamixel_param_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rubbing_hand-msg:fps-val is deprecated.  Use rubbing_hand-msg:fps instead.")
  (fps m))

(cl:ensure-generic-function 'trg_pos-val :lambda-list '(m))
(cl:defmethod trg_pos-val ((m <dynamixel_param_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rubbing_hand-msg:trg_pos-val is deprecated.  Use rubbing_hand-msg:trg_pos instead.")
  (trg_pos m))

(cl:ensure-generic-function 'degree_of_finger-val :lambda-list '(m))
(cl:defmethod degree_of_finger-val ((m <dynamixel_param_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rubbing_hand-msg:degree_of_finger-val is deprecated.  Use rubbing_hand-msg:degree_of_finger instead.")
  (degree_of_finger m))

(cl:ensure-generic-function 'debug-val :lambda-list '(m))
(cl:defmethod debug-val ((m <dynamixel_param_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rubbing_hand-msg:debug-val is deprecated.  Use rubbing_hand-msg:debug instead.")
  (debug m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <dynamixel_param_msg>) ostream)
  "Serializes a message object of type '<dynamixel_param_msg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'surface_pos)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'interval))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'fps))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'trg_pos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'trg_pos))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'degree_of_finger))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'debug))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'debug))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <dynamixel_param_msg>) istream)
  "Deserializes a message object of type '<dynamixel_param_msg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'surface_pos) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'interval) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fps) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'trg_pos) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'trg_pos)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'degree_of_finger) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'debug) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'debug)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<dynamixel_param_msg>)))
  "Returns string type for a message object of type '<dynamixel_param_msg>"
  "rubbing_hand/dynamixel_param_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'dynamixel_param_msg)))
  "Returns string type for a message object of type 'dynamixel_param_msg"
  "rubbing_hand/dynamixel_param_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<dynamixel_param_msg>)))
  "Returns md5sum for a message object of type '<dynamixel_param_msg>"
  "703b4f968dd9341d6e667a6ac1375019")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'dynamixel_param_msg)))
  "Returns md5sum for a message object of type 'dynamixel_param_msg"
  "703b4f968dd9341d6e667a6ac1375019")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<dynamixel_param_msg>)))
  "Returns full string definition for message of type '<dynamixel_param_msg>"
  (cl:format cl:nil "Header header~%int32 surface_pos~%float64 interval~%float64 fps~%int32[] trg_pos~%float64 degree_of_finger~%float64[] debug~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'dynamixel_param_msg)))
  "Returns full string definition for message of type 'dynamixel_param_msg"
  (cl:format cl:nil "Header header~%int32 surface_pos~%float64 interval~%float64 fps~%int32[] trg_pos~%float64 degree_of_finger~%float64[] debug~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <dynamixel_param_msg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     8
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'trg_pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'debug) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <dynamixel_param_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'dynamixel_param_msg
    (cl:cons ':header (header msg))
    (cl:cons ':surface_pos (surface_pos msg))
    (cl:cons ':interval (interval msg))
    (cl:cons ':fps (fps msg))
    (cl:cons ':trg_pos (trg_pos msg))
    (cl:cons ':degree_of_finger (degree_of_finger msg))
    (cl:cons ':debug (debug msg))
))
