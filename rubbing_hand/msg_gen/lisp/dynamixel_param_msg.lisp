; Auto-generated. Do not edit!


(cl:in-package rubbing_hand-msg)


;//! \htmlinclude dynamixel_param_msg.msg.html

(cl:defclass <dynamixel_param_msg> (roslisp-msg-protocol:ros-message)
  ((surface_pos
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
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass dynamixel_param_msg (<dynamixel_param_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <dynamixel_param_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'dynamixel_param_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rubbing_hand-msg:<dynamixel_param_msg> is deprecated: use rubbing_hand-msg:dynamixel_param_msg instead.")))

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
(cl:defmethod roslisp-msg-protocol:serialize ((msg <dynamixel_param_msg>) ostream)
  "Serializes a message object of type '<dynamixel_param_msg>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <dynamixel_param_msg>) istream)
  "Deserializes a message object of type '<dynamixel_param_msg>"
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
  "cdac21b3e022c4862c8d9fd54fe39a8b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'dynamixel_param_msg)))
  "Returns md5sum for a message object of type 'dynamixel_param_msg"
  "cdac21b3e022c4862c8d9fd54fe39a8b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<dynamixel_param_msg>)))
  "Returns full string definition for message of type '<dynamixel_param_msg>"
  (cl:format cl:nil "int32 surface_pos~%float64 interval~%float64 fps~%int32[] trg_pos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'dynamixel_param_msg)))
  "Returns full string definition for message of type 'dynamixel_param_msg"
  (cl:format cl:nil "int32 surface_pos~%float64 interval~%float64 fps~%int32[] trg_pos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <dynamixel_param_msg>))
  (cl:+ 0
     4
     8
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'trg_pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <dynamixel_param_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'dynamixel_param_msg
    (cl:cons ':surface_pos (surface_pos msg))
    (cl:cons ':interval (interval msg))
    (cl:cons ':fps (fps msg))
    (cl:cons ':trg_pos (trg_pos msg))
))
