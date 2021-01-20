; Auto-generated. Do not edit!


(cl:in-package rubbing_hand-srv)


;//! \htmlinclude Set2Float64-request.msg.html

(cl:defclass <Set2Float64-request> (roslisp-msg-protocol:ros-message)
  ((data1
    :reader data1
    :initarg :data1
    :type cl:float
    :initform 0.0)
   (data2
    :reader data2
    :initarg :data2
    :type cl:float
    :initform 0.0))
)

(cl:defclass Set2Float64-request (<Set2Float64-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Set2Float64-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Set2Float64-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rubbing_hand-srv:<Set2Float64-request> is deprecated: use rubbing_hand-srv:Set2Float64-request instead.")))

(cl:ensure-generic-function 'data1-val :lambda-list '(m))
(cl:defmethod data1-val ((m <Set2Float64-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rubbing_hand-srv:data1-val is deprecated.  Use rubbing_hand-srv:data1 instead.")
  (data1 m))

(cl:ensure-generic-function 'data2-val :lambda-list '(m))
(cl:defmethod data2-val ((m <Set2Float64-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rubbing_hand-srv:data2-val is deprecated.  Use rubbing_hand-srv:data2 instead.")
  (data2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Set2Float64-request>) ostream)
  "Serializes a message object of type '<Set2Float64-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'data1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'data2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Set2Float64-request>) istream)
  "Deserializes a message object of type '<Set2Float64-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'data1) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'data2) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Set2Float64-request>)))
  "Returns string type for a service object of type '<Set2Float64-request>"
  "rubbing_hand/Set2Float64Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Set2Float64-request)))
  "Returns string type for a service object of type 'Set2Float64-request"
  "rubbing_hand/Set2Float64Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Set2Float64-request>)))
  "Returns md5sum for a message object of type '<Set2Float64-request>"
  "1a060548b1bc718fe8b44339268f4d29")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Set2Float64-request)))
  "Returns md5sum for a message object of type 'Set2Float64-request"
  "1a060548b1bc718fe8b44339268f4d29")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Set2Float64-request>)))
  "Returns full string definition for message of type '<Set2Float64-request>"
  (cl:format cl:nil "float64 data1~%float64 data2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Set2Float64-request)))
  "Returns full string definition for message of type 'Set2Float64-request"
  (cl:format cl:nil "float64 data1~%float64 data2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Set2Float64-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Set2Float64-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Set2Float64-request
    (cl:cons ':data1 (data1 msg))
    (cl:cons ':data2 (data2 msg))
))
;//! \htmlinclude Set2Float64-response.msg.html

(cl:defclass <Set2Float64-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Set2Float64-response (<Set2Float64-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Set2Float64-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Set2Float64-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rubbing_hand-srv:<Set2Float64-response> is deprecated: use rubbing_hand-srv:Set2Float64-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <Set2Float64-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rubbing_hand-srv:result-val is deprecated.  Use rubbing_hand-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Set2Float64-response>) ostream)
  "Serializes a message object of type '<Set2Float64-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Set2Float64-response>) istream)
  "Deserializes a message object of type '<Set2Float64-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Set2Float64-response>)))
  "Returns string type for a service object of type '<Set2Float64-response>"
  "rubbing_hand/Set2Float64Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Set2Float64-response)))
  "Returns string type for a service object of type 'Set2Float64-response"
  "rubbing_hand/Set2Float64Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Set2Float64-response>)))
  "Returns md5sum for a message object of type '<Set2Float64-response>"
  "1a060548b1bc718fe8b44339268f4d29")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Set2Float64-response)))
  "Returns md5sum for a message object of type 'Set2Float64-response"
  "1a060548b1bc718fe8b44339268f4d29")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Set2Float64-response>)))
  "Returns full string definition for message of type '<Set2Float64-response>"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Set2Float64-response)))
  "Returns full string definition for message of type 'Set2Float64-response"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Set2Float64-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Set2Float64-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Set2Float64-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Set2Float64)))
  'Set2Float64-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Set2Float64)))
  'Set2Float64-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Set2Float64)))
  "Returns string type for a service object of type '<Set2Float64>"
  "rubbing_hand/Set2Float64")