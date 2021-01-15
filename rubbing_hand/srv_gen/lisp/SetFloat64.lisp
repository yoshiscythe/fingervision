; Auto-generated. Do not edit!


(cl:in-package rubbing_hand-srv)


;//! \htmlinclude SetFloat64-request.msg.html

(cl:defclass <SetFloat64-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetFloat64-request (<SetFloat64-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetFloat64-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetFloat64-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rubbing_hand-srv:<SetFloat64-request> is deprecated: use rubbing_hand-srv:SetFloat64-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <SetFloat64-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rubbing_hand-srv:data-val is deprecated.  Use rubbing_hand-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetFloat64-request>) ostream)
  "Serializes a message object of type '<SetFloat64-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetFloat64-request>) istream)
  "Deserializes a message object of type '<SetFloat64-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'data) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetFloat64-request>)))
  "Returns string type for a service object of type '<SetFloat64-request>"
  "rubbing_hand/SetFloat64Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFloat64-request)))
  "Returns string type for a service object of type 'SetFloat64-request"
  "rubbing_hand/SetFloat64Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetFloat64-request>)))
  "Returns md5sum for a message object of type '<SetFloat64-request>"
  "e914a2ad5395f6f66880d5aed68f5700")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetFloat64-request)))
  "Returns md5sum for a message object of type 'SetFloat64-request"
  "e914a2ad5395f6f66880d5aed68f5700")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetFloat64-request>)))
  "Returns full string definition for message of type '<SetFloat64-request>"
  (cl:format cl:nil "float64 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetFloat64-request)))
  "Returns full string definition for message of type 'SetFloat64-request"
  (cl:format cl:nil "float64 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetFloat64-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetFloat64-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetFloat64-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude SetFloat64-response.msg.html

(cl:defclass <SetFloat64-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetFloat64-response (<SetFloat64-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetFloat64-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetFloat64-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rubbing_hand-srv:<SetFloat64-response> is deprecated: use rubbing_hand-srv:SetFloat64-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <SetFloat64-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rubbing_hand-srv:result-val is deprecated.  Use rubbing_hand-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetFloat64-response>) ostream)
  "Serializes a message object of type '<SetFloat64-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetFloat64-response>) istream)
  "Deserializes a message object of type '<SetFloat64-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetFloat64-response>)))
  "Returns string type for a service object of type '<SetFloat64-response>"
  "rubbing_hand/SetFloat64Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFloat64-response)))
  "Returns string type for a service object of type 'SetFloat64-response"
  "rubbing_hand/SetFloat64Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetFloat64-response>)))
  "Returns md5sum for a message object of type '<SetFloat64-response>"
  "e914a2ad5395f6f66880d5aed68f5700")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetFloat64-response)))
  "Returns md5sum for a message object of type 'SetFloat64-response"
  "e914a2ad5395f6f66880d5aed68f5700")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetFloat64-response>)))
  "Returns full string definition for message of type '<SetFloat64-response>"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetFloat64-response)))
  "Returns full string definition for message of type 'SetFloat64-response"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetFloat64-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetFloat64-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetFloat64-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetFloat64)))
  'SetFloat64-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetFloat64)))
  'SetFloat64-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFloat64)))
  "Returns string type for a service object of type '<SetFloat64>"
  "rubbing_hand/SetFloat64")