; Auto-generated. Do not edit!


(cl:in-package rubbing_hand-srv)


;//! \htmlinclude SetFloat64_array-request.msg.html

(cl:defclass <SetFloat64_array-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass SetFloat64_array-request (<SetFloat64_array-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetFloat64_array-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetFloat64_array-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rubbing_hand-srv:<SetFloat64_array-request> is deprecated: use rubbing_hand-srv:SetFloat64_array-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <SetFloat64_array-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rubbing_hand-srv:data-val is deprecated.  Use rubbing_hand-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetFloat64_array-request>) ostream)
  "Serializes a message object of type '<SetFloat64_array-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
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
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetFloat64_array-request>) istream)
  "Deserializes a message object of type '<SetFloat64_array-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetFloat64_array-request>)))
  "Returns string type for a service object of type '<SetFloat64_array-request>"
  "rubbing_hand/SetFloat64_arrayRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFloat64_array-request)))
  "Returns string type for a service object of type 'SetFloat64_array-request"
  "rubbing_hand/SetFloat64_arrayRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetFloat64_array-request>)))
  "Returns md5sum for a message object of type '<SetFloat64_array-request>"
  "ae13d2079549ecafaa66d49d24c80c49")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetFloat64_array-request)))
  "Returns md5sum for a message object of type 'SetFloat64_array-request"
  "ae13d2079549ecafaa66d49d24c80c49")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetFloat64_array-request>)))
  "Returns full string definition for message of type '<SetFloat64_array-request>"
  (cl:format cl:nil "float64[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetFloat64_array-request)))
  "Returns full string definition for message of type 'SetFloat64_array-request"
  (cl:format cl:nil "float64[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetFloat64_array-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetFloat64_array-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetFloat64_array-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude SetFloat64_array-response.msg.html

(cl:defclass <SetFloat64_array-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetFloat64_array-response (<SetFloat64_array-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetFloat64_array-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetFloat64_array-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rubbing_hand-srv:<SetFloat64_array-response> is deprecated: use rubbing_hand-srv:SetFloat64_array-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <SetFloat64_array-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rubbing_hand-srv:result-val is deprecated.  Use rubbing_hand-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetFloat64_array-response>) ostream)
  "Serializes a message object of type '<SetFloat64_array-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetFloat64_array-response>) istream)
  "Deserializes a message object of type '<SetFloat64_array-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetFloat64_array-response>)))
  "Returns string type for a service object of type '<SetFloat64_array-response>"
  "rubbing_hand/SetFloat64_arrayResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFloat64_array-response)))
  "Returns string type for a service object of type 'SetFloat64_array-response"
  "rubbing_hand/SetFloat64_arrayResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetFloat64_array-response>)))
  "Returns md5sum for a message object of type '<SetFloat64_array-response>"
  "ae13d2079549ecafaa66d49d24c80c49")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetFloat64_array-response)))
  "Returns md5sum for a message object of type 'SetFloat64_array-response"
  "ae13d2079549ecafaa66d49d24c80c49")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetFloat64_array-response>)))
  "Returns full string definition for message of type '<SetFloat64_array-response>"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetFloat64_array-response)))
  "Returns full string definition for message of type 'SetFloat64_array-response"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetFloat64_array-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetFloat64_array-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetFloat64_array-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetFloat64_array)))
  'SetFloat64_array-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetFloat64_array)))
  'SetFloat64_array-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetFloat64_array)))
  "Returns string type for a service object of type '<SetFloat64_array>"
  "rubbing_hand/SetFloat64_array")