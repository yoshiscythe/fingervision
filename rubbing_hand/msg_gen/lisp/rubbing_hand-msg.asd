
(cl:in-package :asdf)

(defsystem "rubbing_hand-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Float64" :depends-on ("_package_Float64"))
    (:file "_package_Float64" :depends-on ("_package"))
    (:file "dynamixel_msg" :depends-on ("_package_dynamixel_msg"))
    (:file "_package_dynamixel_msg" :depends-on ("_package"))
    (:file "Float64Array" :depends-on ("_package_Float64Array"))
    (:file "_package_Float64Array" :depends-on ("_package"))
    (:file "dynamixel_param_msg" :depends-on ("_package_dynamixel_param_msg"))
    (:file "_package_dynamixel_param_msg" :depends-on ("_package"))
  ))