
(cl:in-package :asdf)

(defsystem "rubbing_hand-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SetFloat64_array" :depends-on ("_package_SetFloat64_array"))
    (:file "_package_SetFloat64_array" :depends-on ("_package"))
    (:file "Set2Float64" :depends-on ("_package_Set2Float64"))
    (:file "_package_Set2Float64" :depends-on ("_package"))
    (:file "SetFloat64" :depends-on ("_package_SetFloat64"))
    (:file "_package_SetFloat64" :depends-on ("_package"))
  ))