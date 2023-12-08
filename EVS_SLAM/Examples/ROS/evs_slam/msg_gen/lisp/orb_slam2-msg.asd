
(cl:in-package :asdf)

(defsystem "orb_slam2-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "array2D" :depends-on ("_package_array2D"))
    (:file "_package_array2D" :depends-on ("_package"))
    (:file "Num" :depends-on ("_package_Num"))
    (:file "_package_Num" :depends-on ("_package"))
    (:file "msg_mask" :depends-on ("_package_msg_mask"))
    (:file "_package_msg_mask" :depends-on ("_package"))
  ))