
(cl:in-package :asdf)

(defsystem "msg_interfaces-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "VoxGrid" :depends-on ("_package_VoxGrid"))
    (:file "_package_VoxGrid" :depends-on ("_package"))
  ))