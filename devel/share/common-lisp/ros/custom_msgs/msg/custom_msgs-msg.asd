
(cl:in-package :asdf)

(defsystem "custom_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Cone" :depends-on ("_package_Cone"))
    (:file "_package_Cone" :depends-on ("_package"))
    (:file "Map" :depends-on ("_package_Map"))
    (:file "_package_Map" :depends-on ("_package"))
  ))