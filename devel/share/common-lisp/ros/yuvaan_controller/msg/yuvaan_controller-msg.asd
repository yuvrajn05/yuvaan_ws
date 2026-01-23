
(cl:in-package :asdf)

(defsystem "yuvaan_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "drive" :depends-on ("_package_drive"))
    (:file "_package_drive" :depends-on ("_package"))
    (:file "drive_servo" :depends-on ("_package_drive_servo"))
    (:file "_package_drive_servo" :depends-on ("_package"))
    (:file "dual_servo" :depends-on ("_package_dual_servo"))
    (:file "_package_dual_servo" :depends-on ("_package"))
    (:file "mani" :depends-on ("_package_mani"))
    (:file "_package_mani" :depends-on ("_package"))
    (:file "unified_control" :depends-on ("_package_unified_control"))
    (:file "_package_unified_control" :depends-on ("_package"))
    (:file "yuvaan" :depends-on ("_package_yuvaan"))
    (:file "_package_yuvaan" :depends-on ("_package"))
  ))