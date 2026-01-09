;; Auto-generated. Do not edit!


(when (boundp 'yuvaan_controller::drive)
  (if (not (find-package "YUVAAN_CONTROLLER"))
    (make-package "YUVAAN_CONTROLLER"))
  (shadow 'drive (find-package "YUVAAN_CONTROLLER")))
(unless (find-package "YUVAAN_CONTROLLER::DRIVE")
  (make-package "YUVAAN_CONTROLLER::DRIVE"))

(in-package "ROS")
;;//! \htmlinclude drive.msg.html


(defclass yuvaan_controller::drive
  :super ros::object
  :slots (_vel_linear_x _vel_angular_z _mode ))

(defmethod yuvaan_controller::drive
  (:init
   (&key
    ((:vel_linear_x __vel_linear_x) 0)
    ((:vel_angular_z __vel_angular_z) 0)
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _vel_linear_x (round __vel_linear_x))
   (setq _vel_angular_z (round __vel_angular_z))
   (setq _mode (round __mode))
   self)
  (:vel_linear_x
   (&optional __vel_linear_x)
   (if __vel_linear_x (setq _vel_linear_x __vel_linear_x)) _vel_linear_x)
  (:vel_angular_z
   (&optional __vel_angular_z)
   (if __vel_angular_z (setq _vel_angular_z __vel_angular_z)) _vel_angular_z)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; int32 _vel_linear_x
    4
    ;; int32 _vel_angular_z
    4
    ;; int32 _mode
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _vel_linear_x
       (write-long _vel_linear_x s)
     ;; int32 _vel_angular_z
       (write-long _vel_angular_z s)
     ;; int32 _mode
       (write-long _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _vel_linear_x
     (setq _vel_linear_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _vel_angular_z
     (setq _vel_angular_z (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _mode
     (setq _mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get yuvaan_controller::drive :md5sum-) "b1dd02d8c99c39b8457292a0b8bdd1a9")
(setf (get yuvaan_controller::drive :datatype-) "yuvaan_controller/drive")
(setf (get yuvaan_controller::drive :definition-)
      "int32 vel_linear_x
int32 vel_angular_z
int32 mode
")



(provide :yuvaan_controller/drive "b1dd02d8c99c39b8457292a0b8bdd1a9")


