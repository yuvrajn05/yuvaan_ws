;; Auto-generated. Do not edit!


(when (boundp 'yuvaan_controller::yuvaan)
  (if (not (find-package "YUVAAN_CONTROLLER"))
    (make-package "YUVAAN_CONTROLLER"))
  (shadow 'yuvaan (find-package "YUVAAN_CONTROLLER")))
(unless (find-package "YUVAAN_CONTROLLER::YUVAAN")
  (make-package "YUVAAN_CONTROLLER::YUVAAN"))

(in-package "ROS")
;;//! \htmlinclude yuvaan.msg.html


(defclass yuvaan_controller::yuvaan
  :super ros::object
  :slots (_vel_linear_x _vel_angular_z _mode _yaw_mode _roll_mode _ra_1 _ra_2 _ra_3 _ra_4 _ra_5 _ra_6 _ba_1 _ba_2 _ba_3 _ba_4 _ba_5 _ba_6 ))

(defmethod yuvaan_controller::yuvaan
  (:init
   (&key
    ((:vel_linear_x __vel_linear_x) 0)
    ((:vel_angular_z __vel_angular_z) 0)
    ((:mode __mode) 0)
    ((:yaw_mode __yaw_mode) 0)
    ((:roll_mode __roll_mode) 0)
    ((:ra_1 __ra_1) 0)
    ((:ra_2 __ra_2) 0)
    ((:ra_3 __ra_3) 0)
    ((:ra_4 __ra_4) 0)
    ((:ra_5 __ra_5) 0)
    ((:ra_6 __ra_6) 0)
    ((:ba_1 __ba_1) 0)
    ((:ba_2 __ba_2) 0)
    ((:ba_3 __ba_3) 0)
    ((:ba_4 __ba_4) 0)
    ((:ba_5 __ba_5) 0)
    ((:ba_6 __ba_6) 0)
    )
   (send-super :init)
   (setq _vel_linear_x (round __vel_linear_x))
   (setq _vel_angular_z (round __vel_angular_z))
   (setq _mode (round __mode))
   (setq _yaw_mode (round __yaw_mode))
   (setq _roll_mode (round __roll_mode))
   (setq _ra_1 (round __ra_1))
   (setq _ra_2 (round __ra_2))
   (setq _ra_3 (round __ra_3))
   (setq _ra_4 (round __ra_4))
   (setq _ra_5 (round __ra_5))
   (setq _ra_6 (round __ra_6))
   (setq _ba_1 (round __ba_1))
   (setq _ba_2 (round __ba_2))
   (setq _ba_3 (round __ba_3))
   (setq _ba_4 (round __ba_4))
   (setq _ba_5 (round __ba_5))
   (setq _ba_6 (round __ba_6))
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
  (:yaw_mode
   (&optional __yaw_mode)
   (if __yaw_mode (setq _yaw_mode __yaw_mode)) _yaw_mode)
  (:roll_mode
   (&optional __roll_mode)
   (if __roll_mode (setq _roll_mode __roll_mode)) _roll_mode)
  (:ra_1
   (&optional __ra_1)
   (if __ra_1 (setq _ra_1 __ra_1)) _ra_1)
  (:ra_2
   (&optional __ra_2)
   (if __ra_2 (setq _ra_2 __ra_2)) _ra_2)
  (:ra_3
   (&optional __ra_3)
   (if __ra_3 (setq _ra_3 __ra_3)) _ra_3)
  (:ra_4
   (&optional __ra_4)
   (if __ra_4 (setq _ra_4 __ra_4)) _ra_4)
  (:ra_5
   (&optional __ra_5)
   (if __ra_5 (setq _ra_5 __ra_5)) _ra_5)
  (:ra_6
   (&optional __ra_6)
   (if __ra_6 (setq _ra_6 __ra_6)) _ra_6)
  (:ba_1
   (&optional __ba_1)
   (if __ba_1 (setq _ba_1 __ba_1)) _ba_1)
  (:ba_2
   (&optional __ba_2)
   (if __ba_2 (setq _ba_2 __ba_2)) _ba_2)
  (:ba_3
   (&optional __ba_3)
   (if __ba_3 (setq _ba_3 __ba_3)) _ba_3)
  (:ba_4
   (&optional __ba_4)
   (if __ba_4 (setq _ba_4 __ba_4)) _ba_4)
  (:ba_5
   (&optional __ba_5)
   (if __ba_5 (setq _ba_5 __ba_5)) _ba_5)
  (:ba_6
   (&optional __ba_6)
   (if __ba_6 (setq _ba_6 __ba_6)) _ba_6)
  (:serialization-length
   ()
   (+
    ;; int32 _vel_linear_x
    4
    ;; int32 _vel_angular_z
    4
    ;; int32 _mode
    4
    ;; int32 _yaw_mode
    4
    ;; int32 _roll_mode
    4
    ;; int32 _ra_1
    4
    ;; int32 _ra_2
    4
    ;; int32 _ra_3
    4
    ;; int32 _ra_4
    4
    ;; int32 _ra_5
    4
    ;; int32 _ra_6
    4
    ;; int32 _ba_1
    4
    ;; int32 _ba_2
    4
    ;; int32 _ba_3
    4
    ;; int32 _ba_4
    4
    ;; int32 _ba_5
    4
    ;; int32 _ba_6
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
     ;; int32 _yaw_mode
       (write-long _yaw_mode s)
     ;; int32 _roll_mode
       (write-long _roll_mode s)
     ;; int32 _ra_1
       (write-long _ra_1 s)
     ;; int32 _ra_2
       (write-long _ra_2 s)
     ;; int32 _ra_3
       (write-long _ra_3 s)
     ;; int32 _ra_4
       (write-long _ra_4 s)
     ;; int32 _ra_5
       (write-long _ra_5 s)
     ;; int32 _ra_6
       (write-long _ra_6 s)
     ;; int32 _ba_1
       (write-long _ba_1 s)
     ;; int32 _ba_2
       (write-long _ba_2 s)
     ;; int32 _ba_3
       (write-long _ba_3 s)
     ;; int32 _ba_4
       (write-long _ba_4 s)
     ;; int32 _ba_5
       (write-long _ba_5 s)
     ;; int32 _ba_6
       (write-long _ba_6 s)
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
   ;; int32 _yaw_mode
     (setq _yaw_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _roll_mode
     (setq _roll_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ra_1
     (setq _ra_1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ra_2
     (setq _ra_2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ra_3
     (setq _ra_3 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ra_4
     (setq _ra_4 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ra_5
     (setq _ra_5 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ra_6
     (setq _ra_6 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ba_1
     (setq _ba_1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ba_2
     (setq _ba_2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ba_3
     (setq _ba_3 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ba_4
     (setq _ba_4 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ba_5
     (setq _ba_5 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _ba_6
     (setq _ba_6 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get yuvaan_controller::yuvaan :md5sum-) "f9d9c81a204467304bb82823914c9c9d")
(setf (get yuvaan_controller::yuvaan :datatype-) "yuvaan_controller/yuvaan")
(setf (get yuvaan_controller::yuvaan :definition-)
      "int32 vel_linear_x
int32 vel_angular_z
int32 mode
int32 yaw_mode
int32 roll_mode
int32 ra_1
int32 ra_2
int32 ra_3
int32 ra_4
int32 ra_5
int32 ra_6
int32 ba_1
int32 ba_2
int32 ba_3
int32 ba_4
int32 ba_5
int32 ba_6
")



(provide :yuvaan_controller/yuvaan "f9d9c81a204467304bb82823914c9c9d")


