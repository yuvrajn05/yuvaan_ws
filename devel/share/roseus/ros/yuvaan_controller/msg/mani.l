;; Auto-generated. Do not edit!


(when (boundp 'yuvaan_controller::mani)
  (if (not (find-package "YUVAAN_CONTROLLER"))
    (make-package "YUVAAN_CONTROLLER"))
  (shadow 'mani (find-package "YUVAAN_CONTROLLER")))
(unless (find-package "YUVAAN_CONTROLLER::MANI")
  (make-package "YUVAAN_CONTROLLER::MANI"))

(in-package "ROS")
;;//! \htmlinclude mani.msg.html


(defclass yuvaan_controller::mani
  :super ros::object
  :slots (_yaw_mode _roll_mode _ra_1 _ra_2 _ra_3 _ra_4 _ra_5 _ra_6 ))

(defmethod yuvaan_controller::mani
  (:init
   (&key
    ((:yaw_mode __yaw_mode) 0)
    ((:roll_mode __roll_mode) 0)
    ((:ra_1 __ra_1) 0)
    ((:ra_2 __ra_2) 0)
    ((:ra_3 __ra_3) 0)
    ((:ra_4 __ra_4) 0)
    ((:ra_5 __ra_5) 0)
    ((:ra_6 __ra_6) 0)
    )
   (send-super :init)
   (setq _yaw_mode (round __yaw_mode))
   (setq _roll_mode (round __roll_mode))
   (setq _ra_1 (round __ra_1))
   (setq _ra_2 (round __ra_2))
   (setq _ra_3 (round __ra_3))
   (setq _ra_4 (round __ra_4))
   (setq _ra_5 (round __ra_5))
   (setq _ra_6 (round __ra_6))
   self)
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
  (:serialization-length
   ()
   (+
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
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
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
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
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
   ;;
   self)
  )

(setf (get yuvaan_controller::mani :md5sum-) "d4cb1b85a0be84892c13244d118cfb38")
(setf (get yuvaan_controller::mani :datatype-) "yuvaan_controller/mani")
(setf (get yuvaan_controller::mani :definition-)
      "int32 yaw_mode
int32 roll_mode
int32 ra_1
int32 ra_2
int32 ra_3
int32 ra_4
int32 ra_5
int32 ra_6
")



(provide :yuvaan_controller/mani "d4cb1b85a0be84892c13244d118cfb38")


