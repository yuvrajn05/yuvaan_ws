;; Auto-generated. Do not edit!


(when (boundp 'yuvaan_controller::dual_servo)
  (if (not (find-package "YUVAAN_CONTROLLER"))
    (make-package "YUVAAN_CONTROLLER"))
  (shadow 'dual_servo (find-package "YUVAAN_CONTROLLER")))
(unless (find-package "YUVAAN_CONTROLLER::DUAL_SERVO")
  (make-package "YUVAAN_CONTROLLER::DUAL_SERVO"))

(in-package "ROS")
;;//! \htmlinclude dual_servo.msg.html


(defclass yuvaan_controller::dual_servo
  :super ros::object
  :slots (_servo1_speed _servo2_speed ))

(defmethod yuvaan_controller::dual_servo
  (:init
   (&key
    ((:servo1_speed __servo1_speed) 0)
    ((:servo2_speed __servo2_speed) 0)
    )
   (send-super :init)
   (setq _servo1_speed (round __servo1_speed))
   (setq _servo2_speed (round __servo2_speed))
   self)
  (:servo1_speed
   (&optional __servo1_speed)
   (if __servo1_speed (setq _servo1_speed __servo1_speed)) _servo1_speed)
  (:servo2_speed
   (&optional __servo2_speed)
   (if __servo2_speed (setq _servo2_speed __servo2_speed)) _servo2_speed)
  (:serialization-length
   ()
   (+
    ;; int32 _servo1_speed
    4
    ;; int32 _servo2_speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _servo1_speed
       (write-long _servo1_speed s)
     ;; int32 _servo2_speed
       (write-long _servo2_speed s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _servo1_speed
     (setq _servo1_speed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _servo2_speed
     (setq _servo2_speed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get yuvaan_controller::dual_servo :md5sum-) "af2b50820dddbd7e357854376715f6d2")
(setf (get yuvaan_controller::dual_servo :datatype-) "yuvaan_controller/dual_servo")
(setf (get yuvaan_controller::dual_servo :definition-)
      "# Dual Servo Control Message
# Controls 2 servos independently using right analog stick

int32 servo1_speed  # Servo 1 speed in microseconds (1000-2000, 1500=stop)
int32 servo2_speed  # Servo 2 speed in microseconds (1000-2000, 1500=stop)

")



(provide :yuvaan_controller/dual_servo "af2b50820dddbd7e357854376715f6d2")


