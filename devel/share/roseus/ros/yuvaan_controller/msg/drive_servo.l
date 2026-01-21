;; Auto-generated. Do not edit!


(when (boundp 'yuvaan_controller::drive_servo)
  (if (not (find-package "YUVAAN_CONTROLLER"))
    (make-package "YUVAAN_CONTROLLER"))
  (shadow 'drive_servo (find-package "YUVAAN_CONTROLLER")))
(unless (find-package "YUVAAN_CONTROLLER::DRIVE_SERVO")
  (make-package "YUVAAN_CONTROLLER::DRIVE_SERVO"))

(in-package "ROS")
;;//! \htmlinclude drive_servo.msg.html


(defclass yuvaan_controller::drive_servo
  :super ros::object
  :slots (_vel_linear_x _vel_angular_z _mode _servo_speed ))

(defmethod yuvaan_controller::drive_servo
  (:init
   (&key
    ((:vel_linear_x __vel_linear_x) 0)
    ((:vel_angular_z __vel_angular_z) 0)
    ((:mode __mode) 0)
    ((:servo_speed __servo_speed) 0)
    )
   (send-super :init)
   (setq _vel_linear_x (round __vel_linear_x))
   (setq _vel_angular_z (round __vel_angular_z))
   (setq _mode (round __mode))
   (setq _servo_speed (round __servo_speed))
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
  (:servo_speed
   (&optional __servo_speed)
   (if __servo_speed (setq _servo_speed __servo_speed)) _servo_speed)
  (:serialization-length
   ()
   (+
    ;; int32 _vel_linear_x
    4
    ;; int32 _vel_angular_z
    4
    ;; int32 _mode
    4
    ;; int32 _servo_speed
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
     ;; int32 _servo_speed
       (write-long _servo_speed s)
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
   ;; int32 _servo_speed
     (setq _servo_speed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get yuvaan_controller::drive_servo :md5sum-) "f038d4ff1f38bef575d3a34d6c844c6d")
(setf (get yuvaan_controller::drive_servo :datatype-) "yuvaan_controller/drive_servo")
(setf (get yuvaan_controller::drive_servo :definition-)
      "# Drive and Servo Control Message
# Combines differential drive control with continuous rotation servo control

# Drive control
int32 vel_linear_x    # Linear velocity (-255 to 255)
int32 vel_angular_z   # Angular velocity (-255 to 255)
int32 mode            # Drive mode (1=slow, 2=medium, 3=fast, 4=max)

# 360° Servo control (microseconds for precision)
int32 servo_speed     # Servo speed in microseconds (1000-2000, 1500=stop)

")



(provide :yuvaan_controller/drive_servo "f038d4ff1f38bef575d3a34d6c844c6d")


