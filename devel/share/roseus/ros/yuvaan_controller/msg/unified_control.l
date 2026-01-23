;; Auto-generated. Do not edit!


(when (boundp 'yuvaan_controller::unified_control)
  (if (not (find-package "YUVAAN_CONTROLLER"))
    (make-package "YUVAAN_CONTROLLER"))
  (shadow 'unified_control (find-package "YUVAAN_CONTROLLER")))
(unless (find-package "YUVAAN_CONTROLLER::UNIFIED_CONTROL")
  (make-package "YUVAAN_CONTROLLER::UNIFIED_CONTROL"))

(in-package "ROS")
;;//! \htmlinclude unified_control.msg.html


(defclass yuvaan_controller::unified_control
  :super ros::object
  :slots (_vel_linear_x _vel_angular_z _mode _servo_dpad _servo_stick_x _servo_stick_y ))

(defmethod yuvaan_controller::unified_control
  (:init
   (&key
    ((:vel_linear_x __vel_linear_x) 0)
    ((:vel_angular_z __vel_angular_z) 0)
    ((:mode __mode) 0)
    ((:servo_dpad __servo_dpad) 0)
    ((:servo_stick_x __servo_stick_x) 0)
    ((:servo_stick_y __servo_stick_y) 0)
    )
   (send-super :init)
   (setq _vel_linear_x (round __vel_linear_x))
   (setq _vel_angular_z (round __vel_angular_z))
   (setq _mode (round __mode))
   (setq _servo_dpad (round __servo_dpad))
   (setq _servo_stick_x (round __servo_stick_x))
   (setq _servo_stick_y (round __servo_stick_y))
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
  (:servo_dpad
   (&optional __servo_dpad)
   (if __servo_dpad (setq _servo_dpad __servo_dpad)) _servo_dpad)
  (:servo_stick_x
   (&optional __servo_stick_x)
   (if __servo_stick_x (setq _servo_stick_x __servo_stick_x)) _servo_stick_x)
  (:servo_stick_y
   (&optional __servo_stick_y)
   (if __servo_stick_y (setq _servo_stick_y __servo_stick_y)) _servo_stick_y)
  (:serialization-length
   ()
   (+
    ;; int32 _vel_linear_x
    4
    ;; int32 _vel_angular_z
    4
    ;; int32 _mode
    4
    ;; int32 _servo_dpad
    4
    ;; int32 _servo_stick_x
    4
    ;; int32 _servo_stick_y
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
     ;; int32 _servo_dpad
       (write-long _servo_dpad s)
     ;; int32 _servo_stick_x
       (write-long _servo_stick_x s)
     ;; int32 _servo_stick_y
       (write-long _servo_stick_y s)
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
   ;; int32 _servo_dpad
     (setq _servo_dpad (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _servo_stick_x
     (setq _servo_stick_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _servo_stick_y
     (setq _servo_stick_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get yuvaan_controller::unified_control :md5sum-) "4f2a61c6bf7b2e257f911489b14b7b6b")
(setf (get yuvaan_controller::unified_control :datatype-) "yuvaan_controller/unified_control")
(setf (get yuvaan_controller::unified_control :definition-)
      "# Unified Control Message
# Combines drive control + 3 servos (1 D-pad + 2 analog stick)

# Drive control
int32 vel_linear_x    # Linear velocity (-255 to 255)
int32 vel_angular_z   # Angular velocity (-255 to 255)
int32 mode            # Drive mode (1=slow, 2=medium, 3=fast, 4=max)

# Servo control (all in microseconds, 1500=stop)
int32 servo_dpad      # D-pad controlled servo (1000-2000μs)
int32 servo_stick_x   # Right stick X servo (1000-2000μs)
int32 servo_stick_y   # Right stick Y servo (1000-2000μs)

")



(provide :yuvaan_controller/unified_control "4f2a61c6bf7b2e257f911489b14b7b6b")


