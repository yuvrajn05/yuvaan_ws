; Auto-generated. Do not edit!


(cl:in-package yuvaan_controller-msg)


;//! \htmlinclude drive_servo.msg.html

(cl:defclass <drive_servo> (roslisp-msg-protocol:ros-message)
  ((vel_linear_x
    :reader vel_linear_x
    :initarg :vel_linear_x
    :type cl:integer
    :initform 0)
   (vel_angular_z
    :reader vel_angular_z
    :initarg :vel_angular_z
    :type cl:integer
    :initform 0)
   (mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0)
   (servo_speed
    :reader servo_speed
    :initarg :servo_speed
    :type cl:integer
    :initform 0))
)

(cl:defclass drive_servo (<drive_servo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <drive_servo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'drive_servo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yuvaan_controller-msg:<drive_servo> is deprecated: use yuvaan_controller-msg:drive_servo instead.")))

(cl:ensure-generic-function 'vel_linear_x-val :lambda-list '(m))
(cl:defmethod vel_linear_x-val ((m <drive_servo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:vel_linear_x-val is deprecated.  Use yuvaan_controller-msg:vel_linear_x instead.")
  (vel_linear_x m))

(cl:ensure-generic-function 'vel_angular_z-val :lambda-list '(m))
(cl:defmethod vel_angular_z-val ((m <drive_servo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:vel_angular_z-val is deprecated.  Use yuvaan_controller-msg:vel_angular_z instead.")
  (vel_angular_z m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <drive_servo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:mode-val is deprecated.  Use yuvaan_controller-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'servo_speed-val :lambda-list '(m))
(cl:defmethod servo_speed-val ((m <drive_servo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:servo_speed-val is deprecated.  Use yuvaan_controller-msg:servo_speed instead.")
  (servo_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <drive_servo>) ostream)
  "Serializes a message object of type '<drive_servo>"
  (cl:let* ((signed (cl:slot-value msg 'vel_linear_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'vel_angular_z)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'servo_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <drive_servo>) istream)
  "Deserializes a message object of type '<drive_servo>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'vel_linear_x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'vel_angular_z) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'servo_speed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<drive_servo>)))
  "Returns string type for a message object of type '<drive_servo>"
  "yuvaan_controller/drive_servo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'drive_servo)))
  "Returns string type for a message object of type 'drive_servo"
  "yuvaan_controller/drive_servo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<drive_servo>)))
  "Returns md5sum for a message object of type '<drive_servo>"
  "f038d4ff1f38bef575d3a34d6c844c6d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'drive_servo)))
  "Returns md5sum for a message object of type 'drive_servo"
  "f038d4ff1f38bef575d3a34d6c844c6d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<drive_servo>)))
  "Returns full string definition for message of type '<drive_servo>"
  (cl:format cl:nil "# Drive and Servo Control Message~%# Combines differential drive control with continuous rotation servo control~%~%# Drive control~%int32 vel_linear_x    # Linear velocity (-255 to 255)~%int32 vel_angular_z   # Angular velocity (-255 to 255)~%int32 mode            # Drive mode (1=slow, 2=medium, 3=fast, 4=max)~%~%# 360° Servo control (microseconds for precision)~%int32 servo_speed     # Servo speed in microseconds (1000-2000, 1500=stop)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'drive_servo)))
  "Returns full string definition for message of type 'drive_servo"
  (cl:format cl:nil "# Drive and Servo Control Message~%# Combines differential drive control with continuous rotation servo control~%~%# Drive control~%int32 vel_linear_x    # Linear velocity (-255 to 255)~%int32 vel_angular_z   # Angular velocity (-255 to 255)~%int32 mode            # Drive mode (1=slow, 2=medium, 3=fast, 4=max)~%~%# 360° Servo control (microseconds for precision)~%int32 servo_speed     # Servo speed in microseconds (1000-2000, 1500=stop)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <drive_servo>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <drive_servo>))
  "Converts a ROS message object to a list"
  (cl:list 'drive_servo
    (cl:cons ':vel_linear_x (vel_linear_x msg))
    (cl:cons ':vel_angular_z (vel_angular_z msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':servo_speed (servo_speed msg))
))
