; Auto-generated. Do not edit!


(cl:in-package yuvaan_controller-msg)


;//! \htmlinclude unified_control.msg.html

(cl:defclass <unified_control> (roslisp-msg-protocol:ros-message)
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
   (servo_dpad
    :reader servo_dpad
    :initarg :servo_dpad
    :type cl:integer
    :initform 0)
   (servo_stick_x
    :reader servo_stick_x
    :initarg :servo_stick_x
    :type cl:integer
    :initform 0)
   (servo_stick_y
    :reader servo_stick_y
    :initarg :servo_stick_y
    :type cl:integer
    :initform 0))
)

(cl:defclass unified_control (<unified_control>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <unified_control>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'unified_control)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yuvaan_controller-msg:<unified_control> is deprecated: use yuvaan_controller-msg:unified_control instead.")))

(cl:ensure-generic-function 'vel_linear_x-val :lambda-list '(m))
(cl:defmethod vel_linear_x-val ((m <unified_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:vel_linear_x-val is deprecated.  Use yuvaan_controller-msg:vel_linear_x instead.")
  (vel_linear_x m))

(cl:ensure-generic-function 'vel_angular_z-val :lambda-list '(m))
(cl:defmethod vel_angular_z-val ((m <unified_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:vel_angular_z-val is deprecated.  Use yuvaan_controller-msg:vel_angular_z instead.")
  (vel_angular_z m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <unified_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:mode-val is deprecated.  Use yuvaan_controller-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'servo_dpad-val :lambda-list '(m))
(cl:defmethod servo_dpad-val ((m <unified_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:servo_dpad-val is deprecated.  Use yuvaan_controller-msg:servo_dpad instead.")
  (servo_dpad m))

(cl:ensure-generic-function 'servo_stick_x-val :lambda-list '(m))
(cl:defmethod servo_stick_x-val ((m <unified_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:servo_stick_x-val is deprecated.  Use yuvaan_controller-msg:servo_stick_x instead.")
  (servo_stick_x m))

(cl:ensure-generic-function 'servo_stick_y-val :lambda-list '(m))
(cl:defmethod servo_stick_y-val ((m <unified_control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:servo_stick_y-val is deprecated.  Use yuvaan_controller-msg:servo_stick_y instead.")
  (servo_stick_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <unified_control>) ostream)
  "Serializes a message object of type '<unified_control>"
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
  (cl:let* ((signed (cl:slot-value msg 'servo_dpad)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'servo_stick_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'servo_stick_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <unified_control>) istream)
  "Deserializes a message object of type '<unified_control>"
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
      (cl:setf (cl:slot-value msg 'servo_dpad) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'servo_stick_x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'servo_stick_y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<unified_control>)))
  "Returns string type for a message object of type '<unified_control>"
  "yuvaan_controller/unified_control")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'unified_control)))
  "Returns string type for a message object of type 'unified_control"
  "yuvaan_controller/unified_control")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<unified_control>)))
  "Returns md5sum for a message object of type '<unified_control>"
  "4f2a61c6bf7b2e257f911489b14b7b6b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'unified_control)))
  "Returns md5sum for a message object of type 'unified_control"
  "4f2a61c6bf7b2e257f911489b14b7b6b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<unified_control>)))
  "Returns full string definition for message of type '<unified_control>"
  (cl:format cl:nil "# Unified Control Message~%# Combines drive control + 3 servos (1 D-pad + 2 analog stick)~%~%# Drive control~%int32 vel_linear_x    # Linear velocity (-255 to 255)~%int32 vel_angular_z   # Angular velocity (-255 to 255)~%int32 mode            # Drive mode (1=slow, 2=medium, 3=fast, 4=max)~%~%# Servo control (all in microseconds, 1500=stop)~%int32 servo_dpad      # D-pad controlled servo (1000-2000μs)~%int32 servo_stick_x   # Right stick X servo (1000-2000μs)~%int32 servo_stick_y   # Right stick Y servo (1000-2000μs)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'unified_control)))
  "Returns full string definition for message of type 'unified_control"
  (cl:format cl:nil "# Unified Control Message~%# Combines drive control + 3 servos (1 D-pad + 2 analog stick)~%~%# Drive control~%int32 vel_linear_x    # Linear velocity (-255 to 255)~%int32 vel_angular_z   # Angular velocity (-255 to 255)~%int32 mode            # Drive mode (1=slow, 2=medium, 3=fast, 4=max)~%~%# Servo control (all in microseconds, 1500=stop)~%int32 servo_dpad      # D-pad controlled servo (1000-2000μs)~%int32 servo_stick_x   # Right stick X servo (1000-2000μs)~%int32 servo_stick_y   # Right stick Y servo (1000-2000μs)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <unified_control>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <unified_control>))
  "Converts a ROS message object to a list"
  (cl:list 'unified_control
    (cl:cons ':vel_linear_x (vel_linear_x msg))
    (cl:cons ':vel_angular_z (vel_angular_z msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':servo_dpad (servo_dpad msg))
    (cl:cons ':servo_stick_x (servo_stick_x msg))
    (cl:cons ':servo_stick_y (servo_stick_y msg))
))
