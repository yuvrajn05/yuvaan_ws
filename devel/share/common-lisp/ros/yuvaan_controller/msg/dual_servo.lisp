; Auto-generated. Do not edit!


(cl:in-package yuvaan_controller-msg)


;//! \htmlinclude dual_servo.msg.html

(cl:defclass <dual_servo> (roslisp-msg-protocol:ros-message)
  ((servo1_speed
    :reader servo1_speed
    :initarg :servo1_speed
    :type cl:integer
    :initform 0)
   (servo2_speed
    :reader servo2_speed
    :initarg :servo2_speed
    :type cl:integer
    :initform 0))
)

(cl:defclass dual_servo (<dual_servo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <dual_servo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'dual_servo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yuvaan_controller-msg:<dual_servo> is deprecated: use yuvaan_controller-msg:dual_servo instead.")))

(cl:ensure-generic-function 'servo1_speed-val :lambda-list '(m))
(cl:defmethod servo1_speed-val ((m <dual_servo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:servo1_speed-val is deprecated.  Use yuvaan_controller-msg:servo1_speed instead.")
  (servo1_speed m))

(cl:ensure-generic-function 'servo2_speed-val :lambda-list '(m))
(cl:defmethod servo2_speed-val ((m <dual_servo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:servo2_speed-val is deprecated.  Use yuvaan_controller-msg:servo2_speed instead.")
  (servo2_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <dual_servo>) ostream)
  "Serializes a message object of type '<dual_servo>"
  (cl:let* ((signed (cl:slot-value msg 'servo1_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'servo2_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <dual_servo>) istream)
  "Deserializes a message object of type '<dual_servo>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'servo1_speed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'servo2_speed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<dual_servo>)))
  "Returns string type for a message object of type '<dual_servo>"
  "yuvaan_controller/dual_servo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'dual_servo)))
  "Returns string type for a message object of type 'dual_servo"
  "yuvaan_controller/dual_servo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<dual_servo>)))
  "Returns md5sum for a message object of type '<dual_servo>"
  "af2b50820dddbd7e357854376715f6d2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'dual_servo)))
  "Returns md5sum for a message object of type 'dual_servo"
  "af2b50820dddbd7e357854376715f6d2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<dual_servo>)))
  "Returns full string definition for message of type '<dual_servo>"
  (cl:format cl:nil "# Dual Servo Control Message~%# Controls 2 servos independently using right analog stick~%~%int32 servo1_speed  # Servo 1 speed in microseconds (1000-2000, 1500=stop)~%int32 servo2_speed  # Servo 2 speed in microseconds (1000-2000, 1500=stop)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'dual_servo)))
  "Returns full string definition for message of type 'dual_servo"
  (cl:format cl:nil "# Dual Servo Control Message~%# Controls 2 servos independently using right analog stick~%~%int32 servo1_speed  # Servo 1 speed in microseconds (1000-2000, 1500=stop)~%int32 servo2_speed  # Servo 2 speed in microseconds (1000-2000, 1500=stop)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <dual_servo>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <dual_servo>))
  "Converts a ROS message object to a list"
  (cl:list 'dual_servo
    (cl:cons ':servo1_speed (servo1_speed msg))
    (cl:cons ':servo2_speed (servo2_speed msg))
))
