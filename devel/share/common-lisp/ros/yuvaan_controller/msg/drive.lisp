; Auto-generated. Do not edit!


(cl:in-package yuvaan_controller-msg)


;//! \htmlinclude drive.msg.html

(cl:defclass <drive> (roslisp-msg-protocol:ros-message)
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
    :initform 0))
)

(cl:defclass drive (<drive>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <drive>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'drive)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yuvaan_controller-msg:<drive> is deprecated: use yuvaan_controller-msg:drive instead.")))

(cl:ensure-generic-function 'vel_linear_x-val :lambda-list '(m))
(cl:defmethod vel_linear_x-val ((m <drive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:vel_linear_x-val is deprecated.  Use yuvaan_controller-msg:vel_linear_x instead.")
  (vel_linear_x m))

(cl:ensure-generic-function 'vel_angular_z-val :lambda-list '(m))
(cl:defmethod vel_angular_z-val ((m <drive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:vel_angular_z-val is deprecated.  Use yuvaan_controller-msg:vel_angular_z instead.")
  (vel_angular_z m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <drive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:mode-val is deprecated.  Use yuvaan_controller-msg:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <drive>) ostream)
  "Serializes a message object of type '<drive>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <drive>) istream)
  "Deserializes a message object of type '<drive>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<drive>)))
  "Returns string type for a message object of type '<drive>"
  "yuvaan_controller/drive")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'drive)))
  "Returns string type for a message object of type 'drive"
  "yuvaan_controller/drive")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<drive>)))
  "Returns md5sum for a message object of type '<drive>"
  "b1dd02d8c99c39b8457292a0b8bdd1a9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'drive)))
  "Returns md5sum for a message object of type 'drive"
  "b1dd02d8c99c39b8457292a0b8bdd1a9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<drive>)))
  "Returns full string definition for message of type '<drive>"
  (cl:format cl:nil "int32 vel_linear_x~%int32 vel_angular_z~%int32 mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'drive)))
  "Returns full string definition for message of type 'drive"
  (cl:format cl:nil "int32 vel_linear_x~%int32 vel_angular_z~%int32 mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <drive>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <drive>))
  "Converts a ROS message object to a list"
  (cl:list 'drive
    (cl:cons ':vel_linear_x (vel_linear_x msg))
    (cl:cons ':vel_angular_z (vel_angular_z msg))
    (cl:cons ':mode (mode msg))
))
