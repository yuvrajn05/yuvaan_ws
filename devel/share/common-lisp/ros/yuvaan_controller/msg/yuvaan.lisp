; Auto-generated. Do not edit!


(cl:in-package yuvaan_controller-msg)


;//! \htmlinclude yuvaan.msg.html

(cl:defclass <yuvaan> (roslisp-msg-protocol:ros-message)
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
   (yaw_mode
    :reader yaw_mode
    :initarg :yaw_mode
    :type cl:integer
    :initform 0)
   (roll_mode
    :reader roll_mode
    :initarg :roll_mode
    :type cl:integer
    :initform 0)
   (ra_1
    :reader ra_1
    :initarg :ra_1
    :type cl:integer
    :initform 0)
   (ra_2
    :reader ra_2
    :initarg :ra_2
    :type cl:integer
    :initform 0)
   (ra_3
    :reader ra_3
    :initarg :ra_3
    :type cl:integer
    :initform 0)
   (ra_4
    :reader ra_4
    :initarg :ra_4
    :type cl:integer
    :initform 0)
   (ra_5
    :reader ra_5
    :initarg :ra_5
    :type cl:integer
    :initform 0)
   (ra_6
    :reader ra_6
    :initarg :ra_6
    :type cl:integer
    :initform 0)
   (ba_1
    :reader ba_1
    :initarg :ba_1
    :type cl:integer
    :initform 0)
   (ba_2
    :reader ba_2
    :initarg :ba_2
    :type cl:integer
    :initform 0)
   (ba_3
    :reader ba_3
    :initarg :ba_3
    :type cl:integer
    :initform 0)
   (ba_4
    :reader ba_4
    :initarg :ba_4
    :type cl:integer
    :initform 0)
   (ba_5
    :reader ba_5
    :initarg :ba_5
    :type cl:integer
    :initform 0)
   (ba_6
    :reader ba_6
    :initarg :ba_6
    :type cl:integer
    :initform 0))
)

(cl:defclass yuvaan (<yuvaan>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <yuvaan>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'yuvaan)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yuvaan_controller-msg:<yuvaan> is deprecated: use yuvaan_controller-msg:yuvaan instead.")))

(cl:ensure-generic-function 'vel_linear_x-val :lambda-list '(m))
(cl:defmethod vel_linear_x-val ((m <yuvaan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:vel_linear_x-val is deprecated.  Use yuvaan_controller-msg:vel_linear_x instead.")
  (vel_linear_x m))

(cl:ensure-generic-function 'vel_angular_z-val :lambda-list '(m))
(cl:defmethod vel_angular_z-val ((m <yuvaan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:vel_angular_z-val is deprecated.  Use yuvaan_controller-msg:vel_angular_z instead.")
  (vel_angular_z m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <yuvaan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:mode-val is deprecated.  Use yuvaan_controller-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'yaw_mode-val :lambda-list '(m))
(cl:defmethod yaw_mode-val ((m <yuvaan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:yaw_mode-val is deprecated.  Use yuvaan_controller-msg:yaw_mode instead.")
  (yaw_mode m))

(cl:ensure-generic-function 'roll_mode-val :lambda-list '(m))
(cl:defmethod roll_mode-val ((m <yuvaan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:roll_mode-val is deprecated.  Use yuvaan_controller-msg:roll_mode instead.")
  (roll_mode m))

(cl:ensure-generic-function 'ra_1-val :lambda-list '(m))
(cl:defmethod ra_1-val ((m <yuvaan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:ra_1-val is deprecated.  Use yuvaan_controller-msg:ra_1 instead.")
  (ra_1 m))

(cl:ensure-generic-function 'ra_2-val :lambda-list '(m))
(cl:defmethod ra_2-val ((m <yuvaan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:ra_2-val is deprecated.  Use yuvaan_controller-msg:ra_2 instead.")
  (ra_2 m))

(cl:ensure-generic-function 'ra_3-val :lambda-list '(m))
(cl:defmethod ra_3-val ((m <yuvaan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:ra_3-val is deprecated.  Use yuvaan_controller-msg:ra_3 instead.")
  (ra_3 m))

(cl:ensure-generic-function 'ra_4-val :lambda-list '(m))
(cl:defmethod ra_4-val ((m <yuvaan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:ra_4-val is deprecated.  Use yuvaan_controller-msg:ra_4 instead.")
  (ra_4 m))

(cl:ensure-generic-function 'ra_5-val :lambda-list '(m))
(cl:defmethod ra_5-val ((m <yuvaan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:ra_5-val is deprecated.  Use yuvaan_controller-msg:ra_5 instead.")
  (ra_5 m))

(cl:ensure-generic-function 'ra_6-val :lambda-list '(m))
(cl:defmethod ra_6-val ((m <yuvaan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:ra_6-val is deprecated.  Use yuvaan_controller-msg:ra_6 instead.")
  (ra_6 m))

(cl:ensure-generic-function 'ba_1-val :lambda-list '(m))
(cl:defmethod ba_1-val ((m <yuvaan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:ba_1-val is deprecated.  Use yuvaan_controller-msg:ba_1 instead.")
  (ba_1 m))

(cl:ensure-generic-function 'ba_2-val :lambda-list '(m))
(cl:defmethod ba_2-val ((m <yuvaan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:ba_2-val is deprecated.  Use yuvaan_controller-msg:ba_2 instead.")
  (ba_2 m))

(cl:ensure-generic-function 'ba_3-val :lambda-list '(m))
(cl:defmethod ba_3-val ((m <yuvaan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:ba_3-val is deprecated.  Use yuvaan_controller-msg:ba_3 instead.")
  (ba_3 m))

(cl:ensure-generic-function 'ba_4-val :lambda-list '(m))
(cl:defmethod ba_4-val ((m <yuvaan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:ba_4-val is deprecated.  Use yuvaan_controller-msg:ba_4 instead.")
  (ba_4 m))

(cl:ensure-generic-function 'ba_5-val :lambda-list '(m))
(cl:defmethod ba_5-val ((m <yuvaan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:ba_5-val is deprecated.  Use yuvaan_controller-msg:ba_5 instead.")
  (ba_5 m))

(cl:ensure-generic-function 'ba_6-val :lambda-list '(m))
(cl:defmethod ba_6-val ((m <yuvaan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yuvaan_controller-msg:ba_6-val is deprecated.  Use yuvaan_controller-msg:ba_6 instead.")
  (ba_6 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <yuvaan>) ostream)
  "Serializes a message object of type '<yuvaan>"
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
  (cl:let* ((signed (cl:slot-value msg 'yaw_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'roll_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ra_1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ra_2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ra_3)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ra_4)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ra_5)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ra_6)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ba_1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ba_2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ba_3)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ba_4)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ba_5)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ba_6)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <yuvaan>) istream)
  "Deserializes a message object of type '<yuvaan>"
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
      (cl:setf (cl:slot-value msg 'yaw_mode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'roll_mode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ra_1) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ra_2) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ra_3) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ra_4) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ra_5) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ra_6) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ba_1) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ba_2) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ba_3) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ba_4) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ba_5) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ba_6) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<yuvaan>)))
  "Returns string type for a message object of type '<yuvaan>"
  "yuvaan_controller/yuvaan")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'yuvaan)))
  "Returns string type for a message object of type 'yuvaan"
  "yuvaan_controller/yuvaan")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<yuvaan>)))
  "Returns md5sum for a message object of type '<yuvaan>"
  "f9d9c81a204467304bb82823914c9c9d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'yuvaan)))
  "Returns md5sum for a message object of type 'yuvaan"
  "f9d9c81a204467304bb82823914c9c9d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<yuvaan>)))
  "Returns full string definition for message of type '<yuvaan>"
  (cl:format cl:nil "int32 vel_linear_x~%int32 vel_angular_z~%int32 mode~%int32 yaw_mode~%int32 roll_mode~%int32 ra_1~%int32 ra_2~%int32 ra_3~%int32 ra_4~%int32 ra_5~%int32 ra_6~%int32 ba_1~%int32 ba_2~%int32 ba_3~%int32 ba_4~%int32 ba_5~%int32 ba_6~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'yuvaan)))
  "Returns full string definition for message of type 'yuvaan"
  (cl:format cl:nil "int32 vel_linear_x~%int32 vel_angular_z~%int32 mode~%int32 yaw_mode~%int32 roll_mode~%int32 ra_1~%int32 ra_2~%int32 ra_3~%int32 ra_4~%int32 ra_5~%int32 ra_6~%int32 ba_1~%int32 ba_2~%int32 ba_3~%int32 ba_4~%int32 ba_5~%int32 ba_6~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <yuvaan>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <yuvaan>))
  "Converts a ROS message object to a list"
  (cl:list 'yuvaan
    (cl:cons ':vel_linear_x (vel_linear_x msg))
    (cl:cons ':vel_angular_z (vel_angular_z msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':yaw_mode (yaw_mode msg))
    (cl:cons ':roll_mode (roll_mode msg))
    (cl:cons ':ra_1 (ra_1 msg))
    (cl:cons ':ra_2 (ra_2 msg))
    (cl:cons ':ra_3 (ra_3 msg))
    (cl:cons ':ra_4 (ra_4 msg))
    (cl:cons ':ra_5 (ra_5 msg))
    (cl:cons ':ra_6 (ra_6 msg))
    (cl:cons ':ba_1 (ba_1 msg))
    (cl:cons ':ba_2 (ba_2 msg))
    (cl:cons ':ba_3 (ba_3 msg))
    (cl:cons ':ba_4 (ba_4 msg))
    (cl:cons ':ba_5 (ba_5 msg))
    (cl:cons ':ba_6 (ba_6 msg))
))
