; Auto-generated. Do not edit!


(cl:in-package rosarnl-msg)


;//! \htmlinclude BatteryStatus.msg.html

(cl:defclass <BatteryStatus> (roslisp-msg-protocol:ros-message)
  ((charging_state
    :reader charging_state
    :initarg :charging_state
    :type cl:fixnum
    :initform 0)
   (charge_percent
    :reader charge_percent
    :initarg :charge_percent
    :type cl:float
    :initform 0.0))
)

(cl:defclass BatteryStatus (<BatteryStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BatteryStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BatteryStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rosarnl-msg:<BatteryStatus> is deprecated: use rosarnl-msg:BatteryStatus instead.")))

(cl:ensure-generic-function 'charging_state-val :lambda-list '(m))
(cl:defmethod charging_state-val ((m <BatteryStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosarnl-msg:charging_state-val is deprecated.  Use rosarnl-msg:charging_state instead.")
  (charging_state m))

(cl:ensure-generic-function 'charge_percent-val :lambda-list '(m))
(cl:defmethod charge_percent-val ((m <BatteryStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rosarnl-msg:charge_percent-val is deprecated.  Use rosarnl-msg:charge_percent instead.")
  (charge_percent m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<BatteryStatus>)))
    "Constants for message type '<BatteryStatus>"
  '((:CHARGING_UNKNOWN . -1)
    (:CHARGING_NOT . 0)
    (:CHARGING_BULK . 1)
    (:CHARGING_OVERCHARGE . 2)
    (:CHARGING_FLOAT . 3)
    (:CHARGING_BALANCE . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'BatteryStatus)))
    "Constants for message type 'BatteryStatus"
  '((:CHARGING_UNKNOWN . -1)
    (:CHARGING_NOT . 0)
    (:CHARGING_BULK . 1)
    (:CHARGING_OVERCHARGE . 2)
    (:CHARGING_FLOAT . 3)
    (:CHARGING_BALANCE . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BatteryStatus>) ostream)
  "Serializes a message object of type '<BatteryStatus>"
  (cl:let* ((signed (cl:slot-value msg 'charging_state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'charge_percent))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BatteryStatus>) istream)
  "Deserializes a message object of type '<BatteryStatus>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'charging_state) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'charge_percent) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BatteryStatus>)))
  "Returns string type for a message object of type '<BatteryStatus>"
  "rosarnl/BatteryStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BatteryStatus)))
  "Returns string type for a message object of type 'BatteryStatus"
  "rosarnl/BatteryStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BatteryStatus>)))
  "Returns md5sum for a message object of type '<BatteryStatus>"
  "a5ae24af1ef085b1c28fd0c2d4869c5f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BatteryStatus)))
  "Returns md5sum for a message object of type 'BatteryStatus"
  "a5ae24af1ef085b1c28fd0c2d4869c5f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BatteryStatus>)))
  "Returns full string definition for message of type '<BatteryStatus>"
  (cl:format cl:nil "int8 CHARGING_UNKNOWN = -1~%int8 CHARGING_NOT = 0~%int8 CHARGING_BULK = 1~%int8 CHARGING_OVERCHARGE = 2~%int8 CHARGING_FLOAT = 3~%int8 CHARGING_BALANCE = 4~%~%int8 charging_state~%float32 charge_percent~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BatteryStatus)))
  "Returns full string definition for message of type 'BatteryStatus"
  (cl:format cl:nil "int8 CHARGING_UNKNOWN = -1~%int8 CHARGING_NOT = 0~%int8 CHARGING_BULK = 1~%int8 CHARGING_OVERCHARGE = 2~%int8 CHARGING_FLOAT = 3~%int8 CHARGING_BALANCE = 4~%~%int8 charging_state~%float32 charge_percent~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BatteryStatus>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BatteryStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'BatteryStatus
    (cl:cons ':charging_state (charging_state msg))
    (cl:cons ':charge_percent (charge_percent msg))
))
