; Auto-generated. Do not edit!


(cl:in-package cyborg_controller-msg)


;//! \htmlinclude EmotionalFeedback.msg.html

(cl:defclass <EmotionalFeedback> (roslisp-msg-protocol:ros-message)
  ((delta_pleasure
    :reader delta_pleasure
    :initarg :delta_pleasure
    :type cl:float
    :initform 0.0)
   (delta_arousal
    :reader delta_arousal
    :initarg :delta_arousal
    :type cl:float
    :initform 0.0)
   (delta_dominance
    :reader delta_dominance
    :initarg :delta_dominance
    :type cl:float
    :initform 0.0))
)

(cl:defclass EmotionalFeedback (<EmotionalFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EmotionalFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EmotionalFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cyborg_controller-msg:<EmotionalFeedback> is deprecated: use cyborg_controller-msg:EmotionalFeedback instead.")))

(cl:ensure-generic-function 'delta_pleasure-val :lambda-list '(m))
(cl:defmethod delta_pleasure-val ((m <EmotionalFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyborg_controller-msg:delta_pleasure-val is deprecated.  Use cyborg_controller-msg:delta_pleasure instead.")
  (delta_pleasure m))

(cl:ensure-generic-function 'delta_arousal-val :lambda-list '(m))
(cl:defmethod delta_arousal-val ((m <EmotionalFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyborg_controller-msg:delta_arousal-val is deprecated.  Use cyborg_controller-msg:delta_arousal instead.")
  (delta_arousal m))

(cl:ensure-generic-function 'delta_dominance-val :lambda-list '(m))
(cl:defmethod delta_dominance-val ((m <EmotionalFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyborg_controller-msg:delta_dominance-val is deprecated.  Use cyborg_controller-msg:delta_dominance instead.")
  (delta_dominance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EmotionalFeedback>) ostream)
  "Serializes a message object of type '<EmotionalFeedback>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'delta_pleasure))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'delta_arousal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'delta_dominance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EmotionalFeedback>) istream)
  "Deserializes a message object of type '<EmotionalFeedback>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_pleasure) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_arousal) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'delta_dominance) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EmotionalFeedback>)))
  "Returns string type for a message object of type '<EmotionalFeedback>"
  "cyborg_controller/EmotionalFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EmotionalFeedback)))
  "Returns string type for a message object of type 'EmotionalFeedback"
  "cyborg_controller/EmotionalFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EmotionalFeedback>)))
  "Returns md5sum for a message object of type '<EmotionalFeedback>"
  "6f61bf537c02522f197246d02ded0f2c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EmotionalFeedback)))
  "Returns md5sum for a message object of type 'EmotionalFeedback"
  "6f61bf537c02522f197246d02ded0f2c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EmotionalFeedback>)))
  "Returns full string definition for message of type '<EmotionalFeedback>"
  (cl:format cl:nil "float32 delta_pleasure # Changes in pleasure~%float32 delta_arousal # Changes in arousal~%float32 delta_dominance #Changes in dominance~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EmotionalFeedback)))
  "Returns full string definition for message of type 'EmotionalFeedback"
  (cl:format cl:nil "float32 delta_pleasure # Changes in pleasure~%float32 delta_arousal # Changes in arousal~%float32 delta_dominance #Changes in dominance~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EmotionalFeedback>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EmotionalFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'EmotionalFeedback
    (cl:cons ':delta_pleasure (delta_pleasure msg))
    (cl:cons ':delta_arousal (delta_arousal msg))
    (cl:cons ':delta_dominance (delta_dominance msg))
))
