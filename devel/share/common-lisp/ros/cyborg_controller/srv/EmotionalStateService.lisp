; Auto-generated. Do not edit!


(cl:in-package cyborg_controller-srv)


;//! \htmlinclude EmotionalStateService-request.msg.html

(cl:defclass <EmotionalStateService-request> (roslisp-msg-protocol:ros-message)
  ((nothing
    :reader nothing
    :initarg :nothing
    :type cl:string
    :initform ""))
)

(cl:defclass EmotionalStateService-request (<EmotionalStateService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EmotionalStateService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EmotionalStateService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cyborg_controller-srv:<EmotionalStateService-request> is deprecated: use cyborg_controller-srv:EmotionalStateService-request instead.")))

(cl:ensure-generic-function 'nothing-val :lambda-list '(m))
(cl:defmethod nothing-val ((m <EmotionalStateService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyborg_controller-srv:nothing-val is deprecated.  Use cyborg_controller-srv:nothing instead.")
  (nothing m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EmotionalStateService-request>) ostream)
  "Serializes a message object of type '<EmotionalStateService-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'nothing))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'nothing))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EmotionalStateService-request>) istream)
  "Deserializes a message object of type '<EmotionalStateService-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'nothing) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'nothing) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EmotionalStateService-request>)))
  "Returns string type for a service object of type '<EmotionalStateService-request>"
  "cyborg_controller/EmotionalStateServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EmotionalStateService-request)))
  "Returns string type for a service object of type 'EmotionalStateService-request"
  "cyborg_controller/EmotionalStateServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EmotionalStateService-request>)))
  "Returns md5sum for a message object of type '<EmotionalStateService-request>"
  "68da7087fb08817fd317a07916f3d49f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EmotionalStateService-request)))
  "Returns md5sum for a message object of type 'EmotionalStateService-request"
  "68da7087fb08817fd317a07916f3d49f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EmotionalStateService-request>)))
  "Returns full string definition for message of type '<EmotionalStateService-request>"
  (cl:format cl:nil "string nothing~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EmotionalStateService-request)))
  "Returns full string definition for message of type 'EmotionalStateService-request"
  (cl:format cl:nil "string nothing~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EmotionalStateService-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'nothing))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EmotionalStateService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'EmotionalStateService-request
    (cl:cons ':nothing (nothing msg))
))
;//! \htmlinclude EmotionalStateService-response.msg.html

(cl:defclass <EmotionalStateService-response> (roslisp-msg-protocol:ros-message)
  ((emotional_state
    :reader emotional_state
    :initarg :emotional_state
    :type cl:string
    :initform "")
   (current_pleasure
    :reader current_pleasure
    :initarg :current_pleasure
    :type cl:float
    :initform 0.0)
   (current_arousal
    :reader current_arousal
    :initarg :current_arousal
    :type cl:float
    :initform 0.0)
   (current_dominance
    :reader current_dominance
    :initarg :current_dominance
    :type cl:float
    :initform 0.0))
)

(cl:defclass EmotionalStateService-response (<EmotionalStateService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EmotionalStateService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EmotionalStateService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cyborg_controller-srv:<EmotionalStateService-response> is deprecated: use cyborg_controller-srv:EmotionalStateService-response instead.")))

(cl:ensure-generic-function 'emotional_state-val :lambda-list '(m))
(cl:defmethod emotional_state-val ((m <EmotionalStateService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyborg_controller-srv:emotional_state-val is deprecated.  Use cyborg_controller-srv:emotional_state instead.")
  (emotional_state m))

(cl:ensure-generic-function 'current_pleasure-val :lambda-list '(m))
(cl:defmethod current_pleasure-val ((m <EmotionalStateService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyborg_controller-srv:current_pleasure-val is deprecated.  Use cyborg_controller-srv:current_pleasure instead.")
  (current_pleasure m))

(cl:ensure-generic-function 'current_arousal-val :lambda-list '(m))
(cl:defmethod current_arousal-val ((m <EmotionalStateService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyborg_controller-srv:current_arousal-val is deprecated.  Use cyborg_controller-srv:current_arousal instead.")
  (current_arousal m))

(cl:ensure-generic-function 'current_dominance-val :lambda-list '(m))
(cl:defmethod current_dominance-val ((m <EmotionalStateService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyborg_controller-srv:current_dominance-val is deprecated.  Use cyborg_controller-srv:current_dominance instead.")
  (current_dominance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EmotionalStateService-response>) ostream)
  "Serializes a message object of type '<EmotionalStateService-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'emotional_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'emotional_state))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current_pleasure))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current_arousal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current_dominance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EmotionalStateService-response>) istream)
  "Deserializes a message object of type '<EmotionalStateService-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'emotional_state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'emotional_state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_pleasure) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_arousal) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_dominance) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EmotionalStateService-response>)))
  "Returns string type for a service object of type '<EmotionalStateService-response>"
  "cyborg_controller/EmotionalStateServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EmotionalStateService-response)))
  "Returns string type for a service object of type 'EmotionalStateService-response"
  "cyborg_controller/EmotionalStateServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EmotionalStateService-response>)))
  "Returns md5sum for a message object of type '<EmotionalStateService-response>"
  "68da7087fb08817fd317a07916f3d49f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EmotionalStateService-response)))
  "Returns md5sum for a message object of type 'EmotionalStateService-response"
  "68da7087fb08817fd317a07916f3d49f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EmotionalStateService-response>)))
  "Returns full string definition for message of type '<EmotionalStateService-response>"
  (cl:format cl:nil "string emotional_state~%float32 current_pleasure~%float32 current_arousal~%float32 current_dominance~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EmotionalStateService-response)))
  "Returns full string definition for message of type 'EmotionalStateService-response"
  (cl:format cl:nil "string emotional_state~%float32 current_pleasure~%float32 current_arousal~%float32 current_dominance~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EmotionalStateService-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'emotional_state))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EmotionalStateService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'EmotionalStateService-response
    (cl:cons ':emotional_state (emotional_state msg))
    (cl:cons ':current_pleasure (current_pleasure msg))
    (cl:cons ':current_arousal (current_arousal msg))
    (cl:cons ':current_dominance (current_dominance msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'EmotionalStateService)))
  'EmotionalStateService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'EmotionalStateService)))
  'EmotionalStateService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EmotionalStateService)))
  "Returns string type for a service object of type '<EmotionalStateService>"
  "cyborg_controller/EmotionalStateService")