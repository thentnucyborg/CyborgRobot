; Auto-generated. Do not edit!


(cl:in-package cyborg_controller-msg)


;//! \htmlinclude EmotionalState.msg.html

(cl:defclass <EmotionalState> (roslisp-msg-protocol:ros-message)
  ((from_emotional_state
    :reader from_emotional_state
    :initarg :from_emotional_state
    :type cl:string
    :initform "")
   (to_emotional_state
    :reader to_emotional_state
    :initarg :to_emotional_state
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

(cl:defclass EmotionalState (<EmotionalState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EmotionalState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EmotionalState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cyborg_controller-msg:<EmotionalState> is deprecated: use cyborg_controller-msg:EmotionalState instead.")))

(cl:ensure-generic-function 'from_emotional_state-val :lambda-list '(m))
(cl:defmethod from_emotional_state-val ((m <EmotionalState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyborg_controller-msg:from_emotional_state-val is deprecated.  Use cyborg_controller-msg:from_emotional_state instead.")
  (from_emotional_state m))

(cl:ensure-generic-function 'to_emotional_state-val :lambda-list '(m))
(cl:defmethod to_emotional_state-val ((m <EmotionalState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyborg_controller-msg:to_emotional_state-val is deprecated.  Use cyborg_controller-msg:to_emotional_state instead.")
  (to_emotional_state m))

(cl:ensure-generic-function 'current_pleasure-val :lambda-list '(m))
(cl:defmethod current_pleasure-val ((m <EmotionalState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyborg_controller-msg:current_pleasure-val is deprecated.  Use cyborg_controller-msg:current_pleasure instead.")
  (current_pleasure m))

(cl:ensure-generic-function 'current_arousal-val :lambda-list '(m))
(cl:defmethod current_arousal-val ((m <EmotionalState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyborg_controller-msg:current_arousal-val is deprecated.  Use cyborg_controller-msg:current_arousal instead.")
  (current_arousal m))

(cl:ensure-generic-function 'current_dominance-val :lambda-list '(m))
(cl:defmethod current_dominance-val ((m <EmotionalState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyborg_controller-msg:current_dominance-val is deprecated.  Use cyborg_controller-msg:current_dominance instead.")
  (current_dominance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EmotionalState>) ostream)
  "Serializes a message object of type '<EmotionalState>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'from_emotional_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'from_emotional_state))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'to_emotional_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'to_emotional_state))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EmotionalState>) istream)
  "Deserializes a message object of type '<EmotionalState>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'from_emotional_state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'from_emotional_state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'to_emotional_state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'to_emotional_state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EmotionalState>)))
  "Returns string type for a message object of type '<EmotionalState>"
  "cyborg_controller/EmotionalState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EmotionalState)))
  "Returns string type for a message object of type 'EmotionalState"
  "cyborg_controller/EmotionalState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EmotionalState>)))
  "Returns md5sum for a message object of type '<EmotionalState>"
  "ff5f086e373f1e982e89fbc9298d99be")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EmotionalState)))
  "Returns md5sum for a message object of type 'EmotionalState"
  "ff5f086e373f1e982e89fbc9298d99be")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EmotionalState>)))
  "Returns full string definition for message of type '<EmotionalState>"
  (cl:format cl:nil "string from_emotional_state~%string to_emotional_state~%float32 current_pleasure~%float32 current_arousal~%float32 current_dominance~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EmotionalState)))
  "Returns full string definition for message of type 'EmotionalState"
  (cl:format cl:nil "string from_emotional_state~%string to_emotional_state~%float32 current_pleasure~%float32 current_arousal~%float32 current_dominance~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EmotionalState>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'from_emotional_state))
     4 (cl:length (cl:slot-value msg 'to_emotional_state))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EmotionalState>))
  "Converts a ROS message object to a list"
  (cl:list 'EmotionalState
    (cl:cons ':from_emotional_state (from_emotional_state msg))
    (cl:cons ':to_emotional_state (to_emotional_state msg))
    (cl:cons ':current_pleasure (current_pleasure msg))
    (cl:cons ':current_arousal (current_arousal msg))
    (cl:cons ':current_dominance (current_dominance msg))
))
