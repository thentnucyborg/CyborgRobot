; Auto-generated. Do not edit!


(cl:in-package cyborg_controller-msg)


;//! \htmlinclude StateMachineGoal.msg.html

(cl:defclass <StateMachineGoal> (roslisp-msg-protocol:ros-message)
  ((previous_state
    :reader previous_state
    :initarg :previous_state
    :type cl:string
    :initform "")
   (event
    :reader event
    :initarg :event
    :type cl:string
    :initform "")
   (current_state
    :reader current_state
    :initarg :current_state
    :type cl:string
    :initform "")
   (order
    :reader order
    :initarg :order
    :type cl:string
    :initform ""))
)

(cl:defclass StateMachineGoal (<StateMachineGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StateMachineGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StateMachineGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cyborg_controller-msg:<StateMachineGoal> is deprecated: use cyborg_controller-msg:StateMachineGoal instead.")))

(cl:ensure-generic-function 'previous_state-val :lambda-list '(m))
(cl:defmethod previous_state-val ((m <StateMachineGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyborg_controller-msg:previous_state-val is deprecated.  Use cyborg_controller-msg:previous_state instead.")
  (previous_state m))

(cl:ensure-generic-function 'event-val :lambda-list '(m))
(cl:defmethod event-val ((m <StateMachineGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyborg_controller-msg:event-val is deprecated.  Use cyborg_controller-msg:event instead.")
  (event m))

(cl:ensure-generic-function 'current_state-val :lambda-list '(m))
(cl:defmethod current_state-val ((m <StateMachineGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyborg_controller-msg:current_state-val is deprecated.  Use cyborg_controller-msg:current_state instead.")
  (current_state m))

(cl:ensure-generic-function 'order-val :lambda-list '(m))
(cl:defmethod order-val ((m <StateMachineGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyborg_controller-msg:order-val is deprecated.  Use cyborg_controller-msg:order instead.")
  (order m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StateMachineGoal>) ostream)
  "Serializes a message object of type '<StateMachineGoal>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'previous_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'previous_state))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'event))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'event))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'current_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'current_state))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'order))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'order))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StateMachineGoal>) istream)
  "Deserializes a message object of type '<StateMachineGoal>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'previous_state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'previous_state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'event) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'event) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'current_state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'order) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'order) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StateMachineGoal>)))
  "Returns string type for a message object of type '<StateMachineGoal>"
  "cyborg_controller/StateMachineGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StateMachineGoal)))
  "Returns string type for a message object of type 'StateMachineGoal"
  "cyborg_controller/StateMachineGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StateMachineGoal>)))
  "Returns md5sum for a message object of type '<StateMachineGoal>"
  "ca0cbc617848fd96cc506786c47de76e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StateMachineGoal)))
  "Returns md5sum for a message object of type 'StateMachineGoal"
  "ca0cbc617848fd96cc506786c47de76e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StateMachineGoal>)))
  "Returns full string definition for message of type '<StateMachineGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%string previous_state~%string event~%string current_state~%string order #valid is EXECUTE or CANCEL~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StateMachineGoal)))
  "Returns full string definition for message of type 'StateMachineGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%string previous_state~%string event~%string current_state~%string order #valid is EXECUTE or CANCEL~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StateMachineGoal>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'previous_state))
     4 (cl:length (cl:slot-value msg 'event))
     4 (cl:length (cl:slot-value msg 'current_state))
     4 (cl:length (cl:slot-value msg 'order))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StateMachineGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'StateMachineGoal
    (cl:cons ':previous_state (previous_state msg))
    (cl:cons ':event (event msg))
    (cl:cons ':current_state (current_state msg))
    (cl:cons ':order (order msg))
))
