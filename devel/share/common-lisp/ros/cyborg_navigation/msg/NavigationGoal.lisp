; Auto-generated. Do not edit!


(cl:in-package cyborg_navigation-msg)


;//! \htmlinclude NavigationGoal.msg.html

(cl:defclass <NavigationGoal> (roslisp-msg-protocol:ros-message)
  ((order
    :reader order
    :initarg :order
    :type cl:string
    :initform "")
   (location_name
    :reader location_name
    :initarg :location_name
    :type cl:string
    :initform ""))
)

(cl:defclass NavigationGoal (<NavigationGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NavigationGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NavigationGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cyborg_navigation-msg:<NavigationGoal> is deprecated: use cyborg_navigation-msg:NavigationGoal instead.")))

(cl:ensure-generic-function 'order-val :lambda-list '(m))
(cl:defmethod order-val ((m <NavigationGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyborg_navigation-msg:order-val is deprecated.  Use cyborg_navigation-msg:order instead.")
  (order m))

(cl:ensure-generic-function 'location_name-val :lambda-list '(m))
(cl:defmethod location_name-val ((m <NavigationGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyborg_navigation-msg:location_name-val is deprecated.  Use cyborg_navigation-msg:location_name instead.")
  (location_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NavigationGoal>) ostream)
  "Serializes a message object of type '<NavigationGoal>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'order))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'order))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'location_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'location_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NavigationGoal>) istream)
  "Deserializes a message object of type '<NavigationGoal>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'order) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'order) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'location_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'location_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NavigationGoal>)))
  "Returns string type for a message object of type '<NavigationGoal>"
  "cyborg_navigation/NavigationGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NavigationGoal)))
  "Returns string type for a message object of type 'NavigationGoal"
  "cyborg_navigation/NavigationGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NavigationGoal>)))
  "Returns md5sum for a message object of type '<NavigationGoal>"
  "76b28b03838801536ea3c48deea22ff4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NavigationGoal)))
  "Returns md5sum for a message object of type 'NavigationGoal"
  "76b28b03838801536ea3c48deea22ff4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NavigationGoal>)))
  "Returns full string definition for message of type '<NavigationGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%string order~%string location_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NavigationGoal)))
  "Returns full string definition for message of type 'NavigationGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%string order~%string location_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NavigationGoal>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'order))
     4 (cl:length (cl:slot-value msg 'location_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NavigationGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'NavigationGoal
    (cl:cons ':order (order msg))
    (cl:cons ':location_name (location_name msg))
))
