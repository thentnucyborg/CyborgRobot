;; Auto-generated. Do not edit!


(when (boundp 'cyborg_controller::SystemState)
  (if (not (find-package "CYBORG_CONTROLLER"))
    (make-package "CYBORG_CONTROLLER"))
  (shadow 'SystemState (find-package "CYBORG_CONTROLLER")))
(unless (find-package "CYBORG_CONTROLLER::SYSTEMSTATE")
  (make-package "CYBORG_CONTROLLER::SYSTEMSTATE"))

(in-package "ROS")
;;//! \htmlinclude SystemState.msg.html


(defclass cyborg_controller::SystemState
  :super ros::object
  :slots (_event _from_system_state _to_system_state ))

(defmethod cyborg_controller::SystemState
  (:init
   (&key
    ((:event __event) "")
    ((:from_system_state __from_system_state) "")
    ((:to_system_state __to_system_state) "")
    )
   (send-super :init)
   (setq _event (string __event))
   (setq _from_system_state (string __from_system_state))
   (setq _to_system_state (string __to_system_state))
   self)
  (:event
   (&optional __event)
   (if __event (setq _event __event)) _event)
  (:from_system_state
   (&optional __from_system_state)
   (if __from_system_state (setq _from_system_state __from_system_state)) _from_system_state)
  (:to_system_state
   (&optional __to_system_state)
   (if __to_system_state (setq _to_system_state __to_system_state)) _to_system_state)
  (:serialization-length
   ()
   (+
    ;; string _event
    4 (length _event)
    ;; string _from_system_state
    4 (length _from_system_state)
    ;; string _to_system_state
    4 (length _to_system_state)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _event
       (write-long (length _event) s) (princ _event s)
     ;; string _from_system_state
       (write-long (length _from_system_state) s) (princ _from_system_state s)
     ;; string _to_system_state
       (write-long (length _to_system_state) s) (princ _to_system_state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _event
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _event (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _from_system_state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _from_system_state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _to_system_state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _to_system_state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get cyborg_controller::SystemState :md5sum-) "33e4fca93c330d18c8a8d3b755c771c5")
(setf (get cyborg_controller::SystemState :datatype-) "cyborg_controller/SystemState")
(setf (get cyborg_controller::SystemState :definition-)
      "string event
string from_system_state
string to_system_state
")



(provide :cyborg_controller/SystemState "33e4fca93c330d18c8a8d3b755c771c5")


