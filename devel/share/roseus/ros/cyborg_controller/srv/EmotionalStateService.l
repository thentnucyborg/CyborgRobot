;; Auto-generated. Do not edit!


(when (boundp 'cyborg_controller::EmotionalStateService)
  (if (not (find-package "CYBORG_CONTROLLER"))
    (make-package "CYBORG_CONTROLLER"))
  (shadow 'EmotionalStateService (find-package "CYBORG_CONTROLLER")))
(unless (find-package "CYBORG_CONTROLLER::EMOTIONALSTATESERVICE")
  (make-package "CYBORG_CONTROLLER::EMOTIONALSTATESERVICE"))
(unless (find-package "CYBORG_CONTROLLER::EMOTIONALSTATESERVICEREQUEST")
  (make-package "CYBORG_CONTROLLER::EMOTIONALSTATESERVICEREQUEST"))
(unless (find-package "CYBORG_CONTROLLER::EMOTIONALSTATESERVICERESPONSE")
  (make-package "CYBORG_CONTROLLER::EMOTIONALSTATESERVICERESPONSE"))

(in-package "ROS")





(defclass cyborg_controller::EmotionalStateServiceRequest
  :super ros::object
  :slots (_nothing ))

(defmethod cyborg_controller::EmotionalStateServiceRequest
  (:init
   (&key
    ((:nothing __nothing) "")
    )
   (send-super :init)
   (setq _nothing (string __nothing))
   self)
  (:nothing
   (&optional __nothing)
   (if __nothing (setq _nothing __nothing)) _nothing)
  (:serialization-length
   ()
   (+
    ;; string _nothing
    4 (length _nothing)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _nothing
       (write-long (length _nothing) s) (princ _nothing s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _nothing
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _nothing (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass cyborg_controller::EmotionalStateServiceResponse
  :super ros::object
  :slots (_emotional_state _current_pleasure _current_arousal _current_dominance ))

(defmethod cyborg_controller::EmotionalStateServiceResponse
  (:init
   (&key
    ((:emotional_state __emotional_state) "")
    ((:current_pleasure __current_pleasure) 0.0)
    ((:current_arousal __current_arousal) 0.0)
    ((:current_dominance __current_dominance) 0.0)
    )
   (send-super :init)
   (setq _emotional_state (string __emotional_state))
   (setq _current_pleasure (float __current_pleasure))
   (setq _current_arousal (float __current_arousal))
   (setq _current_dominance (float __current_dominance))
   self)
  (:emotional_state
   (&optional __emotional_state)
   (if __emotional_state (setq _emotional_state __emotional_state)) _emotional_state)
  (:current_pleasure
   (&optional __current_pleasure)
   (if __current_pleasure (setq _current_pleasure __current_pleasure)) _current_pleasure)
  (:current_arousal
   (&optional __current_arousal)
   (if __current_arousal (setq _current_arousal __current_arousal)) _current_arousal)
  (:current_dominance
   (&optional __current_dominance)
   (if __current_dominance (setq _current_dominance __current_dominance)) _current_dominance)
  (:serialization-length
   ()
   (+
    ;; string _emotional_state
    4 (length _emotional_state)
    ;; float32 _current_pleasure
    4
    ;; float32 _current_arousal
    4
    ;; float32 _current_dominance
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _emotional_state
       (write-long (length _emotional_state) s) (princ _emotional_state s)
     ;; float32 _current_pleasure
       (sys::poke _current_pleasure (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current_arousal
       (sys::poke _current_arousal (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current_dominance
       (sys::poke _current_dominance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _emotional_state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _emotional_state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _current_pleasure
     (setq _current_pleasure (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current_arousal
     (setq _current_arousal (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current_dominance
     (setq _current_dominance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass cyborg_controller::EmotionalStateService
  :super ros::object
  :slots ())

(setf (get cyborg_controller::EmotionalStateService :md5sum-) "68da7087fb08817fd317a07916f3d49f")
(setf (get cyborg_controller::EmotionalStateService :datatype-) "cyborg_controller/EmotionalStateService")
(setf (get cyborg_controller::EmotionalStateService :request) cyborg_controller::EmotionalStateServiceRequest)
(setf (get cyborg_controller::EmotionalStateService :response) cyborg_controller::EmotionalStateServiceResponse)

(defmethod cyborg_controller::EmotionalStateServiceRequest
  (:response () (instance cyborg_controller::EmotionalStateServiceResponse :init)))

(setf (get cyborg_controller::EmotionalStateServiceRequest :md5sum-) "68da7087fb08817fd317a07916f3d49f")
(setf (get cyborg_controller::EmotionalStateServiceRequest :datatype-) "cyborg_controller/EmotionalStateServiceRequest")
(setf (get cyborg_controller::EmotionalStateServiceRequest :definition-)
      "string nothing
---
string emotional_state
float32 current_pleasure
float32 current_arousal
float32 current_dominance

")

(setf (get cyborg_controller::EmotionalStateServiceResponse :md5sum-) "68da7087fb08817fd317a07916f3d49f")
(setf (get cyborg_controller::EmotionalStateServiceResponse :datatype-) "cyborg_controller/EmotionalStateServiceResponse")
(setf (get cyborg_controller::EmotionalStateServiceResponse :definition-)
      "string nothing
---
string emotional_state
float32 current_pleasure
float32 current_arousal
float32 current_dominance

")



(provide :cyborg_controller/EmotionalStateService "68da7087fb08817fd317a07916f3d49f")


