;; Auto-generated. Do not edit!


(when (boundp 'cyborg_controller::EmotionalState)
  (if (not (find-package "CYBORG_CONTROLLER"))
    (make-package "CYBORG_CONTROLLER"))
  (shadow 'EmotionalState (find-package "CYBORG_CONTROLLER")))
(unless (find-package "CYBORG_CONTROLLER::EMOTIONALSTATE")
  (make-package "CYBORG_CONTROLLER::EMOTIONALSTATE"))

(in-package "ROS")
;;//! \htmlinclude EmotionalState.msg.html


(defclass cyborg_controller::EmotionalState
  :super ros::object
  :slots (_from_emotional_state _to_emotional_state _current_pleasure _current_arousal _current_dominance ))

(defmethod cyborg_controller::EmotionalState
  (:init
   (&key
    ((:from_emotional_state __from_emotional_state) "")
    ((:to_emotional_state __to_emotional_state) "")
    ((:current_pleasure __current_pleasure) 0.0)
    ((:current_arousal __current_arousal) 0.0)
    ((:current_dominance __current_dominance) 0.0)
    )
   (send-super :init)
   (setq _from_emotional_state (string __from_emotional_state))
   (setq _to_emotional_state (string __to_emotional_state))
   (setq _current_pleasure (float __current_pleasure))
   (setq _current_arousal (float __current_arousal))
   (setq _current_dominance (float __current_dominance))
   self)
  (:from_emotional_state
   (&optional __from_emotional_state)
   (if __from_emotional_state (setq _from_emotional_state __from_emotional_state)) _from_emotional_state)
  (:to_emotional_state
   (&optional __to_emotional_state)
   (if __to_emotional_state (setq _to_emotional_state __to_emotional_state)) _to_emotional_state)
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
    ;; string _from_emotional_state
    4 (length _from_emotional_state)
    ;; string _to_emotional_state
    4 (length _to_emotional_state)
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
     ;; string _from_emotional_state
       (write-long (length _from_emotional_state) s) (princ _from_emotional_state s)
     ;; string _to_emotional_state
       (write-long (length _to_emotional_state) s) (princ _to_emotional_state s)
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
   ;; string _from_emotional_state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _from_emotional_state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _to_emotional_state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _to_emotional_state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _current_pleasure
     (setq _current_pleasure (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current_arousal
     (setq _current_arousal (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current_dominance
     (setq _current_dominance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get cyborg_controller::EmotionalState :md5sum-) "ff5f086e373f1e982e89fbc9298d99be")
(setf (get cyborg_controller::EmotionalState :datatype-) "cyborg_controller/EmotionalState")
(setf (get cyborg_controller::EmotionalState :definition-)
      "string from_emotional_state
string to_emotional_state
float32 current_pleasure
float32 current_arousal
float32 current_dominance
")



(provide :cyborg_controller/EmotionalState "ff5f086e373f1e982e89fbc9298d99be")


