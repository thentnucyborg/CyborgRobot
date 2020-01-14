;; Auto-generated. Do not edit!


(when (boundp 'cyborg_controller::EmotionalFeedback)
  (if (not (find-package "CYBORG_CONTROLLER"))
    (make-package "CYBORG_CONTROLLER"))
  (shadow 'EmotionalFeedback (find-package "CYBORG_CONTROLLER")))
(unless (find-package "CYBORG_CONTROLLER::EMOTIONALFEEDBACK")
  (make-package "CYBORG_CONTROLLER::EMOTIONALFEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude EmotionalFeedback.msg.html


(defclass cyborg_controller::EmotionalFeedback
  :super ros::object
  :slots (_delta_pleasure _delta_arousal _delta_dominance ))

(defmethod cyborg_controller::EmotionalFeedback
  (:init
   (&key
    ((:delta_pleasure __delta_pleasure) 0.0)
    ((:delta_arousal __delta_arousal) 0.0)
    ((:delta_dominance __delta_dominance) 0.0)
    )
   (send-super :init)
   (setq _delta_pleasure (float __delta_pleasure))
   (setq _delta_arousal (float __delta_arousal))
   (setq _delta_dominance (float __delta_dominance))
   self)
  (:delta_pleasure
   (&optional __delta_pleasure)
   (if __delta_pleasure (setq _delta_pleasure __delta_pleasure)) _delta_pleasure)
  (:delta_arousal
   (&optional __delta_arousal)
   (if __delta_arousal (setq _delta_arousal __delta_arousal)) _delta_arousal)
  (:delta_dominance
   (&optional __delta_dominance)
   (if __delta_dominance (setq _delta_dominance __delta_dominance)) _delta_dominance)
  (:serialization-length
   ()
   (+
    ;; float32 _delta_pleasure
    4
    ;; float32 _delta_arousal
    4
    ;; float32 _delta_dominance
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _delta_pleasure
       (sys::poke _delta_pleasure (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _delta_arousal
       (sys::poke _delta_arousal (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _delta_dominance
       (sys::poke _delta_dominance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _delta_pleasure
     (setq _delta_pleasure (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _delta_arousal
     (setq _delta_arousal (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _delta_dominance
     (setq _delta_dominance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get cyborg_controller::EmotionalFeedback :md5sum-) "6f61bf537c02522f197246d02ded0f2c")
(setf (get cyborg_controller::EmotionalFeedback :datatype-) "cyborg_controller/EmotionalFeedback")
(setf (get cyborg_controller::EmotionalFeedback :definition-)
      "float32 delta_pleasure # Changes in pleasure
float32 delta_arousal # Changes in arousal
float32 delta_dominance #Changes in dominance
")



(provide :cyborg_controller/EmotionalFeedback "6f61bf537c02522f197246d02ded0f2c")


