;; Auto-generated. Do not edit!


(when (boundp 'rosarnl::BatteryStatus)
  (if (not (find-package "ROSARNL"))
    (make-package "ROSARNL"))
  (shadow 'BatteryStatus (find-package "ROSARNL")))
(unless (find-package "ROSARNL::BATTERYSTATUS")
  (make-package "ROSARNL::BATTERYSTATUS"))

(in-package "ROS")
;;//! \htmlinclude BatteryStatus.msg.html


(intern "*CHARGING_UNKNOWN*" (find-package "ROSARNL::BATTERYSTATUS"))
(shadow '*CHARGING_UNKNOWN* (find-package "ROSARNL::BATTERYSTATUS"))
(defconstant rosarnl::BatteryStatus::*CHARGING_UNKNOWN* -1)
(intern "*CHARGING_NOT*" (find-package "ROSARNL::BATTERYSTATUS"))
(shadow '*CHARGING_NOT* (find-package "ROSARNL::BATTERYSTATUS"))
(defconstant rosarnl::BatteryStatus::*CHARGING_NOT* 0)
(intern "*CHARGING_BULK*" (find-package "ROSARNL::BATTERYSTATUS"))
(shadow '*CHARGING_BULK* (find-package "ROSARNL::BATTERYSTATUS"))
(defconstant rosarnl::BatteryStatus::*CHARGING_BULK* 1)
(intern "*CHARGING_OVERCHARGE*" (find-package "ROSARNL::BATTERYSTATUS"))
(shadow '*CHARGING_OVERCHARGE* (find-package "ROSARNL::BATTERYSTATUS"))
(defconstant rosarnl::BatteryStatus::*CHARGING_OVERCHARGE* 2)
(intern "*CHARGING_FLOAT*" (find-package "ROSARNL::BATTERYSTATUS"))
(shadow '*CHARGING_FLOAT* (find-package "ROSARNL::BATTERYSTATUS"))
(defconstant rosarnl::BatteryStatus::*CHARGING_FLOAT* 3)
(intern "*CHARGING_BALANCE*" (find-package "ROSARNL::BATTERYSTATUS"))
(shadow '*CHARGING_BALANCE* (find-package "ROSARNL::BATTERYSTATUS"))
(defconstant rosarnl::BatteryStatus::*CHARGING_BALANCE* 4)
(defclass rosarnl::BatteryStatus
  :super ros::object
  :slots (_charging_state _charge_percent ))

(defmethod rosarnl::BatteryStatus
  (:init
   (&key
    ((:charging_state __charging_state) 0)
    ((:charge_percent __charge_percent) 0.0)
    )
   (send-super :init)
   (setq _charging_state (round __charging_state))
   (setq _charge_percent (float __charge_percent))
   self)
  (:charging_state
   (&optional __charging_state)
   (if __charging_state (setq _charging_state __charging_state)) _charging_state)
  (:charge_percent
   (&optional __charge_percent)
   (if __charge_percent (setq _charge_percent __charge_percent)) _charge_percent)
  (:serialization-length
   ()
   (+
    ;; int8 _charging_state
    1
    ;; float32 _charge_percent
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _charging_state
       (write-byte _charging_state s)
     ;; float32 _charge_percent
       (sys::poke _charge_percent (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _charging_state
     (setq _charging_state (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _charging_state 127) (setq _charging_state (- _charging_state 256)))
   ;; float32 _charge_percent
     (setq _charge_percent (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get rosarnl::BatteryStatus :md5sum-) "a5ae24af1ef085b1c28fd0c2d4869c5f")
(setf (get rosarnl::BatteryStatus :datatype-) "rosarnl/BatteryStatus")
(setf (get rosarnl::BatteryStatus :definition-)
      "int8 CHARGING_UNKNOWN = -1
int8 CHARGING_NOT = 0
int8 CHARGING_BULK = 1
int8 CHARGING_OVERCHARGE = 2
int8 CHARGING_FLOAT = 3
int8 CHARGING_BALANCE = 4

int8 charging_state
float32 charge_percent

")



(provide :rosarnl/BatteryStatus "a5ae24af1ef085b1c28fd0c2d4869c5f")


