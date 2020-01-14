
(cl:in-package :asdf)

(defsystem "rosarnl-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "BatteryStatus" :depends-on ("_package_BatteryStatus"))
    (:file "_package_BatteryStatus" :depends-on ("_package"))
    (:file "BumperState" :depends-on ("_package_BumperState"))
    (:file "_package_BumperState" :depends-on ("_package"))
    (:file "JogPositionAction" :depends-on ("_package_JogPositionAction"))
    (:file "_package_JogPositionAction" :depends-on ("_package"))
    (:file "JogPositionActionFeedback" :depends-on ("_package_JogPositionActionFeedback"))
    (:file "_package_JogPositionActionFeedback" :depends-on ("_package"))
    (:file "JogPositionActionGoal" :depends-on ("_package_JogPositionActionGoal"))
    (:file "_package_JogPositionActionGoal" :depends-on ("_package"))
    (:file "JogPositionActionResult" :depends-on ("_package_JogPositionActionResult"))
    (:file "_package_JogPositionActionResult" :depends-on ("_package"))
    (:file "JogPositionFeedback" :depends-on ("_package_JogPositionFeedback"))
    (:file "_package_JogPositionFeedback" :depends-on ("_package"))
    (:file "JogPositionGoal" :depends-on ("_package_JogPositionGoal"))
    (:file "_package_JogPositionGoal" :depends-on ("_package"))
    (:file "JogPositionResult" :depends-on ("_package_JogPositionResult"))
    (:file "_package_JogPositionResult" :depends-on ("_package"))
  ))