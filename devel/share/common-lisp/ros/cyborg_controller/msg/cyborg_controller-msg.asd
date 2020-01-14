
(cl:in-package :asdf)

(defsystem "cyborg_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "EmotionalFeedback" :depends-on ("_package_EmotionalFeedback"))
    (:file "_package_EmotionalFeedback" :depends-on ("_package"))
    (:file "EmotionalState" :depends-on ("_package_EmotionalState"))
    (:file "_package_EmotionalState" :depends-on ("_package"))
    (:file "StateMachineAction" :depends-on ("_package_StateMachineAction"))
    (:file "_package_StateMachineAction" :depends-on ("_package"))
    (:file "StateMachineActionFeedback" :depends-on ("_package_StateMachineActionFeedback"))
    (:file "_package_StateMachineActionFeedback" :depends-on ("_package"))
    (:file "StateMachineActionGoal" :depends-on ("_package_StateMachineActionGoal"))
    (:file "_package_StateMachineActionGoal" :depends-on ("_package"))
    (:file "StateMachineActionResult" :depends-on ("_package_StateMachineActionResult"))
    (:file "_package_StateMachineActionResult" :depends-on ("_package"))
    (:file "StateMachineFeedback" :depends-on ("_package_StateMachineFeedback"))
    (:file "_package_StateMachineFeedback" :depends-on ("_package"))
    (:file "StateMachineGoal" :depends-on ("_package_StateMachineGoal"))
    (:file "_package_StateMachineGoal" :depends-on ("_package"))
    (:file "StateMachineResult" :depends-on ("_package_StateMachineResult"))
    (:file "_package_StateMachineResult" :depends-on ("_package"))
    (:file "SystemState" :depends-on ("_package_SystemState"))
    (:file "_package_SystemState" :depends-on ("_package"))
  ))