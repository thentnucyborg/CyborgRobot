
(cl:in-package :asdf)

(defsystem "cyborg_controller-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "EmotionalStateService" :depends-on ("_package_EmotionalStateService"))
    (:file "_package_EmotionalStateService" :depends-on ("_package"))
  ))