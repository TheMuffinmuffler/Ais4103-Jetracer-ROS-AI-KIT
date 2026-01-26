
(cl:in-package :asdf)

(defsystem "ris_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Hello" :depends-on ("_package_Hello"))
    (:file "_package_Hello" :depends-on ("_package"))
  ))