
(cl:in-package :asdf)

(defsystem "testDrive-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "carInfo" :depends-on ("_package_carInfo"))
    (:file "_package_carInfo" :depends-on ("_package"))
  ))