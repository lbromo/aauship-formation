
(cl:in-package :asdf)

(defsystem "aauship-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "testSetpoints" :depends-on ("_package_testSetpoints"))
    (:file "_package_testSetpoints" :depends-on ("_package"))
    (:file "GPS" :depends-on ("_package_GPS"))
    (:file "_package_GPS" :depends-on ("_package"))
    (:file "controlTest" :depends-on ("_package_controlTest"))
    (:file "_package_controlTest" :depends-on ("_package"))
    (:file "ADIS16405" :depends-on ("_package_ADIS16405"))
    (:file "_package_ADIS16405" :depends-on ("_package"))
    (:file "LLIinput" :depends-on ("_package_LLIinput"))
    (:file "_package_LLIinput" :depends-on ("_package"))
    (:file "PID" :depends-on ("_package_PID"))
    (:file "_package_PID" :depends-on ("_package"))
    (:file "Attitude" :depends-on ("_package_Attitude"))
    (:file "_package_Attitude" :depends-on ("_package"))
    (:file "BatteryMonitor" :depends-on ("_package_BatteryMonitor"))
    (:file "_package_BatteryMonitor" :depends-on ("_package"))
    (:file "Faps" :depends-on ("_package_Faps"))
    (:file "_package_Faps" :depends-on ("_package"))
  ))