
(cl:in-package :asdf)

(defsystem "roadmap_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "RoadPolyline" :depends-on ("_package_RoadPolyline"))
    (:file "_package_RoadPolyline" :depends-on ("_package"))
    (:file "RoadPolylineArray" :depends-on ("_package_RoadPolylineArray"))
    (:file "_package_RoadPolylineArray" :depends-on ("_package"))
  ))