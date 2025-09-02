;; Auto-generated. Do not edit!


(when (boundp 'roadmap_msgs::RoadPolylineArray)
  (if (not (find-package "ROADMAP_MSGS"))
    (make-package "ROADMAP_MSGS"))
  (shadow 'RoadPolylineArray (find-package "ROADMAP_MSGS")))
(unless (find-package "ROADMAP_MSGS::ROADPOLYLINEARRAY")
  (make-package "ROADMAP_MSGS::ROADPOLYLINEARRAY"))

(in-package "ROS")
;;//! \htmlinclude RoadPolylineArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass roadmap_msgs::RoadPolylineArray
  :super ros::object
  :slots (_header _road_polylines ))

(defmethod roadmap_msgs::RoadPolylineArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:road_polylines __road_polylines) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _road_polylines __road_polylines)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:road_polylines
   (&rest __road_polylines)
   (if (keywordp (car __road_polylines))
       (send* _road_polylines __road_polylines)
     (progn
       (if __road_polylines (setq _road_polylines (car __road_polylines)))
       _road_polylines)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; roadmap_msgs/RoadPolyline[] _road_polylines
    (apply #'+ (send-all _road_polylines :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; roadmap_msgs/RoadPolyline[] _road_polylines
     (write-long (length _road_polylines) s)
     (dolist (elem _road_polylines)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; roadmap_msgs/RoadPolyline[] _road_polylines
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _road_polylines (let (r) (dotimes (i n) (push (instance roadmap_msgs::RoadPolyline :init) r)) r))
     (dolist (elem- _road_polylines)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get roadmap_msgs::RoadPolylineArray :md5sum-) "4ef0cc5f23352e54e8abcc0a04846779")
(setf (get roadmap_msgs::RoadPolylineArray :datatype-) "roadmap_msgs/RoadPolylineArray")
(setf (get roadmap_msgs::RoadPolylineArray :definition-)
      "Header header

roadmap_msgs/RoadPolyline[] road_polylines


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: roadmap_msgs/RoadPolyline
# Road line identifier
int32 id

# Type of road line 
uint8 LANECENTER_FREEWAY=1
uint8 LANECENTER_SURFACESTREET=2
uint8 LANECENTER_BIKELANE=3
uint8 ROADLINE_BROKENSINGLEWHITE=6
uint8 ROADLINE_SOLIDSINGLEWHITE=7
uint8 ROADLINE_SOLIDDOUBLEWHITE=8
uint8 ROADLINE_BROKENSINGLEYELLOW=9
uint8 ROADLINE_BROKENDOUBLEYELLOW=10
uint8 ROADLINE_SOLIDSINGLEYELLOW=11
uint8 ROADLINE_SOLIDDOUBLEYELLOW=12
uint8 ROADLINE_PASSINGDOUBLEYELLOW=13
uint8 ROADEDGEBOUNDARY=15
uint8 ROADEDGEMEDIAN=16
uint8 STOPSIGN=17
uint8 CROSSWALK=18
uint8 SPEEDBUMP=19

uint8 type

# Polyline coordinates
geometry_msgs/Point[] coords



================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :roadmap_msgs/RoadPolylineArray "4ef0cc5f23352e54e8abcc0a04846779")


