; Auto-generated. Do not edit!


(cl:in-package roadmap_msgs-msg)


;//! \htmlinclude RoadPolylineArray.msg.html

(cl:defclass <RoadPolylineArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (road_polylines
    :reader road_polylines
    :initarg :road_polylines
    :type (cl:vector roadmap_msgs-msg:RoadPolyline)
   :initform (cl:make-array 0 :element-type 'roadmap_msgs-msg:RoadPolyline :initial-element (cl:make-instance 'roadmap_msgs-msg:RoadPolyline))))
)

(cl:defclass RoadPolylineArray (<RoadPolylineArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RoadPolylineArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RoadPolylineArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name roadmap_msgs-msg:<RoadPolylineArray> is deprecated: use roadmap_msgs-msg:RoadPolylineArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RoadPolylineArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roadmap_msgs-msg:header-val is deprecated.  Use roadmap_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'road_polylines-val :lambda-list '(m))
(cl:defmethod road_polylines-val ((m <RoadPolylineArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roadmap_msgs-msg:road_polylines-val is deprecated.  Use roadmap_msgs-msg:road_polylines instead.")
  (road_polylines m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RoadPolylineArray>) ostream)
  "Serializes a message object of type '<RoadPolylineArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'road_polylines))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'road_polylines))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RoadPolylineArray>) istream)
  "Deserializes a message object of type '<RoadPolylineArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'road_polylines) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'road_polylines)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'roadmap_msgs-msg:RoadPolyline))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RoadPolylineArray>)))
  "Returns string type for a message object of type '<RoadPolylineArray>"
  "roadmap_msgs/RoadPolylineArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RoadPolylineArray)))
  "Returns string type for a message object of type 'RoadPolylineArray"
  "roadmap_msgs/RoadPolylineArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RoadPolylineArray>)))
  "Returns md5sum for a message object of type '<RoadPolylineArray>"
  "4ef0cc5f23352e54e8abcc0a04846779")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RoadPolylineArray)))
  "Returns md5sum for a message object of type 'RoadPolylineArray"
  "4ef0cc5f23352e54e8abcc0a04846779")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RoadPolylineArray>)))
  "Returns full string definition for message of type '<RoadPolylineArray>"
  (cl:format cl:nil "Header header~%~%roadmap_msgs/RoadPolyline[] road_polylines~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: roadmap_msgs/RoadPolyline~%# Road line identifier~%int32 id~%~%# Type of road line ~%uint8 LANECENTER_FREEWAY=1~%uint8 LANECENTER_SURFACESTREET=2~%uint8 LANECENTER_BIKELANE=3~%uint8 ROADLINE_BROKENSINGLEWHITE=6~%uint8 ROADLINE_SOLIDSINGLEWHITE=7~%uint8 ROADLINE_SOLIDDOUBLEWHITE=8~%uint8 ROADLINE_BROKENSINGLEYELLOW=9~%uint8 ROADLINE_BROKENDOUBLEYELLOW=10~%uint8 ROADLINE_SOLIDSINGLEYELLOW=11~%uint8 ROADLINE_SOLIDDOUBLEYELLOW=12~%uint8 ROADLINE_PASSINGDOUBLEYELLOW=13~%uint8 ROADEDGEBOUNDARY=15~%uint8 ROADEDGEMEDIAN=16~%uint8 STOPSIGN=17~%uint8 CROSSWALK=18~%uint8 SPEEDBUMP=19~%~%uint8 type~%~%# Polyline coordinates~%geometry_msgs/Point[] coords~%~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RoadPolylineArray)))
  "Returns full string definition for message of type 'RoadPolylineArray"
  (cl:format cl:nil "Header header~%~%roadmap_msgs/RoadPolyline[] road_polylines~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: roadmap_msgs/RoadPolyline~%# Road line identifier~%int32 id~%~%# Type of road line ~%uint8 LANECENTER_FREEWAY=1~%uint8 LANECENTER_SURFACESTREET=2~%uint8 LANECENTER_BIKELANE=3~%uint8 ROADLINE_BROKENSINGLEWHITE=6~%uint8 ROADLINE_SOLIDSINGLEWHITE=7~%uint8 ROADLINE_SOLIDDOUBLEWHITE=8~%uint8 ROADLINE_BROKENSINGLEYELLOW=9~%uint8 ROADLINE_BROKENDOUBLEYELLOW=10~%uint8 ROADLINE_SOLIDSINGLEYELLOW=11~%uint8 ROADLINE_SOLIDDOUBLEYELLOW=12~%uint8 ROADLINE_PASSINGDOUBLEYELLOW=13~%uint8 ROADEDGEBOUNDARY=15~%uint8 ROADEDGEMEDIAN=16~%uint8 STOPSIGN=17~%uint8 CROSSWALK=18~%uint8 SPEEDBUMP=19~%~%uint8 type~%~%# Polyline coordinates~%geometry_msgs/Point[] coords~%~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RoadPolylineArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'road_polylines) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RoadPolylineArray>))
  "Converts a ROS message object to a list"
  (cl:list 'RoadPolylineArray
    (cl:cons ':header (header msg))
    (cl:cons ':road_polylines (road_polylines msg))
))
