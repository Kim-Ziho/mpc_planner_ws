; Auto-generated. Do not edit!


(cl:in-package roadmap_msgs-msg)


;//! \htmlinclude RoadPolyline.msg.html

(cl:defclass <RoadPolyline> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (coords
    :reader coords
    :initarg :coords
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass RoadPolyline (<RoadPolyline>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RoadPolyline>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RoadPolyline)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name roadmap_msgs-msg:<RoadPolyline> is deprecated: use roadmap_msgs-msg:RoadPolyline instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <RoadPolyline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roadmap_msgs-msg:id-val is deprecated.  Use roadmap_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <RoadPolyline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roadmap_msgs-msg:type-val is deprecated.  Use roadmap_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'coords-val :lambda-list '(m))
(cl:defmethod coords-val ((m <RoadPolyline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader roadmap_msgs-msg:coords-val is deprecated.  Use roadmap_msgs-msg:coords instead.")
  (coords m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<RoadPolyline>)))
    "Constants for message type '<RoadPolyline>"
  '((:LANECENTER_FREEWAY . 1)
    (:LANECENTER_SURFACESTREET . 2)
    (:LANECENTER_BIKELANE . 3)
    (:ROADLINE_BROKENSINGLEWHITE . 6)
    (:ROADLINE_SOLIDSINGLEWHITE . 7)
    (:ROADLINE_SOLIDDOUBLEWHITE . 8)
    (:ROADLINE_BROKENSINGLEYELLOW . 9)
    (:ROADLINE_BROKENDOUBLEYELLOW . 10)
    (:ROADLINE_SOLIDSINGLEYELLOW . 11)
    (:ROADLINE_SOLIDDOUBLEYELLOW . 12)
    (:ROADLINE_PASSINGDOUBLEYELLOW . 13)
    (:ROADEDGEBOUNDARY . 15)
    (:ROADEDGEMEDIAN . 16)
    (:STOPSIGN . 17)
    (:CROSSWALK . 18)
    (:SPEEDBUMP . 19))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'RoadPolyline)))
    "Constants for message type 'RoadPolyline"
  '((:LANECENTER_FREEWAY . 1)
    (:LANECENTER_SURFACESTREET . 2)
    (:LANECENTER_BIKELANE . 3)
    (:ROADLINE_BROKENSINGLEWHITE . 6)
    (:ROADLINE_SOLIDSINGLEWHITE . 7)
    (:ROADLINE_SOLIDDOUBLEWHITE . 8)
    (:ROADLINE_BROKENSINGLEYELLOW . 9)
    (:ROADLINE_BROKENDOUBLEYELLOW . 10)
    (:ROADLINE_SOLIDSINGLEYELLOW . 11)
    (:ROADLINE_SOLIDDOUBLEYELLOW . 12)
    (:ROADLINE_PASSINGDOUBLEYELLOW . 13)
    (:ROADEDGEBOUNDARY . 15)
    (:ROADEDGEMEDIAN . 16)
    (:STOPSIGN . 17)
    (:CROSSWALK . 18)
    (:SPEEDBUMP . 19))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RoadPolyline>) ostream)
  "Serializes a message object of type '<RoadPolyline>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'coords))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'coords))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RoadPolyline>) istream)
  "Deserializes a message object of type '<RoadPolyline>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'coords) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'coords)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RoadPolyline>)))
  "Returns string type for a message object of type '<RoadPolyline>"
  "roadmap_msgs/RoadPolyline")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RoadPolyline)))
  "Returns string type for a message object of type 'RoadPolyline"
  "roadmap_msgs/RoadPolyline")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RoadPolyline>)))
  "Returns md5sum for a message object of type '<RoadPolyline>"
  "35f06eaee5fd980da3b6582143b5a629")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RoadPolyline)))
  "Returns md5sum for a message object of type 'RoadPolyline"
  "35f06eaee5fd980da3b6582143b5a629")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RoadPolyline>)))
  "Returns full string definition for message of type '<RoadPolyline>"
  (cl:format cl:nil "# Road line identifier~%int32 id~%~%# Type of road line ~%uint8 LANECENTER_FREEWAY=1~%uint8 LANECENTER_SURFACESTREET=2~%uint8 LANECENTER_BIKELANE=3~%uint8 ROADLINE_BROKENSINGLEWHITE=6~%uint8 ROADLINE_SOLIDSINGLEWHITE=7~%uint8 ROADLINE_SOLIDDOUBLEWHITE=8~%uint8 ROADLINE_BROKENSINGLEYELLOW=9~%uint8 ROADLINE_BROKENDOUBLEYELLOW=10~%uint8 ROADLINE_SOLIDSINGLEYELLOW=11~%uint8 ROADLINE_SOLIDDOUBLEYELLOW=12~%uint8 ROADLINE_PASSINGDOUBLEYELLOW=13~%uint8 ROADEDGEBOUNDARY=15~%uint8 ROADEDGEMEDIAN=16~%uint8 STOPSIGN=17~%uint8 CROSSWALK=18~%uint8 SPEEDBUMP=19~%~%uint8 type~%~%# Polyline coordinates~%geometry_msgs/Point[] coords~%~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RoadPolyline)))
  "Returns full string definition for message of type 'RoadPolyline"
  (cl:format cl:nil "# Road line identifier~%int32 id~%~%# Type of road line ~%uint8 LANECENTER_FREEWAY=1~%uint8 LANECENTER_SURFACESTREET=2~%uint8 LANECENTER_BIKELANE=3~%uint8 ROADLINE_BROKENSINGLEWHITE=6~%uint8 ROADLINE_SOLIDSINGLEWHITE=7~%uint8 ROADLINE_SOLIDDOUBLEWHITE=8~%uint8 ROADLINE_BROKENSINGLEYELLOW=9~%uint8 ROADLINE_BROKENDOUBLEYELLOW=10~%uint8 ROADLINE_SOLIDSINGLEYELLOW=11~%uint8 ROADLINE_SOLIDDOUBLEYELLOW=12~%uint8 ROADLINE_PASSINGDOUBLEYELLOW=13~%uint8 ROADEDGEBOUNDARY=15~%uint8 ROADEDGEMEDIAN=16~%uint8 STOPSIGN=17~%uint8 CROSSWALK=18~%uint8 SPEEDBUMP=19~%~%uint8 type~%~%# Polyline coordinates~%geometry_msgs/Point[] coords~%~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RoadPolyline>))
  (cl:+ 0
     4
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'coords) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RoadPolyline>))
  "Converts a ROS message object to a list"
  (cl:list 'RoadPolyline
    (cl:cons ':id (id msg))
    (cl:cons ':type (type msg))
    (cl:cons ':coords (coords msg))
))
