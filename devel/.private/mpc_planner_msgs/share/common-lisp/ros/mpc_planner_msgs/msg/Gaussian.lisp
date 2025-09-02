; Auto-generated. Do not edit!


(cl:in-package mpc_planner_msgs-msg)


;//! \htmlinclude Gaussian.msg.html

(cl:defclass <Gaussian> (roslisp-msg-protocol:ros-message)
  ((mean
    :reader mean
    :initarg :mean
    :type nav_msgs-msg:Path
    :initform (cl:make-instance 'nav_msgs-msg:Path))
   (major_semiaxis
    :reader major_semiaxis
    :initarg :major_semiaxis
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (minor_semiaxis
    :reader minor_semiaxis
    :initarg :minor_semiaxis
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Gaussian (<Gaussian>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Gaussian>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Gaussian)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mpc_planner_msgs-msg:<Gaussian> is deprecated: use mpc_planner_msgs-msg:Gaussian instead.")))

(cl:ensure-generic-function 'mean-val :lambda-list '(m))
(cl:defmethod mean-val ((m <Gaussian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mpc_planner_msgs-msg:mean-val is deprecated.  Use mpc_planner_msgs-msg:mean instead.")
  (mean m))

(cl:ensure-generic-function 'major_semiaxis-val :lambda-list '(m))
(cl:defmethod major_semiaxis-val ((m <Gaussian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mpc_planner_msgs-msg:major_semiaxis-val is deprecated.  Use mpc_planner_msgs-msg:major_semiaxis instead.")
  (major_semiaxis m))

(cl:ensure-generic-function 'minor_semiaxis-val :lambda-list '(m))
(cl:defmethod minor_semiaxis-val ((m <Gaussian>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mpc_planner_msgs-msg:minor_semiaxis-val is deprecated.  Use mpc_planner_msgs-msg:minor_semiaxis instead.")
  (minor_semiaxis m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Gaussian>) ostream)
  "Serializes a message object of type '<Gaussian>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mean) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'major_semiaxis))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'major_semiaxis))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'minor_semiaxis))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'minor_semiaxis))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Gaussian>) istream)
  "Deserializes a message object of type '<Gaussian>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mean) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'major_semiaxis) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'major_semiaxis)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'minor_semiaxis) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'minor_semiaxis)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Gaussian>)))
  "Returns string type for a message object of type '<Gaussian>"
  "mpc_planner_msgs/Gaussian")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Gaussian)))
  "Returns string type for a message object of type 'Gaussian"
  "mpc_planner_msgs/Gaussian")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Gaussian>)))
  "Returns md5sum for a message object of type '<Gaussian>"
  "850460d51db9d70a66e94a860b9ab01d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Gaussian)))
  "Returns md5sum for a message object of type 'Gaussian"
  "850460d51db9d70a66e94a860b9ab01d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Gaussian>)))
  "Returns full string definition for message of type '<Gaussian>"
  (cl:format cl:nil "# Trajectory of the mean prediction~%nav_msgs/Path mean~%~%# Covariances decomposed into their major and minor axes~%float64[] major_semiaxis~%float64[] minor_semiaxis~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Gaussian)))
  "Returns full string definition for message of type 'Gaussian"
  (cl:format cl:nil "# Trajectory of the mean prediction~%nav_msgs/Path mean~%~%# Covariances decomposed into their major and minor axes~%float64[] major_semiaxis~%float64[] minor_semiaxis~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Gaussian>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mean))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'major_semiaxis) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'minor_semiaxis) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Gaussian>))
  "Converts a ROS message object to a list"
  (cl:list 'Gaussian
    (cl:cons ':mean (mean msg))
    (cl:cons ':major_semiaxis (major_semiaxis msg))
    (cl:cons ':minor_semiaxis (minor_semiaxis msg))
))
