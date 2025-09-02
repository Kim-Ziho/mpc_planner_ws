; Auto-generated. Do not edit!


(cl:in-package mpc_planner_msgs-msg)


;//! \htmlinclude ObstacleGMM.msg.html

(cl:defclass <ObstacleGMM> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (gaussians
    :reader gaussians
    :initarg :gaussians
    :type (cl:vector mpc_planner_msgs-msg:Gaussian)
   :initform (cl:make-array 0 :element-type 'mpc_planner_msgs-msg:Gaussian :initial-element (cl:make-instance 'mpc_planner_msgs-msg:Gaussian)))
   (probabilities
    :reader probabilities
    :initarg :probabilities
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ObstacleGMM (<ObstacleGMM>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObstacleGMM>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObstacleGMM)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mpc_planner_msgs-msg:<ObstacleGMM> is deprecated: use mpc_planner_msgs-msg:ObstacleGMM instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <ObstacleGMM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mpc_planner_msgs-msg:id-val is deprecated.  Use mpc_planner_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <ObstacleGMM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mpc_planner_msgs-msg:pose-val is deprecated.  Use mpc_planner_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'gaussians-val :lambda-list '(m))
(cl:defmethod gaussians-val ((m <ObstacleGMM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mpc_planner_msgs-msg:gaussians-val is deprecated.  Use mpc_planner_msgs-msg:gaussians instead.")
  (gaussians m))

(cl:ensure-generic-function 'probabilities-val :lambda-list '(m))
(cl:defmethod probabilities-val ((m <ObstacleGMM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mpc_planner_msgs-msg:probabilities-val is deprecated.  Use mpc_planner_msgs-msg:probabilities instead.")
  (probabilities m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObstacleGMM>) ostream)
  "Serializes a message object of type '<ObstacleGMM>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'gaussians))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'gaussians))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'probabilities))))
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
   (cl:slot-value msg 'probabilities))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObstacleGMM>) istream)
  "Deserializes a message object of type '<ObstacleGMM>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'gaussians) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'gaussians)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'mpc_planner_msgs-msg:Gaussian))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'probabilities) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'probabilities)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObstacleGMM>)))
  "Returns string type for a message object of type '<ObstacleGMM>"
  "mpc_planner_msgs/ObstacleGMM")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObstacleGMM)))
  "Returns string type for a message object of type 'ObstacleGMM"
  "mpc_planner_msgs/ObstacleGMM")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObstacleGMM>)))
  "Returns md5sum for a message object of type '<ObstacleGMM>"
  "43fafe3d1d2bd26d6dea45245b7944b1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObstacleGMM)))
  "Returns md5sum for a message object of type 'ObstacleGMM"
  "43fafe3d1d2bd26d6dea45245b7944b1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObstacleGMM>)))
  "Returns full string definition for message of type '<ObstacleGMM>"
  (cl:format cl:nil "# ID for obstacle association~%int32 id~%~%# Current pose of the obstacle~%geometry_msgs/Pose pose~%~%# List of Gaussians and their probabilities~%mpc_planner_msgs/Gaussian[] gaussians~%float64[] probabilities~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: mpc_planner_msgs/Gaussian~%# Trajectory of the mean prediction~%nav_msgs/Path mean~%~%# Covariances decomposed into their major and minor axes~%float64[] major_semiaxis~%float64[] minor_semiaxis~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObstacleGMM)))
  "Returns full string definition for message of type 'ObstacleGMM"
  (cl:format cl:nil "# ID for obstacle association~%int32 id~%~%# Current pose of the obstacle~%geometry_msgs/Pose pose~%~%# List of Gaussians and their probabilities~%mpc_planner_msgs/Gaussian[] gaussians~%float64[] probabilities~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: mpc_planner_msgs/Gaussian~%# Trajectory of the mean prediction~%nav_msgs/Path mean~%~%# Covariances decomposed into their major and minor axes~%float64[] major_semiaxis~%float64[] minor_semiaxis~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObstacleGMM>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'gaussians) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'probabilities) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObstacleGMM>))
  "Converts a ROS message object to a list"
  (cl:list 'ObstacleGMM
    (cl:cons ':id (id msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':gaussians (gaussians msg))
    (cl:cons ':probabilities (probabilities msg))
))
