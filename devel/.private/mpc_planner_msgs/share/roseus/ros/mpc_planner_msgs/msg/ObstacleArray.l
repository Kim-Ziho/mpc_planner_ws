;; Auto-generated. Do not edit!


(when (boundp 'mpc_planner_msgs::ObstacleArray)
  (if (not (find-package "MPC_PLANNER_MSGS"))
    (make-package "MPC_PLANNER_MSGS"))
  (shadow 'ObstacleArray (find-package "MPC_PLANNER_MSGS")))
(unless (find-package "MPC_PLANNER_MSGS::OBSTACLEARRAY")
  (make-package "MPC_PLANNER_MSGS::OBSTACLEARRAY"))

(in-package "ROS")
;;//! \htmlinclude ObstacleArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass mpc_planner_msgs::ObstacleArray
  :super ros::object
  :slots (_header _obstacles ))

(defmethod mpc_planner_msgs::ObstacleArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:obstacles __obstacles) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _obstacles __obstacles)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:obstacles
   (&rest __obstacles)
   (if (keywordp (car __obstacles))
       (send* _obstacles __obstacles)
     (progn
       (if __obstacles (setq _obstacles (car __obstacles)))
       _obstacles)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; mpc_planner_msgs/ObstacleGMM[] _obstacles
    (apply #'+ (send-all _obstacles :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; mpc_planner_msgs/ObstacleGMM[] _obstacles
     (write-long (length _obstacles) s)
     (dolist (elem _obstacles)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; mpc_planner_msgs/ObstacleGMM[] _obstacles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacles (let (r) (dotimes (i n) (push (instance mpc_planner_msgs::ObstacleGMM :init) r)) r))
     (dolist (elem- _obstacles)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get mpc_planner_msgs::ObstacleArray :md5sum-) "54a5cf85a46c0ed6e1b76b6a339b56e3")
(setf (get mpc_planner_msgs::ObstacleArray :datatype-) "mpc_planner_msgs/ObstacleArray")
(setf (get mpc_planner_msgs::ObstacleArray :definition-)
      "std_msgs/Header header

mpc_planner_msgs/ObstacleGMM[] obstacles

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
MSG: mpc_planner_msgs/ObstacleGMM
# ID for obstacle association
int32 id

# Current pose of the obstacle
geometry_msgs/Pose pose

# List of Gaussians and their probabilities
mpc_planner_msgs/Gaussian[] gaussians
float64[] probabilities


================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: mpc_planner_msgs/Gaussian
# Trajectory of the mean prediction
nav_msgs/Path mean

# Covariances decomposed into their major and minor axes
float64[] major_semiaxis
float64[] minor_semiaxis
================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

")



(provide :mpc_planner_msgs/ObstacleArray "54a5cf85a46c0ed6e1b76b6a339b56e3")


