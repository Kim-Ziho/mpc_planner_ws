;; Auto-generated. Do not edit!


(when (boundp 'mpc_planner_msgs::ObstacleGMM)
  (if (not (find-package "MPC_PLANNER_MSGS"))
    (make-package "MPC_PLANNER_MSGS"))
  (shadow 'ObstacleGMM (find-package "MPC_PLANNER_MSGS")))
(unless (find-package "MPC_PLANNER_MSGS::OBSTACLEGMM")
  (make-package "MPC_PLANNER_MSGS::OBSTACLEGMM"))

(in-package "ROS")
;;//! \htmlinclude ObstacleGMM.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass mpc_planner_msgs::ObstacleGMM
  :super ros::object
  :slots (_id _pose _gaussians _probabilities ))

(defmethod mpc_planner_msgs::ObstacleGMM
  (:init
   (&key
    ((:id __id) 0)
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:gaussians __gaussians) ())
    ((:probabilities __probabilities) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _pose __pose)
   (setq _gaussians __gaussians)
   (setq _probabilities __probabilities)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:gaussians
   (&rest __gaussians)
   (if (keywordp (car __gaussians))
       (send* _gaussians __gaussians)
     (progn
       (if __gaussians (setq _gaussians (car __gaussians)))
       _gaussians)))
  (:probabilities
   (&optional __probabilities)
   (if __probabilities (setq _probabilities __probabilities)) _probabilities)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; mpc_planner_msgs/Gaussian[] _gaussians
    (apply #'+ (send-all _gaussians :serialization-length)) 4
    ;; float64[] _probabilities
    (* 8    (length _probabilities)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; mpc_planner_msgs/Gaussian[] _gaussians
     (write-long (length _gaussians) s)
     (dolist (elem _gaussians)
       (send elem :serialize s)
       )
     ;; float64[] _probabilities
     (write-long (length _probabilities) s)
     (dotimes (i (length _probabilities))
       (sys::poke (elt _probabilities i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; mpc_planner_msgs/Gaussian[] _gaussians
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _gaussians (let (r) (dotimes (i n) (push (instance mpc_planner_msgs::Gaussian :init) r)) r))
     (dolist (elem- _gaussians)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64[] _probabilities
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _probabilities (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _probabilities i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get mpc_planner_msgs::ObstacleGMM :md5sum-) "43fafe3d1d2bd26d6dea45245b7944b1")
(setf (get mpc_planner_msgs::ObstacleGMM :datatype-) "mpc_planner_msgs/ObstacleGMM")
(setf (get mpc_planner_msgs::ObstacleGMM :definition-)
      "# ID for obstacle association
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
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

")



(provide :mpc_planner_msgs/ObstacleGMM "43fafe3d1d2bd26d6dea45245b7944b1")


