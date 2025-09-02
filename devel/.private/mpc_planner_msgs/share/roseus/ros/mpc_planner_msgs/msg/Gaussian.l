;; Auto-generated. Do not edit!


(when (boundp 'mpc_planner_msgs::Gaussian)
  (if (not (find-package "MPC_PLANNER_MSGS"))
    (make-package "MPC_PLANNER_MSGS"))
  (shadow 'Gaussian (find-package "MPC_PLANNER_MSGS")))
(unless (find-package "MPC_PLANNER_MSGS::GAUSSIAN")
  (make-package "MPC_PLANNER_MSGS::GAUSSIAN"))

(in-package "ROS")
;;//! \htmlinclude Gaussian.msg.html
(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))


(defclass mpc_planner_msgs::Gaussian
  :super ros::object
  :slots (_mean _major_semiaxis _minor_semiaxis ))

(defmethod mpc_planner_msgs::Gaussian
  (:init
   (&key
    ((:mean __mean) (instance nav_msgs::Path :init))
    ((:major_semiaxis __major_semiaxis) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:minor_semiaxis __minor_semiaxis) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _mean __mean)
   (setq _major_semiaxis __major_semiaxis)
   (setq _minor_semiaxis __minor_semiaxis)
   self)
  (:mean
   (&rest __mean)
   (if (keywordp (car __mean))
       (send* _mean __mean)
     (progn
       (if __mean (setq _mean (car __mean)))
       _mean)))
  (:major_semiaxis
   (&optional __major_semiaxis)
   (if __major_semiaxis (setq _major_semiaxis __major_semiaxis)) _major_semiaxis)
  (:minor_semiaxis
   (&optional __minor_semiaxis)
   (if __minor_semiaxis (setq _minor_semiaxis __minor_semiaxis)) _minor_semiaxis)
  (:serialization-length
   ()
   (+
    ;; nav_msgs/Path _mean
    (send _mean :serialization-length)
    ;; float64[] _major_semiaxis
    (* 8    (length _major_semiaxis)) 4
    ;; float64[] _minor_semiaxis
    (* 8    (length _minor_semiaxis)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; nav_msgs/Path _mean
       (send _mean :serialize s)
     ;; float64[] _major_semiaxis
     (write-long (length _major_semiaxis) s)
     (dotimes (i (length _major_semiaxis))
       (sys::poke (elt _major_semiaxis i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _minor_semiaxis
     (write-long (length _minor_semiaxis) s)
     (dotimes (i (length _minor_semiaxis))
       (sys::poke (elt _minor_semiaxis i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; nav_msgs/Path _mean
     (send _mean :deserialize buf ptr-) (incf ptr- (send _mean :serialization-length))
   ;; float64[] _major_semiaxis
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _major_semiaxis (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _major_semiaxis i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _minor_semiaxis
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _minor_semiaxis (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _minor_semiaxis i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get mpc_planner_msgs::Gaussian :md5sum-) "850460d51db9d70a66e94a860b9ab01d")
(setf (get mpc_planner_msgs::Gaussian :datatype-) "mpc_planner_msgs/Gaussian")
(setf (get mpc_planner_msgs::Gaussian :definition-)
      "# Trajectory of the mean prediction
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

")



(provide :mpc_planner_msgs/Gaussian "850460d51db9d70a66e94a860b9ab01d")


