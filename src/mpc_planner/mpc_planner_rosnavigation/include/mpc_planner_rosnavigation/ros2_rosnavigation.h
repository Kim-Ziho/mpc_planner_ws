#ifndef __ROS2_ROSNAVIGATION_PLANNER_H__
#define __ROS2_ROSNAVIGATION_PLANNER_H__

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/empty.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <mpc_planner_msgs/msg/obstacle_array.hpp>

#include <mpc_planner_rosnavigation/rosnavigation_ros2_reconfigure.h>
#include <mpc_planner_solver/solver_interface.h>
#include <mpc_planner_types/realtime_data.h>

#include <ros_tools/profiling.h>

#include <memory>
#include <mutex>

#define CAMERA_BUFFER 10

namespace MPCPlanner
{
    class Planner;
}

namespace local_planner
{
    class JackalPlanner : public rclcpp::Node
    {
    public:
        JackalPlanner();
        ~JackalPlanner();

        void initialize();

        void loop();

    private:
        // Subscribers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _state_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _state_pose_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _goal_sub;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_sub;
        rclcpp::Subscription<mpc_planner_msgs::msg::ObstacleArray>::SharedPtr _obstacle_sub;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _collisions_sub;

        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pose_pub;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _reset_simulation_pub;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _ped_horizon_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _ped_integrator_step_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _ped_clock_frequency_pub;

        // Service clients
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _ped_start_client;
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _reset_simulation_client;

        // Timer
        rclcpp::TimerBase::SharedPtr _timer;

        // TF
        std::shared_ptr<tf2_ros::TransformBroadcaster> _camera_pub;
        rclcpp::Time _prev_camera_stamp;

        // State
        std::unique_ptr<MPCPlanner::Planner> _planner;
        std::unique_ptr<RosnavigationReconfigure> _reconfigure;
        MPCPlanner::RealTimeData _data;
        MPCPlanner::State _state;
        bool _enable_output{false};
        bool _rotate_to_goal{false};
        bool _done{false};
        std::mutex _reset_mutex;

        RosTools::Timer _timeout_timer;

        double _x_buffer[CAMERA_BUFFER];
        double _y_buffer[CAMERA_BUFFER];

        // Callbacks
        void stateCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
        void statePoseCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
        void goalCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
        void pathCallback(nav_msgs::msg::Path::ConstSharedPtr msg);
        void obstacleCallback(mpc_planner_msgs::msg::ObstacleArray::ConstSharedPtr msg);
        void collisionCallback(std_msgs::msg::Float64::ConstSharedPtr msg);

        // Helpers
        void initializeSubscribersAndPublishers();
        void startEnvironment();
        void rotateToGoal(geometry_msgs::msg::Twist &cmd_vel);
        void runMPC(geometry_msgs::msg::Twist &cmd_vel);
        bool isGoalReached();
        bool isPathTheSame(nav_msgs::msg::Path::ConstSharedPtr msg);
        void reset(bool success = true);
        void publishPose();
        void publishCamera();
        void visualize();
    };
} // namespace local_planner

#endif // __ROS2_ROSNAVIGATION_PLANNER_H__
