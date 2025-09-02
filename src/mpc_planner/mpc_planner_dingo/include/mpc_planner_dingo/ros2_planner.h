#ifndef dingo_PLANNER_H
#define dingo_PLANNER_H

#include <mpc_planner/planner.h>

#include <mpc_planner_solver/solver_interface.h>

#include <mpc_planner_types/realtime_data.h>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <ros_tools/helpers.h>

#include <memory>

using namespace MPCPlanner;

class dingoPlanner : public rclcpp::Node
{
public:
    dingoPlanner();

    void initializeSubscribersAndPublishers();

    void Loop();

    void stateCallback(nav_msgs::msg::Odometry::SharedPtr msg);
    void goalCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void pathCallback(nav_msgs::msg::Path::SharedPtr msg);

private:
    std::unique_ptr<Planner> _planner;

    RealTimeData _data;
    State _state;

    rclcpp::TimerBase::SharedPtr _timer;

    std::unique_ptr<RosTools::Benchmarker> _benchmarker;

    // Subscribers and publishers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _state_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _goal_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_sub;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_pub;

    bool isPathTheSame(nav_msgs::msg::Path::SharedPtr path);

    void visualize();
};

#endif // dingo_PLANNER_H
