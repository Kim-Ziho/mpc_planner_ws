// MPC controller implementing the nav2_core::Controller interface, so the MPC
// can be loaded into controller_server alongside bt_navigator / planner_server
// / behavior_server (Phase 2 / Option A in
// docs/nav2_full_plugin_migration_plan.md).
//
// The actual MPC solve logic lives in MPCCore -- this class is purely the
// ROS2/Nav2 plumbing.

#ifndef __MPC_PLANNER_ROSNAVIGATION_MPC_CONTROLLER_PLUGIN_H__
#define __MPC_PLANNER_ROSNAVIGATION_MPC_CONTROLLER_PLUGIN_H__

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_core/goal_checker.hpp>

#include <tf2_ros/buffer.h>

#include <mpc_planner_msgs/msg/obstacle_array.hpp>

#include <mpc_planner_rosnavigation/mpc_core.h>

namespace local_planner
{
    class MPCController : public nav2_core::Controller
    {
    public:
        MPCController() = default;
        ~MPCController() override = default;

        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            std::string name,
            std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

        void cleanup() override;
        void activate() override;
        void deactivate() override;

        void setPlan(const nav_msgs::msg::Path &path) override;

        geometry_msgs::msg::TwistStamped computeVelocityCommands(
            const geometry_msgs::msg::PoseStamped &pose,
            const geometry_msgs::msg::Twist &velocity,
            nav2_core::GoalChecker *goal_checker) override;

        void setSpeedLimit(const double &speed_limit, const bool &percentage) override;

    private:
        // ros_tools' STATIC_NODE_POINTER / VISUALS singletons need a plain
        // rclcpp::Node*. controller_server's parent is a LifecycleNode which
        // is a different type, so the plugin spins up a private companion
        // node solely for logging + marker publishers. It is not added to
        // any executor (publishing does not require spinning).
        std::shared_ptr<rclcpp::Node> _companion_node;

        rclcpp_lifecycle::LifecycleNode::WeakPtr _parent;
        std::shared_ptr<tf2_ros::Buffer> _tf;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> _costmap_ros;
        std::string _plugin_name;

        std::unique_ptr<MPCCore> _core;

        // Obstacle ingestion: initial cut subscribes from inside the plugin
        // (M-D decision). Defer separating into MPCObstacleCollector unless
        // executor contention shows up.
        rclcpp::Subscription<mpc_planner_msgs::msg::ObstacleArray>::SharedPtr _obs_sub;

        double _speed_limit{0.0};
        bool _speed_limit_percent{false};

        void obstacleCallback(mpc_planner_msgs::msg::ObstacleArray::ConstSharedPtr msg);
    };
} // namespace local_planner

#endif // __MPC_PLANNER_ROSNAVIGATION_MPC_CONTROLLER_PLUGIN_H__
