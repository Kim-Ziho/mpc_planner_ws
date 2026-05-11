#include <mpc_planner_rosnavigation/mpc_controller_plugin.h>

#include <mpc_planner_util/parameters.h>
#include <mpc_planner_util/load_yaml.hpp>

#include <ros_tools/logging.h>
#include <ros_tools/visuals.h>
#include <ros_tools/convertions.h>
#include <ros_tools/profiling.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>

using namespace MPCPlanner;

namespace local_planner
{

    void MPCController::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        _parent = parent;
        _plugin_name = name;
        _tf = tf;
        _costmap_ros = costmap_ros;

        // Stand up the ros_tools companion node first so any LOG_* / VISUALS
        // call inside MPCCore::configure has a valid singleton target.
        _companion_node = std::make_shared<rclcpp::Node>(
            "mpc_controller_companion_" + name);
        STATIC_NODE_POINTER.init(_companion_node.get());
        VISUALS.init(_companion_node.get());

        const std::string pkg_share =
            ament_index_cpp::get_package_share_directory("mpc_planner_rosnavigation");
        const std::string settings_path = pkg_share + "/config/settings.yaml";

        _core = std::make_unique<MPCCore>();
        _core->configure(settings_path);

        // Use the local_costmap that controller_server owns -- no own costmap.
        if (_costmap_ros)
            _core->setCostmap(_costmap_ros->getCostmap());

        // Obstacle subscription on parent LifecycleNode so it lives in the
        // controller_server executor.
        if (auto node = _parent.lock())
        {
            _obs_sub = node->create_subscription<mpc_planner_msgs::msg::ObstacleArray>(
                "/input/obstacles", rclcpp::QoS(1),
                std::bind(&MPCController::obstacleCallback, this, std::placeholders::_1));
            RCLCPP_INFO(node->get_logger(),
                        "MPCController '%s' configured", name.c_str());
        }
    }

    void MPCController::cleanup()
    {
        _obs_sub.reset();
        _core.reset();
        _costmap_ros.reset();
        _tf.reset();
        _companion_node.reset();
    }

    void MPCController::activate()
    {
        if (_core)
            _core->setEnableOutput(CONFIG["enable_output"].as<bool>());
    }

    void MPCController::deactivate()
    {
        if (_core)
            _core->setEnableOutput(false);
    }

    void MPCController::setPlan(const nav_msgs::msg::Path &path)
    {
        if (!_core)
            return;

        _core->setReferencePath(path);

        if (path.poses.empty())
            return;

        const auto &last = path.poses.back().pose.position;
        _core->setGoal(last.x, last.y);

        // Only request rotation when the goal actually changes. bt_navigator
        // re-sends setPlan() on every replan; an unconditional requestRotation
        // makes computeVelocityCommands return v=0 forever.
        constexpr double kGoalChangeSqr = 0.25 * 0.25; // 0.25 m
        const double dx = last.x - _last_goal_x;
        const double dy = last.y - _last_goal_y;
        if (!_has_last_goal || (dx * dx + dy * dy) > kGoalChangeSqr)
        {
            _core->requestRotation();
            _has_last_goal = true;
            _last_goal_x = last.x;
            _last_goal_y = last.y;
        }
    }

    void MPCController::obstacleCallback(
        mpc_planner_msgs::msg::ObstacleArray::ConstSharedPtr msg)
    {
        if (_core)
            _core->setObstacles(*msg);
    }

    geometry_msgs::msg::TwistStamped MPCController::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker * /*goal_checker*/)
    {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header = pose.header;

        if (!_core)
            return cmd;

        // Refresh state from the controller_server-provided pose/velocity.
        State s = _core->state();
        s.set("x", pose.pose.position.x);
        s.set("y", pose.pose.position.y);
        s.set("psi", RosTools::quaternionToAngle(pose.pose.orientation));
        s.set("v", std::sqrt(std::pow(velocity.linear.x, 2.) +
                             std::pow(velocity.linear.y, 2.)));
        _core->setState(s);

        // Re-bind the costmap each tick: controller_server can rebuild the
        // Costmap2D under us, leaving stale raw pointers behind.
        if (_costmap_ros)
            _core->setCostmap(_costmap_ros->getCostmap());

        MPCCommand out;
        if (_core->isRotating())
            _core->rotateToGoal(out);
        else
            out = _core->solve();

        // Optional speed-limit clamp from setSpeedLimit().
        double v = out.v;
        if (_speed_limit > 0.0)
        {
            if (_speed_limit_percent)
            {
                const double max_v = CONFIG["max_v"]
                                         ? CONFIG["max_v"].as<double>()
                                         : 1.0;
                v = std::min(v, max_v * (_speed_limit / 100.0));
            }
            else
            {
                v = std::min(v, _speed_limit);
            }
        }
        cmd.twist.linear.x = v;
        cmd.twist.angular.z = out.w;
        return cmd;
    }

    void MPCController::setSpeedLimit(
        const double &speed_limit, const bool &percentage)
    {
        _speed_limit = speed_limit;
        _speed_limit_percent = percentage;
    }

} // namespace local_planner

PLUGINLIB_EXPORT_CLASS(local_planner::MPCController, nav2_core::Controller)
