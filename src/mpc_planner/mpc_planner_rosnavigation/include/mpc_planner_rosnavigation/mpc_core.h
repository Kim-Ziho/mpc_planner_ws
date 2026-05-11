// MPC core extracted from JackalPlanner so the same logic can be used by both
// the standalone rclcpp::Node (ros2_rosnavigation.cpp) and the upcoming
// nav2_core::Controller plugin (see docs/nav2_full_plugin_migration_plan.md).
//
// MPCCore intentionally has no Node ownership: callers are responsible for
// initializing ros_tools singletons (STATIC_NODE_POINTER, VISUALS) before any
// LOG_*/visualize call lands inside MPCCore.

#ifndef __MPC_PLANNER_ROSNAVIGATION_MPC_CORE_H__
#define __MPC_PLANNER_ROSNAVIGATION_MPC_CORE_H__

#include <Eigen/Dense>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

#include <mpc_planner_msgs/msg/obstacle_array.hpp>

#include <mpc_planner_solver/state.h>
#include <mpc_planner_types/realtime_data.h>

#include <memory>
#include <string>

namespace nav2_costmap_2d
{
    class Costmap2D;
}

namespace MPCPlanner
{
    class Planner;
}

namespace local_planner
{
    struct MPCCommand
    {
        double v{0.0};
        double w{0.0};
        bool success{false};
    };

    class MPCCore
    {
    public:
        MPCCore();
        ~MPCCore();

        // One-shot setup: load settings.yaml, instantiate Planner, prime
        // robot_area. Must be called once before any other method.
        void configure(const std::string &settings_path);

        // Inputs
        void setState(const MPCPlanner::State &state);
        void setReferencePath(const nav_msgs::msg::Path &path);
        void setObstacles(const mpc_planner_msgs::msg::ObstacleArray &msg);
        void setGoal(double x, double y);
        void setCostmap(nav2_costmap_2d::Costmap2D *costmap);
        void setEnableOutput(bool en) { _enable_output = en; }

        // Solve a single MPC step and produce a (v, w) command. Caller wraps
        // into Twist/TwistStamped. On infeasible or disabled output, brakes.
        MPCCommand solve();

        // Pre-MPC orientation: align robot heading with reference path. Sets
        // cmd in-place and returns true while still rotating. When alignment
        // is within tolerance the internal flag is cleared.
        bool rotateToGoal(MPCCommand &cmd);

        // Triggered by goalCallback equivalents. Causes the next loop tick to
        // execute rotateToGoal until alignment is achieved.
        void requestRotation() { _rotate_to_goal = true; }
        bool isRotating() const { return _rotate_to_goal; }

        // True the moment all modules report objective reached. Latches via
        // _done so the caller can run any one-shot finalization safely.
        bool checkGoalReached();
        bool goalReceived() const { return _data.goal_received; }
        bool isDone() const { return _done; }

        // MPC-specific reset. Wipes Planner internal state, state, and data
        // (modulo robot_area). Re-binds the remembered costmap pointer.
        void reset(bool success = true);

        // Visualization: draw the current heading line ("angle" topic).
        void visualizeHeading();

        // Accessors for callers that still need direct access.
        const MPCPlanner::State &state() const { return _state; }
        MPCPlanner::State &state() { return _state; }
        const MPCPlanner::RealTimeData &data() const { return _data; }
        MPCPlanner::RealTimeData &data() { return _data; }
        MPCPlanner::Planner *planner() { return _planner.get(); }

    private:
        std::unique_ptr<MPCPlanner::Planner> _planner;
        MPCPlanner::RealTimeData _data;
        MPCPlanner::State _state;

        nav2_costmap_2d::Costmap2D *_costmap{nullptr};

        bool _enable_output{false};
        bool _rotate_to_goal{false};
        bool _done{false};
        bool _configured{false};

        bool isPathTheSame(const nav_msgs::msg::Path &msg) const;
    };
} // namespace local_planner

#endif // __MPC_PLANNER_ROSNAVIGATION_MPC_CORE_H__
