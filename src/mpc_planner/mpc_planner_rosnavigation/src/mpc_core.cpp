#include <mpc_planner_rosnavigation/mpc_core.h>

#include <mpc_planner/planner.h>
#include <mpc_planner/data_preparation.h>

#include <mpc_planner_util/parameters.h>
#include <mpc_planner_util/load_yaml.hpp>

#include <ros_tools/visuals.h>
#include <ros_tools/logging.h>
#include <ros_tools/convertions.h>
#include <ros_tools/math.h>
#include <ros_tools/data_saver.h>
#include <ros_tools/profiling.h>

#include <nav2_costmap_2d/costmap_2d.hpp>

#include <chrono>
#include <cmath>

using namespace MPCPlanner;

namespace local_planner
{
    MPCCore::MPCCore() = default;
    MPCCore::~MPCCore() = default;

    void MPCCore::configure(const std::string &settings_path)
    {
        Configuration::getInstance().initialize(settings_path);

        _data.robot_area = {Disc(0., CONFIG["robot_radius"].as<double>())};

        _planner = std::make_unique<Planner>();
        _enable_output = CONFIG["enable_output"].as<bool>();
        _configured = true;
    }

    void MPCCore::setState(const MPCPlanner::State &state)
    {
        _state = state;
    }

    bool MPCCore::isPathTheSame(const nav_msgs::msg::Path &msg) const
    {
        if (_data.reference_path.x.size() != msg.poses.size())
            return false;

        const int num_points = std::min(2, (int)_data.reference_path.x.size());
        for (int i = 0; i < num_points; i++)
        {
            if (!_data.reference_path.pointInPath(i,
                                                  msg.poses[i].pose.position.x,
                                                  msg.poses[i].pose.position.y))
                return false;
        }
        return true;
    }

    void MPCCore::setReferencePath(const nav_msgs::msg::Path &msg)
    {
        const int downsample = CONFIG["downsample_path"].as<double>();

        if (isPathTheSame(msg) || (int)msg.poses.size() < downsample + 1)
            return;

        _data.reference_path.clear();

        int count = 0;
        for (auto &pose : msg.poses)
        {
            if (count % downsample == 0 || count == (int)msg.poses.size() - 1)
            {
                _data.reference_path.x.push_back(pose.pose.position.x);
                _data.reference_path.y.push_back(pose.pose.position.y);
                _data.reference_path.psi.push_back(
                    RosTools::quaternionToAngle(pose.pose.orientation));
            }
            count++;
        }

        if (_planner)
            _planner->onDataReceived(_data, "reference_path");
    }

    void MPCCore::setObstacles(const mpc_planner_msgs::msg::ObstacleArray &msg)
    {
        _data.dynamic_obstacles.clear();

        for (auto &obstacle : msg.obstacles)
        {
            _data.dynamic_obstacles.emplace_back(
                obstacle.id,
                Eigen::Vector2d(obstacle.pose.position.x, obstacle.pose.position.y),
                RosTools::quaternionToAngle(obstacle.pose),
                CONFIG["obstacle_radius"].as<double>());
            auto &dynamic_obstacle = _data.dynamic_obstacles.back();

            if (obstacle.probabilities.size() == 0)
                continue;

            if (obstacle.probabilities.size() == 1)
            {
                dynamic_obstacle.prediction = Prediction(PredictionType::GAUSSIAN);

                const auto &mode = obstacle.gaussians[0];
                for (size_t k = 0; k < mode.mean.poses.size(); k++)
                {
                    dynamic_obstacle.prediction.modes[0].emplace_back(
                        Eigen::Vector2d(mode.mean.poses[k].pose.position.x,
                                        mode.mean.poses[k].pose.position.y),
                        RosTools::quaternionToAngle(mode.mean.poses[k].pose.orientation),
                        mode.major_semiaxis[k],
                        mode.minor_semiaxis[k]);
                }

                if (mode.major_semiaxis.back() == 0. ||
                    !CONFIG["probabilistic"]["enable"].as<bool>())
                    dynamic_obstacle.prediction.type = PredictionType::DETERMINISTIC;
                else
                    dynamic_obstacle.prediction.type = PredictionType::GAUSSIAN;
            }
            else
            {
                ROSTOOLS_ASSERT(false, "Multiple modes not yet supported");
            }
        }
        ensureObstacleSize(_data.dynamic_obstacles, _state);

        if (CONFIG["probabilistic"]["propagate_uncertainty"].as<bool>())
            propagatePredictionUncertainty(_data.dynamic_obstacles);

        if (_planner)
            _planner->onDataReceived(_data, "dynamic obstacles");
    }

    void MPCCore::setGoal(double x, double y)
    {
        _data.goal(0) = x;
        _data.goal(1) = y;
        _data.goal_received = true;
    }

    void MPCCore::setCostmap(nav2_costmap_2d::Costmap2D *costmap)
    {
        _costmap = costmap;
        _data.costmap = costmap;
    }

    bool MPCCore::rotateToGoal(MPCCommand &cmd)
    {
        LOG_INFO_THROTTLE(1500, "Rotating to the goal");
        if (!_data.goal_received)
        {
            LOG_INFO("Waiting for the goal");
            cmd.v = 0.0;
            cmd.w = 0.0;
            cmd.success = false;
            return true;
        }

        double goal_angle = 0.;
        if (_data.reference_path.x.size() > 2)
            goal_angle = std::atan2(_data.reference_path.y[2] - _state.get("y"),
                                    _data.reference_path.x[2] - _state.get("x"));
        else
            goal_angle = std::atan2(_data.goal(1) - _state.get("y"),
                                    _data.goal(0) - _state.get("x"));

        double angle_diff = goal_angle - _state.get("psi");
        if (angle_diff > M_PI)
            angle_diff -= 2 * M_PI;

        if (std::abs(angle_diff) > M_PI / 4.)
        {
            cmd.v = 0.0;
            cmd.w = _enable_output ? 1.5 * RosTools::sgn(angle_diff) : 0.0;
            cmd.success = true;
            return true;
        }

        LOG_SUCCESS("Robot rotated and is ready to follow the path");
        _rotate_to_goal = false;
        cmd.v = 0.0;
        cmd.w = 0.0;
        cmd.success = true;
        return false;
    }

    MPCCommand MPCCore::solve()
    {
        MPCCommand cmd;
        if (!_planner)
            return cmd;

        // Snapshot state/data for this solve so concurrent setters cannot
        // mutate them mid-solve.
        RealTimeData data = _data;
        State state = _state;

        data.planning_start_time = std::chrono::system_clock::now();

        LOG_MARK("============= Loop =============");

        if (CONFIG["debug_output"].as<bool>())
            state.print();

        auto &loop_benchmarker = BENCHMARKERS.getBenchmarker("loop");
        loop_benchmarker.start();

        auto output = _planner->solveMPC(state, data);

        LOG_MARK("Success: " << output.success);

        if (_enable_output && output.success)
        {
            cmd.v = _planner->getSolution(1, "v");
            cmd.w = _planner->getSolution(0, "w");
            cmd.success = true;
        }
        else
        {
            const double deceleration = CONFIG["deceleration_at_infeasible"].as<double>();
            const double dt = 1. / CONFIG["control_frequency"].as<double>();
            const double velocity = _state.get("v");
            const double velocity_after_braking = velocity - deceleration * dt;
            cmd.v = std::max(velocity_after_braking, 0.);
            cmd.w = 0.0;
            cmd.success = false;
        }

        loop_benchmarker.stop();

        if (CONFIG["recording"]["enable"].as<bool>() && output.success)
        {
            auto &data_saver = _planner->getDataSaver();
            data_saver.AddData("input_a", state.get("a"));
            data_saver.AddData("input_v", _planner->getSolution(1, "v"));
            data_saver.AddData("input_w", _planner->getSolution(0, "w"));
            _planner->saveData(state, data);
        }

        // Publish markers every cycle, including failed solves. Without this
        // the visualization stays empty during the infeasible/brake-only
        // window (mpc_wall_collision_analysis.md §5-6), which is exactly when
        // it is needed for debugging.
        _planner->visualize(state, data);
        visualizeHeading();

        LOG_MARK("============= End Loop =============");
        return cmd;
    }

    bool MPCCore::checkGoalReached()
    {
        if (!_planner)
            return false;
        const bool goal_reached =
            _planner->isObjectiveReached(_state, _data) && !_done;
        if (goal_reached)
        {
            LOG_SUCCESS("Goal Reached!");
            _done = true;
        }
        return goal_reached;
    }

    void MPCCore::reset(bool success)
    {
        if (!_planner)
            return;
        _planner->reset(_state, _data, success);
        // RealTimeData::reset() wipes the costmap pointer; re-bind the
        // remembered costmap so static-obstacle modules stay ready.
        if (_costmap)
            _data.costmap = _costmap;
        _done = false;
        _rotate_to_goal = false;
    }

    void MPCCore::visualizeHeading()
    {
        auto &publisher = VISUALS.getPublisher("angle");
        auto &line = publisher.getNewLine();
        line.addLine(
            Eigen::Vector2d(_state.get("x"), _state.get("y")),
            Eigen::Vector2d(_state.get("x") + 1.0 * std::cos(_state.get("psi")),
                            _state.get("y") + 1.0 * std::sin(_state.get("psi"))));
        publisher.publish();
    }
} // namespace local_planner
