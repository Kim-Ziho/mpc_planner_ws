#include <mpc_planner_rosnavigation/ros1_rosnavigation.h>
#include <pluginlib/class_list_macros.h>

#include <mpc_planner/planner.h>

#include <mpc_planner/data_preparation.h>

#include <mpc_planner_util/parameters.h>
#include <mpc_planner_util/load_yaml.hpp>

#include <ros_tools/visuals.h>
#include <ros_tools/logging.h>
#include <ros_tools/convertions.h>
#include <ros_tools/math.h>
#include <ros_tools/data_saver.h>
#include <ros_tools/spline.h>

#include <geometry_msgs/Point.h>
#include <costmap_2d/cost_values.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <vector>

#include <std_msgs/Empty.h>
#include <ros_tools/profiling.h>

using namespace MPCPlanner;

PLUGINLIB_EXPORT_CLASS(local_planner::ROSNavigationPlanner, nav_core::BaseLocalPlanner)

namespace local_planner
{

    ROSNavigationPlanner::ROSNavigationPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

    ROSNavigationPlanner::ROSNavigationPlanner(std::string name, tf2_ros::Buffer *tf,
                                               costmap_2d::Costmap2DROS *costmap_ros)
        : costmap_ros_(NULL), tf_(NULL), initialized_(false)
    {
        initialize(name, tf, costmap_ros);
    }

    void ROSNavigationPlanner::initialize(std::string name, tf2_ros::Buffer *tf,
                                          costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            ros::NodeHandle nh("~/" + name);

            tf_ = tf;

            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            _data.costmap = costmap_;

            initialized_ = true;

            LOG_INFO("Started ROSNavigation Planner");

            VISUALS.init(&general_nh_);

            // Initialize the configuration
            Configuration::getInstance().initialize(SYSTEM_CONFIG_PATH(__FILE__, "settings"));

            _data.robot_area = {Disc(0., CONFIG["robot_radius"].as<double>())};

            // Initialize the planner
            _planner = std::make_unique<Planner>();

            // Initialize the ROS interface
            initializeSubscribersAndPublishers(nh);

            startEnvironment();

            _reconfigure = std::make_unique<RosnavigationReconfigure>();

            updateSpatioTemporalMap();   // 초기 시공간 맵 구성
            publishSpatioTemporalMap();  // 초기 시각화

            _timeout_timer.setDuration(60.);
            _timeout_timer.start();
            for (int i = 0; i < CAMERA_BUFFER; i++)
            {
                _x_buffer[i] = 0.;
                _y_buffer[i] = 0.;
            }

            RosTools::Instrumentor::Get().BeginSession("mpc_planner_rosnavigation");

            LOG_DIVIDER();
        }
    }

    ROSNavigationPlanner::~ROSNavigationPlanner()
    {
        LOG_INFO("Stopped ROSNavigation Planner");
        BENCHMARKERS.print();

        RosTools::Instrumentor::Get().EndSession();
    }

    bool ROSNavigationPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        // check if plugin is initialized
        if (!initialized_)
        {
            ROS_ERROR("planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        // store the global plan
        global_plan_.clear();
        global_plan_ = orig_global_plan;

        // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
        // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.

        // reset goal_reached_ flag
        // goal_reached_ = false;

        return true;
    }

    bool ROSNavigationPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

        auto path = boost::make_shared<nav_msgs::Path>();
        path->poses = global_plan_;
        pathCallback(path);

        if (_rotate_to_goal)
            rotateToGoal(cmd_vel);
        else
            loop(cmd_vel);

        return true;
    }

    void ROSNavigationPlanner::initializeSubscribersAndPublishers(ros::NodeHandle &nh)
    {
        LOG_INFO("initializeSubscribersAndPublishers");

        _state_sub = nh.subscribe<nav_msgs::Odometry>(
            "/input/state", 5,
            boost::bind(&ROSNavigationPlanner::stateCallback, this, _1));

        _state_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
            "/input/state_pose", 5,
            boost::bind(&ROSNavigationPlanner::statePoseCallback, this, _1));

        _goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(
            "/input/goal", 1,
            boost::bind(&ROSNavigationPlanner::goalCallback, this, _1));

        _path_sub = nh.subscribe<nav_msgs::Path>(
            "/input/reference_path", 1,
            boost::bind(&ROSNavigationPlanner::pathCallback, this, _1));

        _obstacle_sim_sub = nh.subscribe<mpc_planner_msgs::ObstacleArray>(
            "/input/obstacles", 1,
            boost::bind(&ROSNavigationPlanner::obstacleCallback, this, _1));

        _cmd_pub = nh.advertise<geometry_msgs::Twist>(
            "/output/command", 1);

        _pose_pub = nh.advertise<geometry_msgs::PoseStamped>(
            "/output/pose", 1);

        _collisions_sub = nh.subscribe<std_msgs::Float64>(
            "/feedback/collisions", 1,
            boost::bind(&ROSNavigationPlanner::collisionCallback, this, _1));

        // Environment Reset
        _reset_simulation_pub = nh.advertise<std_msgs::Empty>("/lmpcc/reset_environment", 1);
        _reset_simulation_client = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
        _reset_ekf_client = nh.serviceClient<robot_localization::SetPose>("/set_pose");

        // Pedestrian simulator
        _ped_horizon_pub = nh.advertise<std_msgs::Int32>("/pedestrian_simulator/horizon", 1);
        _ped_integrator_step_pub = nh.advertise<std_msgs::Float32>("/pedestrian_simulator/integrator_step", 1);
        _ped_clock_frequency_pub = nh.advertise<std_msgs::Float32>("/pedestrian_simulator/clock_frequency", 1);
        _ped_start_client = nh.serviceClient<std_srvs::Empty>("/pedestrian_simulator/start");
    }

    void ROSNavigationPlanner::startEnvironment()
    {

        // Manually add obstacles in the costmap!
        // int mx, my;
        // costmap_->worldToMapEnforceBounds(2., 2., mx, my);
        // LOG_VALUE("mx", mx);
        // LOG_VALUE("my", my);

        // for (int i = 0; i < 10; i++)
        // {
        //     costmap_->setCost(mx + i, my, costmap_2d::LETHAL_OBSTACLE);
        // }

        LOG_INFO("Starting pedestrian simulator");
        for (int i = 0; i < 20; i++)
        {
            std_msgs::Int32 horizon_msg;
            horizon_msg.data = CONFIG["N"].as<int>();
            _ped_horizon_pub.publish(horizon_msg);

            std_msgs::Float32 integrator_step_msg;
            integrator_step_msg.data = CONFIG["integrator_step"].as<double>();
            _ped_integrator_step_pub.publish(integrator_step_msg);

            std_msgs::Float32 clock_frequency_msg;
            clock_frequency_msg.data = CONFIG["control_frequency"].as<double>();
            _ped_clock_frequency_pub.publish(clock_frequency_msg);

            std_srvs::Empty empty_msg;
            if (_ped_start_client.call(empty_msg))
                break;
            else
            {
                LOG_INFO_THROTTLE(3, "Waiting for pedestrian simulator to start");
                ros::Duration(1.0).sleep();

                _reset_simulation_pub.publish(std_msgs::Empty());
            }
        }
        _enable_output = CONFIG["enable_output"].as<bool>();
        LOG_INFO("Environment ready.");
    }

    bool ROSNavigationPlanner::isGoalReached()
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

        bool goal_reached = _planner->isObjectiveReached(_state, _data) && !done_; // Activate once
        if (goal_reached)
        {
            LOG_SUCCESS("Goal Reached!");
            done_ = true;
            reset();
        }

        return goal_reached;
    }

    void ROSNavigationPlanner::rotateToGoal(geometry_msgs::Twist &cmd_vel)
    {
        LOG_INFO_THROTTLE(1500, "Rotating to the goal");
        if (!_data.goal_received)
        {
            LOG_INFO("Waiting for the goal");
            return;
        }
        double goal_angle = 0.;

        if (_data.reference_path.x.size() > 2)
            goal_angle = std::atan2(_data.reference_path.y[2] - _state.get("y"), _data.reference_path.x[2] - _state.get("x"));
        else
            goal_angle = std::atan2(_data.goal(1) - _state.get("y"), _data.goal(0) - _state.get("x"));

        double angle_diff = goal_angle - _state.get("psi");

        if (angle_diff > M_PI)
            angle_diff -= 2 * M_PI;

        geometry_msgs::Twist cmd;
        if (std::abs(angle_diff) > M_PI / 8.)
        {
            cmd_vel.linear.x = 0.0;
            if (_enable_output)
                cmd_vel.angular.z = 1.5 * RosTools::sgn(angle_diff);
            else
                cmd_vel.angular.z = 0.;
        }
        else
        {
            LOG_SUCCESS("Robot rotated and is ready to follow the path");
            _rotate_to_goal = false;
        }
    }

    void ROSNavigationPlanner::loop(geometry_msgs::Twist &cmd_vel)
    {

        // Copy data for thread safety
        RealTimeData data = _data;
        State state = _state;

        data.planning_start_time = std::chrono::system_clock::now();

        LOG_MARK("============= Loop =============");

        if (_timeout_timer.hasFinished())
        {
            reset(false);
            cmd_vel.linear.x = 0.;
            cmd_vel.angular.z = 0.;
            return;
        }

        if (CONFIG["debug_output"].as<bool>())
            state.print();

        auto &loop_benchmarker = BENCHMARKERS.getBenchmarker("loop");
        loop_benchmarker.start();

        auto output = _planner->solveMPC(state, data);

        LOG_MARK("Success: " << output.success);

        geometry_msgs::Twist cmd;
        if (_enable_output && output.success)
        {
            // Publish the command
            cmd_vel.linear.x = _planner->getSolution(1, "v");  // = x1
            cmd_vel.angular.z = _planner->getSolution(0, "w"); // = u0
            LOG_VALUE_DEBUG("Commanded v", cmd.linear.x);
            LOG_VALUE_DEBUG("Commanded w", cmd.angular.z);
        }
        else
        {
            double deceleration = CONFIG["deceleration_at_infeasible"].as<double>();
            double velocity_after_braking;
            double velocity;
            double dt = 1. / CONFIG["control_frequency"].as<double>();

            velocity = _state.get("v");
            velocity_after_braking = velocity - deceleration * dt;   // Brake with the given deceleration
            cmd_vel.linear.x = std::max(velocity_after_braking, 0.); // Don't drive backwards when braking
            cmd_vel.angular.z = 0.0;
        }
        _cmd_pub.publish(cmd);

        publishPose();
        publishCamera();

        loop_benchmarker.stop();

        if (CONFIG["recording"]["enable"].as<bool>())
        {

            // Save control inputs
            if (output.success)
            {
                auto &data_saver = _planner->getDataSaver();
                data_saver.AddData("input_a", state.get("a"));
                data_saver.AddData("input_v", _planner->getSolution(1, "v"));
                data_saver.AddData("input_w", _planner->getSolution(0, "w"));
            }

            _planner->saveData(state, data);
        }
        if (output.success)
        {
            _planner->visualize(state, data);
            visualize();
        }
        LOG_MARK("============= End Loop =============");
    }

    void ROSNavigationPlanner::stateCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        LOG_MARK("State callback");
        _state.set("x", msg->pose.pose.position.x);
        _state.set("y", msg->pose.pose.position.y);
        _state.set("psi", RosTools::quaternionToAngle(msg->pose.pose.orientation));
        _state.set("v", std::sqrt(std::pow(msg->twist.twist.linear.x, 2.) + std::pow(msg->twist.twist.linear.y, 2.)));

        if (std::abs(msg->pose.pose.orientation.x) > (M_PI / 8.) || std::abs(msg->pose.pose.orientation.y) > (M_PI / 8.))
        {
            LOG_WARN("Detected flipped robot. Resetting.");
            reset(false); // Reset without success
        }
    }

    void ROSNavigationPlanner::statePoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        LOG_MARK("State callback");

        _state.set("x", msg->pose.position.x);
        _state.set("y", msg->pose.position.y);
        _state.set("psi", msg->pose.orientation.z);
        _state.set("v", msg->pose.position.z);

        if (std::abs(msg->pose.orientation.x) > (M_PI / 8.) || std::abs(msg->pose.orientation.y) > (M_PI / 8.))
        {
            LOG_ERROR("Detected flipped robot. Resetting.");
            reset(false); // Reset without success
        }
    }

    void ROSNavigationPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        LOG_MARK("Goal callback");

        _data.goal(0) = msg->pose.position.x;
        _data.goal(1) = msg->pose.position.y;
        _data.goal_received = true;

        _rotate_to_goal = true;
    }

    bool ROSNavigationPlanner::isPathTheSame(const nav_msgs::Path::ConstPtr &msg)
    {
        // Check if the path is the same
        if (_data.reference_path.x.size() != msg->poses.size())
            return false;

        // Check up to the first two points
        int num_points = std::min(2, (int)_data.reference_path.x.size());
        for (int i = 0; i < num_points; i++)
        {
            if (!_data.reference_path.pointInPath(i, msg->poses[i].pose.position.x, msg->poses[i].pose.position.y))
                return false;
        }
        return true;
    }

    void ROSNavigationPlanner::pathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        LOG_MARK("Path callback");

        int downsample = CONFIG["downsample_path"].as<double>();

        if (isPathTheSame(msg) || msg->poses.size() < downsample + 1)
            return;

        _data.reference_path.clear();

        int count = 0;
        for (auto &pose : msg->poses)
        {
            if (count % downsample == 0 || count == msg->poses.size() - 1) // Todo
            {
                _data.reference_path.x.push_back(pose.pose.position.x);
                _data.reference_path.y.push_back(pose.pose.position.y);
                _data.reference_path.psi.push_back(RosTools::quaternionToAngle(pose.pose.orientation));
            }
            count++;
        }

        // Fit a clothoid on the global path to sample points on the spline from
        // RosTools::Clothoid2D clothoid(_data.reference_path.x, _data.reference_path.y, _data.reference_path.psi, 2.0);
        // _data.reference_path.clear();
        // clothoid.getPointsOnClothoid(_data.reference_path.x, _data.reference_path.y, _data.reference_path.s);

        // Velocity
        /*LOG_VALUE("velocity reference", CONFIG["weights"]["reference_velocity"].as<double>());
        for (size_t i = 0; i < _data.reference_path.x.size(); i++)
        {
            if (i != _data.reference_path.x.size() - 1)
                _data.reference_path.v.push_back(CONFIG["weights"]["reference_velocity"].as<double>());
            else
                _data.reference_path.v.push_back(0.);
        }*/

        _planner->onDataReceived(_data, "reference_path");
    }

    void ROSNavigationPlanner::obstacleCallback(const mpc_planner_msgs::ObstacleArray::ConstPtr &msg)
    {
        LOG_MARK("Obstacle callback");

        _data.dynamic_obstacles.clear();

        for (auto &obstacle : msg->obstacles)
        {
            // Save the obstacle
            _data.dynamic_obstacles.emplace_back(
                obstacle.id,
                Eigen::Vector2d(obstacle.pose.position.x, obstacle.pose.position.y),
                RosTools::quaternionToAngle(obstacle.pose),
                CONFIG["obstacle_radius"].as<double>());
            auto &dynamic_obstacle = _data.dynamic_obstacles.back();

            if (obstacle.probabilities.size() == 0) // No Predictions!
                continue;

            // Save the prediction
            if (obstacle.probabilities.size() == 1) // One mode
            {
                dynamic_obstacle.prediction = Prediction(PredictionType::GAUSSIAN);

                const auto &mode = obstacle.gaussians[0];
                for (size_t k = 0; k < mode.mean.poses.size(); k++)
                {
                    dynamic_obstacle.prediction.modes[0].emplace_back(
                        Eigen::Vector2d(mode.mean.poses[k].pose.position.x, mode.mean.poses[k].pose.position.y),
                        RosTools::quaternionToAngle(mode.mean.poses[k].pose.orientation),
                        mode.major_semiaxis[k],
                        mode.minor_semiaxis[k]);
                }

                if (mode.major_semiaxis.back() == 0. || !CONFIG["probabilistic"]["enable"].as<bool>())
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

        updateSpatioTemporalMap();  // 예측 기반 시공간 맵 갱신
        publishSpatioTemporalMap(); // RViz 갱신

        _planner->onDataReceived(_data, "dynamic obstacles");
    }

    void ROSNavigationPlanner::updateSpatioTemporalMap()
    {
        if (_data.costmap == nullptr)
            return;

        const unsigned int size_x = _data.costmap->getSizeInCellsX();
        const unsigned int size_y = _data.costmap->getSizeInCellsY();
        const int horizon = CONFIG["N"].as<int>();

        if (size_x == 0 || size_y == 0 || horizon <= 0)
            return;

        const unsigned int time_steps = static_cast<unsigned int>(horizon);
        const double resolution_xy = _data.costmap->getResolution();
        const double resolution_t = CONFIG["integrator_step"].as<double>();
        const double origin_x = _data.costmap->getOriginX();
        const double origin_y = _data.costmap->getOriginY();
        const double robot_radius = CONFIG["robot_radius"].as<double>();

        constexpr unsigned int spatial_downsample = 2;
        const double coarse_resolution_xy = resolution_xy * static_cast<double>(spatial_downsample);
        const unsigned int coarse_full_x = (size_x + spatial_downsample - 1) / spatial_downsample;
        const unsigned int coarse_full_y = (size_y + spatial_downsample - 1) / spatial_downsample;

        double window_radius_m = 15.0;
        double forward_offset_m = 0.0;
        if (CONFIG["spatio_temporal"])
        {
            if (CONFIG["spatio_temporal"]["window_radius"])
                window_radius_m = CONFIG["spatio_temporal"]["window_radius"].as<double>();
            if (CONFIG["spatio_temporal"]["forward_offset"])
                forward_offset_m = CONFIG["spatio_temporal"]["forward_offset"].as<double>();
        }
        const unsigned int window_radius_cells = std::max(1u, static_cast<unsigned int>(std::ceil(window_radius_m / coarse_resolution_xy)));

        unsigned int robot_mx = size_x / 2;
        unsigned int robot_my = size_y / 2;
        if (!_data.costmap->worldToMap(_state.get("x"), _state.get("y"), robot_mx, robot_my))
        {
            robot_mx = size_x / 2;
            robot_my = size_y / 2;
        }

        double psi = _state.get("psi");
        int shift_map_cells_x = static_cast<int>(std::round((forward_offset_m * std::cos(psi)) / resolution_xy));
        int shift_map_cells_y = static_cast<int>(std::round((forward_offset_m * std::sin(psi)) / resolution_xy));

        int shifted_robot_mx = std::clamp(static_cast<int>(robot_mx) + shift_map_cells_x, 0, static_cast<int>(size_x) - 1);
        int shifted_robot_my = std::clamp(static_cast<int>(robot_my) + shift_map_cells_y, 0, static_cast<int>(size_y) - 1);

        unsigned int coarse_robot_x = std::min(coarse_full_x - 1, static_cast<unsigned int>(shifted_robot_mx) / spatial_downsample);
        unsigned int coarse_robot_y = std::min(coarse_full_y - 1, static_cast<unsigned int>(shifted_robot_my) / spatial_downsample);

        const unsigned int min_coarse_x = (coarse_robot_x > window_radius_cells) ? (coarse_robot_x - window_radius_cells) : 0;
        const unsigned int min_coarse_y = (coarse_robot_y > window_radius_cells) ? (coarse_robot_y - window_radius_cells) : 0;
        const unsigned int max_coarse_x = std::min(coarse_full_x - 1, coarse_robot_x + window_radius_cells);
        const unsigned int max_coarse_y = std::min(coarse_full_y - 1, coarse_robot_y + window_radius_cells);

        const unsigned int coarse_size_x = max_coarse_x - min_coarse_x + 1;
        const unsigned int coarse_size_y = max_coarse_y - min_coarse_y + 1;

        auto &map = _data.spatio_temporal_map;
        map.configure(coarse_resolution_xy, resolution_t,
                      origin_x + static_cast<double>(min_coarse_x) * coarse_resolution_xy,
                      origin_y + static_cast<double>(min_coarse_y) * coarse_resolution_xy,
                      0.0, coarse_size_x, coarse_size_y, time_steps);
        map.clear(0.f);

        const float static_value = 1.0f;
        const float dynamic_value = 1.0f;
        const int static_padding_cells = std::max(0, static_cast<int>(std::ceil(robot_radius / coarse_resolution_xy)));

        static std::vector<uint8_t> coarse_static;
        coarse_static.assign(static_cast<size_t>(map.cells_x) * map.cells_y, 0);

        const unsigned int min_x_cell = min_coarse_x * spatial_downsample;
        const unsigned int max_x_cell = std::min(size_x, (max_coarse_x + 1) * spatial_downsample);
        const unsigned int min_y_cell = min_coarse_y * spatial_downsample;
        const unsigned int max_y_cell = std::min(size_y, (max_coarse_y + 1) * spatial_downsample);

        for (unsigned int y = min_y_cell; y < max_y_cell; ++y)
        {
            const unsigned int coarse_y_total = y / spatial_downsample;
            const unsigned int coarse_y = coarse_y_total - min_coarse_y;
            for (unsigned int x = min_x_cell; x < max_x_cell; ++x)
            {
                const unsigned int coarse_x_total = x / spatial_downsample;
                const unsigned int coarse_x = coarse_x_total - min_coarse_x;
                const unsigned char cost = _data.costmap->getCost(x, y);
                if (cost < costmap_2d::LETHAL_OBSTACLE)
                    continue;

                const size_t idx = static_cast<size_t>(coarse_y) * map.cells_x + coarse_x;
                coarse_static[idx] = 1;
            }
        }

        auto markStatic = [&](unsigned int cx, unsigned int cy) {
            for (int dx = -static_padding_cells; dx <= static_padding_cells; ++dx)
            {
                int nx = static_cast<int>(cx) + dx;
                if (nx < 0 || nx >= static_cast<int>(map.cells_x))
                    continue;

                for (int dy = -static_padding_cells; dy <= static_padding_cells; ++dy)
                {
                    int ny = static_cast<int>(cy) + dy;
                    if (ny < 0 || ny >= static_cast<int>(map.cells_y))
                        continue;

                    double dist = std::hypot(dx * coarse_resolution_xy, dy * coarse_resolution_xy);
                    if (dist > robot_radius)
                        continue;

                    for (unsigned int t = 0; t < time_steps; ++t)
                    {
                        const unsigned int ux = static_cast<unsigned int>(nx);
                        const unsigned int uy = static_cast<unsigned int>(ny);
                        if (!map.contains(ux, uy, t))
                            continue;

                        auto &cell = map.at(ux, uy, t);
                        cell = std::max(cell, static_value);
                    }
                }
            }
        };

        for (unsigned int cy = 0; cy < map.cells_y; ++cy)
        {
            for (unsigned int cx = 0; cx < map.cells_x; ++cx)
            {
                const size_t idx = static_cast<size_t>(cy) * map.cells_x + cx;
                if (!coarse_static[idx])
                    continue;

                markStatic(cx, cy);
            }
        }

        // 동적 장애물은 예측 위치마다 로봇 반경과 장애물 반경을 더해 확장하여 누적한다.
        for (const auto &obstacle : _data.dynamic_obstacles)
        {
            std::vector<Eigen::Vector2d> samples;
            samples.reserve(time_steps);
            samples.push_back(obstacle.position);

            if (!obstacle.prediction.empty() && !obstacle.prediction.modes.empty())
            {
                const auto &mode = obstacle.prediction.modes.front();
                for (size_t idx = 0; idx < mode.size() && samples.size() < time_steps; ++idx)
                    samples.push_back(mode[idx].position);
            }

            const size_t usable_samples = std::min(samples.size(), static_cast<size_t>(time_steps));
            const double obstacle_radius = std::max(0.0, obstacle.radius) + robot_radius;
            const int dynamic_padding_cells = std::max(0, static_cast<int>(std::ceil(obstacle_radius / coarse_resolution_xy)));

            for (size_t step = 0; step < usable_samples; ++step)
            {
                unsigned int mx = 0;
                unsigned int my = 0;
                if (!_data.costmap->worldToMap(samples[step].x(), samples[step].y(), mx, my))
                    continue;

                const unsigned int coarse_total_x = mx / spatial_downsample;
                const unsigned int coarse_total_y = my / spatial_downsample;
                if (coarse_total_x < min_coarse_x || coarse_total_x > max_coarse_x ||
                    coarse_total_y < min_coarse_y || coarse_total_y > max_coarse_y)
                    continue;

                const unsigned int coarse_cx = coarse_total_x - min_coarse_x;
                const unsigned int coarse_cy = coarse_total_y - min_coarse_y;

                for (int dx = -dynamic_padding_cells; dx <= dynamic_padding_cells; ++dx)
                {
                    int nx = static_cast<int>(coarse_cx) + dx;
                    if (nx < 0 || nx >= static_cast<int>(map.cells_x))
                        continue;

                    for (int dy = -dynamic_padding_cells; dy <= dynamic_padding_cells; ++dy)
                    {
                        int ny = static_cast<int>(coarse_cy) + dy;
                        if (ny < 0 || ny >= static_cast<int>(map.cells_y))
                            continue;

                        double dist = std::hypot(dx * coarse_resolution_xy, dy * coarse_resolution_xy);
                        if (dist > obstacle_radius)
                            continue;

                        const unsigned int ux = static_cast<unsigned int>(nx);
                        const unsigned int uy = static_cast<unsigned int>(ny);
                        const unsigned int ut = static_cast<unsigned int>(step);
                        if (!map.contains(ux, uy, ut))
                            continue;

                        auto &cell = map.at(ux, uy, ut);
                        cell = dynamic_value;
                    }
                }
            }
        }
    }

    void ROSNavigationPlanner::publishSpatioTemporalMap()
    {
        auto &publisher = VISUALS.getPublisher("spatio_temporal_map");
        auto &map = _data.spatio_temporal_map;

        if (map.empty())
        {
            publisher.publish();
            return;
        }

        // 확률값(0~1)에 따라 색상의 알파를 다르게 표현하기 위해 버킷을 만든다.
        std::map<float, std::vector<geometry_msgs::Point>> buckets;

        for (unsigned int t = 0; t < map.time_steps; ++t)
        {
            const double z = map.origin_t + (static_cast<double>(t) + 0.5) * map.resolution_t;
            for (unsigned int y = 0; y < map.cells_y; ++y)
            {
                for (unsigned int x = 0; x < map.cells_x; ++x)
                {
                    const float value = map.at(x, y, t);
                    if (value <= 0.f)
                        continue;

                    geometry_msgs::Point p;
                    p.x = map.origin_x + (static_cast<double>(x) + 0.5) * map.resolution_xy;
                    p.y = map.origin_y + (static_cast<double>(y) + 0.5) * map.resolution_xy;
                    p.z = z;

                    buckets[value].push_back(p);
                }
            }
        }

        const double cube_z = std::max(map.resolution_t, map.resolution_xy);

        for (auto &entry : buckets)
        {
            auto &cubes = publisher.getNewMultiplePointMarker("CUBE");
            const double alpha = std::max(0.0f, std::min(1.0f, entry.first)) * 0.5; // 확률 1.0을 50% 투명도로 매핑
            cubes.setColor(1.0, 1.0, 0.0, alpha);                                  // 모든 영역을 노란색으로 표현

            cubes.setScale(map.resolution_xy, map.resolution_xy, cube_z);

            for (const auto &point : entry.second)
                cubes.addPointMarker(point);

            cubes.finishPoints();
        }

        publisher.publish();
    }

    void ROSNavigationPlanner::visualize()
    {
        auto &publisher = VISUALS.getPublisher("angle");
        auto &line = publisher.getNewLine();

        line.addLine(Eigen::Vector2d(_state.get("x"), _state.get("y")),
                     Eigen::Vector2d(_state.get("x") + 1.0 * std::cos(_state.get("psi")), _state.get("y") + 1.0 * std::sin(_state.get("psi"))));
        publisher.publish();
    }

    void ROSNavigationPlanner::reset(bool success)
    {
        LOG_INFO("Resetting");
        boost::mutex::scoped_lock l(_reset_mutex);

        _reset_simulation_client.call(_reset_msg);
        _reset_ekf_client.call(_reset_pose_msg);
        _reset_simulation_pub.publish(std_msgs::Empty());

        for (int i = 0; i < CAMERA_BUFFER; i++)
        {
            _x_buffer[i] = 0.;
            _y_buffer[i] = 0.;
        }

        _planner->reset(_state, _data, success);
        _data.costmap = costmap_;

        updateSpatioTemporalMap();   // 리셋 직후 시공간 정보 갱신
        publishSpatioTemporalMap();  // RViz 갱신

        ros::Duration(1.0 / CONFIG["control_frequency"].as<double>()).sleep();

        done_ = false;
        _rotate_to_goal = false;

        _timeout_timer.start();
    }

    void ROSNavigationPlanner::collisionCallback(const std_msgs::Float64::ConstPtr &msg)
    {
        LOG_MARK("Collision callback");

        _data.intrusion = (float)(msg->data);

        if (_data.intrusion > 0.)
            LOG_INFO_THROTTLE(500., "Collision detected (Intrusion: " << _data.intrusion << ")");
    }

    void ROSNavigationPlanner::publishPose()
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = _state.get("x");
        pose.pose.position.y = _state.get("y");
        pose.pose.orientation = RosTools::angleToQuaternion(_state.get("psi"));

        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";

        _pose_pub.publish(pose);
    }

    void ROSNavigationPlanner::publishCamera()
    {
        geometry_msgs::TransformStamped msg;
        msg.header.stamp = ros::Time::now();

        if ((msg.header.stamp - _prev_stamp) < ros::Duration(0.5 / CONFIG["control_frequency"].as<double>()))
            return;

        _prev_stamp = msg.header.stamp;

        msg.header.frame_id = "map";
        msg.child_frame_id = "camera";

        // Smoothen the camera
        for (int i = 0; i < CAMERA_BUFFER - 1; i++)
        {
            _x_buffer[i] = _x_buffer[i + 1];
            _y_buffer[i] = _y_buffer[i + 1];
        }
        _x_buffer[CAMERA_BUFFER - 1] = _state.get("x");
        _y_buffer[CAMERA_BUFFER - 1] = _state.get("y");
        double camera_x = 0., camera_y = 0.;
        for (int i = 0; i < CAMERA_BUFFER; i++)
        {
            camera_x += _x_buffer[i];
            camera_y += _y_buffer[i];
        }
        msg.transform.translation.x = camera_x / (double)CAMERA_BUFFER; //_state.get("x");
        msg.transform.translation.y = camera_y / (double)CAMERA_BUFFER; //_state.get("y");
        msg.transform.translation.z = 0.0;
        msg.transform.rotation.x = 0;
        msg.transform.rotation.y = 0;
        msg.transform.rotation.z = 0;
        msg.transform.rotation.w = 1;

        _camera_pub.sendTransform(msg);
    }

} // namespace local_planner
