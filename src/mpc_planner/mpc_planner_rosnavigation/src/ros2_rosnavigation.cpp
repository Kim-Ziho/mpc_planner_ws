#include <mpc_planner_rosnavigation/ros2_rosnavigation.h>

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
#include <ros_tools/profiling.h>

#include <cmath>
#include <chrono>

using namespace MPCPlanner;
using namespace std::chrono_literals;

namespace local_planner
{

    static rclcpp::NodeOptions makeNodeOptions()
    {
        // ros_tools::retrieveParameter declares parameters lazily on first access,
        // so we leave automatic declaration off to avoid ParameterAlreadyDeclared.
        // Yaml overrides loaded via launch's `parameters=[...]` still apply as
        // initial values that the lazy declare will pick up.
        rclcpp::NodeOptions options;
        return options;
    }

    JackalPlanner::JackalPlanner()
        : rclcpp::Node("jackal_planner", makeNodeOptions())
    {
    }

    void JackalPlanner::initialize()
    {
        // Static node pointer is what ros_tools logging/visuals macros dereference.
        // Must be set before any LOG_* / VISUALS call.
        STATIC_NODE_POINTER.init(this);
        VISUALS.init(this);

        // Initialize the configuration from settings.yaml
        Configuration::getInstance().initialize(SYSTEM_CONFIG_PATH(__FILE__, "settings"));

        _data.robot_area = {Disc(0., CONFIG["robot_radius"].as<double>())};

        _planner = std::make_unique<Planner>();
        _reconfigure = std::make_unique<RosnavigationReconfigure>(this);

        _camera_pub = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        _prev_camera_stamp = this->now();

        initializeSubscribersAndPublishers();
        startEnvironment();

        _timeout_timer.setDuration(60.);
        _timeout_timer.start();
        for (int i = 0; i < CAMERA_BUFFER; i++)
        {
            _x_buffer[i] = 0.;
            _y_buffer[i] = 0.;
        }

        // Control loop timer at the configured rate
        const double rate_hz = CONFIG["control_frequency"].as<double>();
        const auto period =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / rate_hz));
        _timer = this->create_wall_timer(period, std::bind(&JackalPlanner::loop, this));

        RosTools::Instrumentor::Get().BeginSession("mpc_planner_rosnavigation");
        LOG_DIVIDER();
        LOG_INFO("Started JackalPlanner node");
    }

    JackalPlanner::~JackalPlanner()
    {
        LOG_INFO("Stopping JackalPlanner node");
        BENCHMARKERS.print();
        RosTools::Instrumentor::Get().EndSession();
    }

    void JackalPlanner::initializeSubscribersAndPublishers()
    {
        LOG_INFO("initializeSubscribersAndPublishers");

        const auto qos = rclcpp::QoS(5);
        const auto qos_one = rclcpp::QoS(1);

        _state_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/input/state", qos,
            std::bind(&JackalPlanner::stateCallback, this, std::placeholders::_1));

        _state_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/input/state_pose", qos,
            std::bind(&JackalPlanner::statePoseCallback, this, std::placeholders::_1));

        _goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/input/goal", qos_one,
            std::bind(&JackalPlanner::goalCallback, this, std::placeholders::_1));

        _path_sub = this->create_subscription<nav_msgs::msg::Path>(
            "/input/reference_path", qos_one,
            std::bind(&JackalPlanner::pathCallback, this, std::placeholders::_1));

        _obstacle_sub = this->create_subscription<mpc_planner_msgs::msg::ObstacleArray>(
            "/input/obstacles", qos_one,
            std::bind(&JackalPlanner::obstacleCallback, this, std::placeholders::_1));

        _collisions_sub = this->create_subscription<std_msgs::msg::Float64>(
            "/feedback/collisions", qos_one,
            std::bind(&JackalPlanner::collisionCallback, this, std::placeholders::_1));

        _cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/output/command", qos_one);
        _pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/output/pose", qos_one);

        // Pedestrian simulator setup
        _ped_horizon_pub = this->create_publisher<std_msgs::msg::Int32>(
            "/pedestrian_simulator/horizon", qos_one);
        _ped_integrator_step_pub = this->create_publisher<std_msgs::msg::Float32>(
            "/pedestrian_simulator/integrator_step", qos_one);
        _ped_clock_frequency_pub = this->create_publisher<std_msgs::msg::Float32>(
            "/pedestrian_simulator/clock_frequency", qos_one);
        _ped_start_client = this->create_client<std_srvs::srv::Empty>(
            "/pedestrian_simulator/start");

        _reset_simulation_pub = this->create_publisher<std_msgs::msg::Empty>(
            "/lmpcc/reset_environment", qos_one);
        _reset_simulation_client = this->create_client<std_srvs::srv::Empty>(
            "/gazebo/reset_world");
    }

    void JackalPlanner::startEnvironment()
    {
        LOG_INFO("Starting pedestrian simulator");
        for (int i = 0; i < 20; i++)
        {
            std_msgs::msg::Int32 horizon_msg;
            horizon_msg.data = CONFIG["N"].as<int>();
            _ped_horizon_pub->publish(horizon_msg);

            std_msgs::msg::Float32 integrator_step_msg;
            integrator_step_msg.data = CONFIG["integrator_step"].as<double>();
            _ped_integrator_step_pub->publish(integrator_step_msg);

            std_msgs::msg::Float32 clock_frequency_msg;
            clock_frequency_msg.data = CONFIG["control_frequency"].as<double>();
            _ped_clock_frequency_pub->publish(clock_frequency_msg);

            if (_ped_start_client->wait_for_service(std::chrono::milliseconds(200)))
            {
                auto request = std::make_shared<std_srvs::srv::Empty::Request>();
                auto future = _ped_start_client->async_send_request(request);
                break;
            }
            LOG_INFO_THROTTLE(3000, "Waiting for pedestrian simulator to start");
            _reset_simulation_pub->publish(std_msgs::msg::Empty());
            rclcpp::sleep_for(1s);
        }
        _enable_output = CONFIG["enable_output"].as<bool>();
        LOG_INFO("Environment ready.");
    }

    void JackalPlanner::loop()
    {
        geometry_msgs::msg::Twist cmd_vel;
        if (_rotate_to_goal)
            rotateToGoal(cmd_vel);
        else
            runMPC(cmd_vel);

        _cmd_pub->publish(cmd_vel);

        publishPose();
        publishCamera();

        isGoalReached();
    }

    void JackalPlanner::rotateToGoal(geometry_msgs::msg::Twist &cmd_vel)
    {
        LOG_INFO_THROTTLE(1500, "Rotating to the goal");
        if (!_data.goal_received)
        {
            LOG_INFO("Waiting for the goal");
            return;
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

    void JackalPlanner::runMPC(geometry_msgs::msg::Twist &cmd_vel)
    {
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

        if (_enable_output && output.success)
        {
            cmd_vel.linear.x = _planner->getSolution(1, "v");
            cmd_vel.angular.z = _planner->getSolution(0, "w");
        }
        else
        {
            const double deceleration = CONFIG["deceleration_at_infeasible"].as<double>();
            const double dt = 1. / CONFIG["control_frequency"].as<double>();
            const double velocity = _state.get("v");
            const double velocity_after_braking = velocity - deceleration * dt;
            cmd_vel.linear.x = std::max(velocity_after_braking, 0.);
            cmd_vel.angular.z = 0.0;
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

        if (output.success)
        {
            _planner->visualize(state, data);
            visualize();
        }

        LOG_MARK("============= End Loop =============");
    }

    bool JackalPlanner::isGoalReached()
    {
        bool goal_reached = _planner && _planner->isObjectiveReached(_state, _data) && !_done;
        if (goal_reached)
        {
            LOG_SUCCESS("Goal Reached!");
            _done = true;
            reset();
        }
        return goal_reached;
    }

    void JackalPlanner::stateCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        LOG_MARK("State callback");
        _state.set("x", msg->pose.pose.position.x);
        _state.set("y", msg->pose.pose.position.y);
        _state.set("psi", RosTools::quaternionToAngle(msg->pose.pose.orientation));
        _state.set("v", std::sqrt(std::pow(msg->twist.twist.linear.x, 2.) +
                                  std::pow(msg->twist.twist.linear.y, 2.)));

        if (std::abs(msg->pose.pose.orientation.x) > (M_PI / 8.) ||
            std::abs(msg->pose.pose.orientation.y) > (M_PI / 8.))
        {
            LOG_WARN("Detected flipped robot. Resetting.");
            reset(false);
        }
    }

    void JackalPlanner::statePoseCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        LOG_MARK("State pose callback");
        _state.set("x", msg->pose.position.x);
        _state.set("y", msg->pose.position.y);
        _state.set("psi", msg->pose.orientation.z);
        _state.set("v", msg->pose.position.z);
    }

    void JackalPlanner::goalCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        LOG_MARK("Goal callback");
        _data.goal(0) = msg->pose.position.x;
        _data.goal(1) = msg->pose.position.y;
        _data.goal_received = true;
        _rotate_to_goal = true;
    }

    bool JackalPlanner::isPathTheSame(nav_msgs::msg::Path::ConstSharedPtr msg)
    {
        if (_data.reference_path.x.size() != msg->poses.size())
            return false;

        const int num_points = std::min(2, (int)_data.reference_path.x.size());
        for (int i = 0; i < num_points; i++)
        {
            if (!_data.reference_path.pointInPath(i, msg->poses[i].pose.position.x,
                                                  msg->poses[i].pose.position.y))
                return false;
        }
        return true;
    }

    void JackalPlanner::pathCallback(nav_msgs::msg::Path::ConstSharedPtr msg)
    {
        LOG_MARK("Path callback");
        const int downsample = CONFIG["downsample_path"].as<double>();

        if (isPathTheSame(msg) || (int)msg->poses.size() < downsample + 1)
            return;

        _data.reference_path.clear();

        int count = 0;
        for (auto &pose : msg->poses)
        {
            if (count % downsample == 0 || count == (int)msg->poses.size() - 1)
            {
                _data.reference_path.x.push_back(pose.pose.position.x);
                _data.reference_path.y.push_back(pose.pose.position.y);
                _data.reference_path.psi.push_back(RosTools::quaternionToAngle(pose.pose.orientation));
            }
            count++;
        }

        _planner->onDataReceived(_data, "reference_path");
    }

    void JackalPlanner::obstacleCallback(mpc_planner_msgs::msg::ObstacleArray::ConstSharedPtr msg)
    {
        LOG_MARK("Obstacle callback");
        _data.dynamic_obstacles.clear();

        for (auto &obstacle : msg->obstacles)
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

        _planner->onDataReceived(_data, "dynamic obstacles");
    }

    void JackalPlanner::collisionCallback(std_msgs::msg::Float64::ConstSharedPtr msg)
    {
        _data.intrusion = (float)msg->data;
        if (_data.intrusion > 0.)
            LOG_INFO_THROTTLE(500, "Collision detected (Intrusion: " << _data.intrusion << ")");
    }

    void JackalPlanner::reset(bool success)
    {
        LOG_INFO("Resetting");
        std::lock_guard<std::mutex> lock(_reset_mutex);

        if (_reset_simulation_client->wait_for_service(std::chrono::milliseconds(50)))
        {
            auto request = std::make_shared<std_srvs::srv::Empty::Request>();
            _reset_simulation_client->async_send_request(request);
        }
        _reset_simulation_pub->publish(std_msgs::msg::Empty());

        for (int i = 0; i < CAMERA_BUFFER; i++)
        {
            _x_buffer[i] = 0.;
            _y_buffer[i] = 0.;
        }

        _planner->reset(_state, _data, success);

        _done = false;
        _rotate_to_goal = false;
        _timeout_timer.start();
    }

    void JackalPlanner::publishPose()
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = _state.get("x");
        pose.pose.position.y = _state.get("y");
        pose.pose.orientation = RosTools::angleToQuaternion(_state.get("psi"));
        pose.header.stamp = this->now();
        pose.header.frame_id = "map";
        _pose_pub->publish(pose);
    }

    void JackalPlanner::publishCamera()
    {
        const auto stamp = this->now();
        if ((stamp - _prev_camera_stamp) <
            rclcpp::Duration::from_seconds(0.5 / CONFIG["control_frequency"].as<double>()))
            return;
        _prev_camera_stamp = stamp;

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

        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "map";
        msg.child_frame_id = "camera";
        msg.transform.translation.x = camera_x / (double)CAMERA_BUFFER;
        msg.transform.translation.y = camera_y / (double)CAMERA_BUFFER;
        msg.transform.translation.z = 0.0;
        msg.transform.rotation.w = 1.0;

        _camera_pub->sendTransform(msg);
    }

    void JackalPlanner::visualize()
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<local_planner::JackalPlanner>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
