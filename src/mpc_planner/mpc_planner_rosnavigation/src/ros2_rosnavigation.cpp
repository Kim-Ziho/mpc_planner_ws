#include <mpc_planner_rosnavigation/ros2_rosnavigation.h>

#include <mpc_planner/planner.h>

#include <mpc_planner_util/parameters.h>
#include <mpc_planner_util/load_yaml.hpp>

#include <ros_tools/visuals.h>
#include <ros_tools/logging.h>
#include <ros_tools/convertions.h>
#include <ros_tools/math.h>
#include <ros_tools/profiling.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

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

        _core = std::make_unique<MPCCore>();
        _core->configure(SYSTEM_CONFIG_PATH(__FILE__, "settings"));

        _reconfigure = std::make_unique<RosnavigationReconfigure>(this);

        _camera_pub = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        _prev_camera_stamp = this->now();

        initializeSubscribersAndPublishers();
        initializeCostmap();
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
        if (_costmap_ros)
        {
            _costmap_ros->deactivate();
            _costmap_ros->cleanup();
        }
        _costmap_thread.reset();
        _costmap_ros.reset();
        if (_core)
            _core->setCostmap(nullptr);
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

        // Nav2 NavfnPlanner action client. The reference path used by the MPC
        // is sourced from planner_server's grid-based plan rather than a
        // straight-line path published externally.
        _compute_path_client = rclcpp_action::create_client<
            nav2_msgs::action::ComputePathToPose>(this, "/compute_path_to_pose");
    }

    void JackalPlanner::initializeCostmap()
    {
        LOG_INFO("Bringing up local_costmap (nav2_costmap_2d)");

        const std::string pkg_share =
            ament_index_cpp::get_package_share_directory("mpc_planner_rosnavigation");
        const std::string yaml_path = pkg_share + "/config/local_costmap.yaml";

        rclcpp::NodeOptions opts;
        opts.arguments({
            "--ros-args",
            "-r", "__node:=local_costmap",
            "--params-file", yaml_path,
        });

        _costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(opts);
        _costmap_thread = std::make_unique<nav2_util::NodeThread>(_costmap_ros);

        _costmap_ros->configure();
        _costmap_ros->activate();

        _core->setCostmap(_costmap_ros->getCostmap());
        LOG_INFO("local_costmap active");
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
        _core->setEnableOutput(CONFIG["enable_output"].as<bool>());
        LOG_INFO("Environment ready.");
    }

    void JackalPlanner::loop()
    {
        geometry_msgs::msg::Twist cmd_vel;

        if (_timeout_timer.hasFinished())
        {
            reset(false);
            cmd_vel.linear.x = 0.;
            cmd_vel.angular.z = 0.;
            _cmd_pub->publish(cmd_vel);
            publishPose();
            publishCamera();
            return;
        }

        MPCCommand cmd;
        if (_core->isRotating())
            _core->rotateToGoal(cmd);
        else
            cmd = _core->solve();

        cmd_vel.linear.x = cmd.v;
        cmd_vel.angular.z = cmd.w;
        _cmd_pub->publish(cmd_vel);

        publishPose();
        publishCamera();

        if (_core->checkGoalReached())
            reset(true);
    }

    void JackalPlanner::stateCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        LOG_MARK("State callback");
        State s = _core->state();
        s.set("x", msg->pose.pose.position.x);
        s.set("y", msg->pose.pose.position.y);
        s.set("psi", RosTools::quaternionToAngle(msg->pose.pose.orientation));
        s.set("v", std::sqrt(std::pow(msg->twist.twist.linear.x, 2.) +
                             std::pow(msg->twist.twist.linear.y, 2.)));
        _core->setState(s);

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
        State s = _core->state();
        s.set("x", msg->pose.position.x);
        s.set("y", msg->pose.position.y);
        s.set("psi", msg->pose.orientation.z);
        s.set("v", msg->pose.position.z);
        _core->setState(s);
    }

    void JackalPlanner::goalCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        LOG_MARK("Goal callback");
        _core->setGoal(msg->pose.position.x, msg->pose.position.y);
        _core->requestRotation();

        requestGlobalPlan(*msg);
    }

    void JackalPlanner::requestGlobalPlan(const geometry_msgs::msg::PoseStamped &goal)
    {
        if (!_compute_path_client)
            return;

        // Non-blocking readiness check; planner_server may not be up yet on
        // the first goalCallback during launch.
        if (!_compute_path_client->action_server_is_ready())
        {
            LOG_WARN_THROTTLE(5000,
                "compute_path_to_pose action server not ready; skipping global plan request.");
            return;
        }

        nav2_msgs::action::ComputePathToPose::Goal action_goal;
        action_goal.goal = goal;
        action_goal.goal.header.stamp = this->now();
        if (action_goal.goal.header.frame_id.empty())
            action_goal.goal.header.frame_id = "map";
        action_goal.use_start = false;
        action_goal.planner_id = "GridBased";

        auto opts = rclcpp_action::Client<
            nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
        opts.result_callback = std::bind(&JackalPlanner::onPlanResult, this,
                                         std::placeholders::_1);
        _compute_path_client->async_send_goal(action_goal, opts);
        LOG_MARK("compute_path_to_pose goal dispatched");
    }

    void JackalPlanner::onPlanResult(
        const rclcpp_action::ClientGoalHandle<
            nav2_msgs::action::ComputePathToPose>::WrappedResult &result)
    {
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
        {
            LOG_WARN("Global plan request did not succeed (code="
                     << static_cast<int>(result.code) << ")");
            return;
        }
        if (!result.result || result.result->path.poses.empty())
        {
            LOG_WARN("Global plan returned an empty path");
            return;
        }

        _core->setReferencePath(result.result->path);
    }

    void JackalPlanner::pathCallback(nav_msgs::msg::Path::ConstSharedPtr msg)
    {
        LOG_MARK("Path callback");
        _core->setReferencePath(*msg);
    }

    void JackalPlanner::obstacleCallback(mpc_planner_msgs::msg::ObstacleArray::ConstSharedPtr msg)
    {
        LOG_MARK("Obstacle callback");
        _core->setObstacles(*msg);
    }

    void JackalPlanner::collisionCallback(std_msgs::msg::Float64::ConstSharedPtr msg)
    {
        _core->data().intrusion = (float)msg->data;
        if (_core->data().intrusion > 0.)
            LOG_INFO_THROTTLE(500, "Collision detected (Intrusion: "
                                       << _core->data().intrusion << ")");
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

        _core->reset(success);
        _timeout_timer.start();
    }

    void JackalPlanner::publishPose()
    {
        const auto &state = _core->state();
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = state.get("x");
        pose.pose.position.y = state.get("y");
        pose.pose.orientation = RosTools::angleToQuaternion(state.get("psi"));
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

        const auto &state = _core->state();
        for (int i = 0; i < CAMERA_BUFFER - 1; i++)
        {
            _x_buffer[i] = _x_buffer[i + 1];
            _y_buffer[i] = _y_buffer[i + 1];
        }
        _x_buffer[CAMERA_BUFFER - 1] = state.get("x");
        _y_buffer[CAMERA_BUFFER - 1] = state.get("y");
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
