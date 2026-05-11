// scenario_orchestrator: owns the simulation/benchmark plumbing that used to
// live inside JackalPlanner. With MPC running as a nav2_core::Controller
// plugin inside controller_server, the orchestrator drives the scenario via
// the NavigateToPose action and resets pedsim + Gazebo around it.
//
// Responsibilities:
//   - Pedsim handshake on startup (/pedestrian_simulator/{horizon,
//     integrator_step, clock_frequency, start}).
//   - Translate /move_base_simple/goal -> NavigateToPose (replaces the
//     bridge that bt_navigator's panel would normally provide).
//   - Per-attempt 60s timeout: cancel the action, publish
//     /lmpcc/reset_environment and call /gazebo/reset_world. goal_publisher.py
//     listens to /lmpcc/reset_environment and re-fires a new random goal.
//   - On NavigateToPose result (any code): publish /lmpcc/reset_environment +
//     /gazebo/reset_world so the next attempt starts from a known state.
//   - Camera TF map -> camera (rolling-average smoothing) so RViz's camera
//     reference frame keeps following the robot, same as the old node.
//
// See docs/nav2_full_plugin_migration_plan.md §4 for design context.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <mpc_planner_util/parameters.h>
#include <mpc_planner_util/load_yaml.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

namespace
{
    constexpr int CAMERA_BUFFER = 10;
}

class ScenarioOrchestrator : public rclcpp::Node
{
public:
    ScenarioOrchestrator()
        : rclcpp::Node("scenario_orchestrator")
    {
        // Resolve settings.yaml for handshake values (N, integrator_step,
        // control_frequency). We don't load the MPC config; we just need the
        // pedsim handshake numbers from the same source so they match.
        const std::string pkg_share =
            ament_index_cpp::get_package_share_directory("mpc_planner_rosnavigation");
        Configuration::getInstance().initialize(pkg_share + "/config/settings.yaml");

        const auto qos = rclcpp::QoS(1);

        _goal_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/move_base_simple/goal", qos,
            std::bind(&ScenarioOrchestrator::onGoal, this, std::placeholders::_1));

        _odom_sub = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", rclcpp::QoS(5),
            std::bind(&ScenarioOrchestrator::onOdom, this, std::placeholders::_1));

        _collision_sub = create_subscription<std_msgs::msg::Float64>(
            "/feedback/collisions", qos,
            std::bind(&ScenarioOrchestrator::onCollision, this, std::placeholders::_1));

        _ped_horizon_pub = create_publisher<std_msgs::msg::Int32>(
            "/pedestrian_simulator/horizon", qos);
        _ped_integrator_step_pub = create_publisher<std_msgs::msg::Float32>(
            "/pedestrian_simulator/integrator_step", qos);
        _ped_clock_frequency_pub = create_publisher<std_msgs::msg::Float32>(
            "/pedestrian_simulator/clock_frequency", qos);
        _reset_simulation_pub = create_publisher<std_msgs::msg::Empty>(
            "/lmpcc/reset_environment", qos);

        _ped_start_client = create_client<std_srvs::srv::Empty>(
            "/pedestrian_simulator/start");
        _gazebo_reset_client = create_client<std_srvs::srv::Empty>(
            "/gazebo/reset_world");

        _navigate_client = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        _prev_camera_stamp = now();
        for (int i = 0; i < CAMERA_BUFFER; ++i)
        {
            _x_buffer[i] = 0.;
            _y_buffer[i] = 0.;
        }

        _timeout_seconds = 60.0;

        // Run pedsim handshake on a one-shot timer so this constructor stays
        // non-blocking (the service_wait below sleeps up to ~20*200ms).
        _startup_timer = create_wall_timer(
            100ms, [this]() {
                _startup_timer->cancel();
                startEnvironment();
            });
    }

private:
    void startEnvironment()
    {
        RCLCPP_INFO(get_logger(), "Starting pedestrian simulator");
        for (int i = 0; i < 20; ++i)
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

            if (_ped_start_client->wait_for_service(200ms))
            {
                auto request = std::make_shared<std_srvs::srv::Empty::Request>();
                _ped_start_client->async_send_request(request);
                RCLCPP_INFO(get_logger(), "Pedestrian simulator started");
                return;
            }
            _reset_simulation_pub->publish(std_msgs::msg::Empty());
            rclcpp::sleep_for(1s);
        }
        RCLCPP_WARN(get_logger(),
                    "Pedestrian simulator did not respond in 20 attempts");
    }

    void onGoal(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        sendNavigateGoal(*msg);
    }

    void sendNavigateGoal(const geometry_msgs::msg::PoseStamped &goal)
    {
        if (!_navigate_client->wait_for_action_server(2s))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "navigate_to_pose action server unavailable; dropping goal");
            return;
        }

        NavigateToPose::Goal action_goal;
        action_goal.pose = goal;
        action_goal.pose.header.stamp = now();
        if (action_goal.pose.header.frame_id.empty())
            action_goal.pose.header.frame_id = "map";

        auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        opts.result_callback =
            std::bind(&ScenarioOrchestrator::onNavigateResult, this, std::placeholders::_1);
        _navigate_client->async_send_goal(action_goal, opts);

        // (Re)arm the per-attempt timeout.
        if (_timeout_timer)
            _timeout_timer->cancel();
        _timeout_timer = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(_timeout_seconds)),
            [this]() {
                _timeout_timer->cancel();
                RCLCPP_WARN(get_logger(),
                            "Per-attempt timeout reached; resetting scenario");
                cancelAndReset();
            });
    }

    void onNavigateResult(const GoalHandle::WrappedResult &result)
    {
        if (_timeout_timer)
            _timeout_timer->cancel();

        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "NavigateToPose succeeded");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(get_logger(), "NavigateToPose canceled");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(get_logger(), "NavigateToPose aborted");
            break;
        default:
            RCLCPP_WARN(get_logger(),
                        "NavigateToPose ended with unknown code %d",
                        static_cast<int>(result.code));
            break;
        }
        resetScenario();
    }

    void cancelAndReset()
    {
        _navigate_client->async_cancel_all_goals();
        resetScenario();
    }

    void resetScenario()
    {
        std::lock_guard<std::mutex> lock(_reset_mutex);
        if (_gazebo_reset_client->wait_for_service(50ms))
        {
            auto request = std::make_shared<std_srvs::srv::Empty::Request>();
            _gazebo_reset_client->async_send_request(request);
        }
        _reset_simulation_pub->publish(std_msgs::msg::Empty());

        for (int i = 0; i < CAMERA_BUFFER; ++i)
        {
            _x_buffer[i] = 0.;
            _y_buffer[i] = 0.;
        }
    }

    void onOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        // Update rolling-average camera position and publish map -> camera TF
        // at half the control frequency. Same cadence as the old JackalPlanner.
        const auto stamp = now();
        const double rate = CONFIG["control_frequency"].as<double>();
        if ((stamp - _prev_camera_stamp) <
            rclcpp::Duration::from_seconds(0.5 / rate))
            return;
        _prev_camera_stamp = stamp;

        for (int i = 0; i < CAMERA_BUFFER - 1; ++i)
        {
            _x_buffer[i] = _x_buffer[i + 1];
            _y_buffer[i] = _y_buffer[i + 1];
        }
        _x_buffer[CAMERA_BUFFER - 1] = msg->pose.pose.position.x;
        _y_buffer[CAMERA_BUFFER - 1] = msg->pose.pose.position.y;

        double cx = 0., cy = 0.;
        for (int i = 0; i < CAMERA_BUFFER; ++i)
        {
            cx += _x_buffer[i];
            cy += _y_buffer[i];
        }

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = "map";
        tf.child_frame_id = "camera";
        tf.transform.translation.x = cx / CAMERA_BUFFER;
        tf.transform.translation.y = cy / CAMERA_BUFFER;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation.w = 1.0;
        _tf_broadcaster->sendTransform(tf);
    }

    void onCollision(std_msgs::msg::Float64::ConstSharedPtr msg)
    {
        if (msg->data > 0.)
        {
            auto clk = *get_clock();
            RCLCPP_INFO_STREAM_THROTTLE(get_logger(), clk, 500,
                "Collision detected (Intrusion: " << msg->data << ")");
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _goal_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _collision_sub;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _ped_horizon_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _ped_integrator_step_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _ped_clock_frequency_pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _reset_simulation_pub;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _ped_start_client;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _gazebo_reset_client;

    rclcpp_action::Client<NavigateToPose>::SharedPtr _navigate_client;

    rclcpp::TimerBase::SharedPtr _startup_timer;
    rclcpp::TimerBase::SharedPtr _timeout_timer;

    std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
    rclcpp::Time _prev_camera_stamp;
    double _x_buffer[CAMERA_BUFFER]{};
    double _y_buffer[CAMERA_BUFFER]{};

    double _timeout_seconds{60.0};
    std::mutex _reset_mutex;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScenarioOrchestrator>());
    rclcpp::shutdown();
    return 0;
}
