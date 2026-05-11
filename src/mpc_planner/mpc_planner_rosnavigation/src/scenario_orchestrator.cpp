// scenario_orchestrator: owns the simulation/benchmark plumbing that used to
// live inside JackalPlanner. With MPC running as a nav2_core::Controller
// plugin inside controller_server, the orchestrator drives the scenario
// end-to-end via the NavigateToPose action and resets pedsim + Gazebo around
// each attempt.
//
// Responsibilities:
//   - Pedsim handshake on startup (/pedestrian_simulator/{horizon,
//     integrator_step, clock_frequency, start}). Pedsim is started
//     immediately so /pedestrian_simulator/trajectory_predictions flows
//     -- the MPC plugin's isDataReady check needs obstacles before it
//     will produce a cmd_vel. Pedsim is launched paused
//     (/pedestrian_simulator/paused = true) so its Loop publishes
//     predictions but pedestrians do not physically advance. The pause is
//     lifted from onOdom() the moment the robot crosses the motion
//     threshold, so pedestrians and the robot start walking in the same
//     odom tick.
//   - Generate random goals in [X_MIN, X_MAX] x [Y_MIN, Y_MAX] and send them
//     to bt_navigator via the NavigateToPose action. Manual goals from RViz
//     ("2D Goal Pose") on /move_base_simple/goal still work and interrupt the
//     auto-loop.
//   - On NavigateToPose result (SUCCEEDED / ABORTED / CANCELED): call
//     /reset_world, pause + reset pedsim
//     (/pedestrian_simulator/{paused,reset}; new pedestrians stay frozen at
//     their fresh spawns until motion detection resumes them), then after a
//     short settle delay send the next random goal. This forms the auto-loop
//     the user expects.
//   - Per-attempt 60s timeout: cancel the action, then run the same reset
//     path so a stuck attempt does not deadlock the scenario.
//   - Camera TF map -> camera (rolling-average smoothing) so RViz's camera
//     reference frame keeps following the robot, same as the old node.
//
// See docs/nav2_full_plugin_migration_plan.md §4 for design context.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <robot_localization/srv/set_pose.hpp>

#include <mpc_planner_util/parameters.h>
#include <mpc_planner_util/load_yaml.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <random>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

namespace
{
    constexpr int CAMERA_BUFFER = 10;

    // Random goal range. Matches the legacy goal_publisher.py defaults so
    // the auto-loop behaviour is identical between standalone and full-stack
    // bring-ups.
    constexpr double X_MIN = 25.5;
    constexpr double X_MAX = 25.6;
    constexpr double Y_MIN = 25.5;
    constexpr double Y_MAX = 25.6;
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

        // Manual goals (RViz "2D Goal Pose" / "Nav2 Goal") interrupt the
        // auto-loop -- useful for debugging without restarting the bring-up.
        _goal_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/move_base_simple/goal", qos,
            std::bind(&ScenarioOrchestrator::onGoal, this, std::placeholders::_1));

        _odom_sub = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", rclcpp::QoS(5),
            std::bind(&ScenarioOrchestrator::onOdom, this, std::placeholders::_1));

        std::random_device rd;
        _rng.seed(rd());

        _collision_sub = create_subscription<std_msgs::msg::Float64>(
            "/feedback/collisions", qos,
            std::bind(&ScenarioOrchestrator::onCollision, this, std::placeholders::_1));

        _ped_horizon_pub = create_publisher<std_msgs::msg::Int32>(
            "/pedestrian_simulator/horizon", qos);
        _ped_integrator_step_pub = create_publisher<std_msgs::msg::Float32>(
            "/pedestrian_simulator/integrator_step", qos);
        _ped_clock_frequency_pub = create_publisher<std_msgs::msg::Float32>(
            "/pedestrian_simulator/clock_frequency", qos);
        // /pedestrian_simulator/reset is what the ROS2 pedsim node actually
        // subscribes to (calls Reset(): re-randomizes start, goal, velocity
        // for each pedestrian -> brand new scenario). The legacy ROS1 topic
        // /lmpcc/reset_environment has no subscriber in the ROS2 build, so
        // publishing there silently dropped every reset request.
        _reset_simulation_pub = create_publisher<std_msgs::msg::Empty>(
            "/pedestrian_simulator/reset", qos);
        // Pause/resume control. Pedsim is started immediately at startup so
        // /pedestrian_simulator/trajectory_predictions is flowing (the MPC's
        // isDataReady check requires obstacles before it will produce a
        // cmd_vel; without that it never commands the robot, never crosses
        // the motion threshold, and the auto-loop deadlocks). Pedsim is
        // launched paused (Loop publishes predictions but skips the physical
        // advance), and we unpause from onOdom() the moment the robot
        // actually starts rolling -- so pedestrians and the robot begin
        // walking in the same odom tick.
        _ped_paused_pub = create_publisher<std_msgs::msg::Bool>(
            "/pedestrian_simulator/paused",
            rclcpp::QoS(1).transient_local());

        // /goal_pose is the canonical Nav2 goal topic. The orchestrator sends
        // goals via the NavigateToPose action, but publishing them here too
        // lets RViz's NavGoal Pose display visualise the current target.
        _goal_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", qos);

        _ped_start_client = create_client<std_srvs::srv::Empty>(
            "/pedestrian_simulator/start");
        // Use /reset_world (NOT /gazebo/reset_world). Both exist but only
        // /reset_world actually teleports entities back to their spawn
        // poses; /gazebo/reset_world is a no-op on robot position in this
        // gazebo_ros build, so the auto-loop kept "resetting" without the
        // robot moving.
        _gazebo_reset_client = create_client<std_srvs::srv::Empty>(
            "/reset_world");
        // EKF's /set_pose is required after /gazebo/reset_world: gazebo
        // teleports the entity but the EKF keeps integrating its previous
        // estimate, so without this call Nav2 sees the robot still near the
        // last goal pose even though Gazebo says origin. The robot then
        // doesn't actually drive on the next auto-loop attempt.
        _ekf_reset_client = create_client<robot_localization::srv::SetPose>(
            "/set_pose");

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

        // Run pedsim configure handshake on a one-shot timer so this
        // constructor stays non-blocking (the service_wait below sleeps up
        // to ~20*200ms). startPedsim() is deferred to checkInitialKickoff so
        // pedestrians and robot start moving together.
        _startup_timer = create_wall_timer(
            100ms, [this]() {
                _startup_timer->cancel();
                configurePedsim();
            });

        // Poll for the conditions to fire the first auto goal: odometry
        // received AND navigate_to_pose action server actually ready to
        // accept goals (i.e. bt_navigator lifecycle-active). Sending the
        // first goal earlier gets it rejected and wastes the first 60s on
        // the per-attempt timeout fallback before the loop catches up.
        _kickoff_timer = create_wall_timer(
            500ms, std::bind(&ScenarioOrchestrator::checkInitialKickoff, this));
    }

    void checkInitialKickoff()
    {
        if (_initial_goal_sent)
        {
            _kickoff_timer->cancel();
            return;
        }
        if (!_have_odom)
            return;
        if (!_navigate_client->action_server_is_ready())
            return;
        if (!_pedsim_configured)
            return;  // wait for configurePedsim() to finish handshake
        _initial_goal_sent = true;
        _kickoff_timer->cancel();
        RCLCPP_INFO(get_logger(),
                    "Lifecycle active and odom received; sending first goal "
                    "(pedsim paused until robot motion is observed)");
        // Pedsim is already started (configurePedsim called startPedsim) but
        // is paused -- pedestrians don't physically advance until onOdom()
        // sees the robot crossing the motion threshold and resumes pedsim.
        _waiting_for_robot_motion = true;
        sendRandomGoal();
    }

private:
    // Configure pedsim (horizon / integrator_step / clock_frequency), put
    // it in paused mode (so its Loop publishes predictions but pedestrians
    // do not physically advance), and call the start service so the Loop
    // timer actually runs. Without start, pedsim never publishes
    // trajectory_predictions and the MPC blocks on missing obstacles. The
    // pause is lifted from onOdom() the first time the robot crosses the
    // motion threshold for the current goal, so pedestrians and the robot
    // start walking in the same odom tick.
    void configurePedsim()
    {
        RCLCPP_INFO(get_logger(), "Configuring pedestrian simulator");
        // Publish paused=true with transient_local QoS so pedsim sees it on
        // its first subscribe (the Bool subscriber comes up after this
        // publisher in launch ordering otherwise the message is dropped).
        pausePedsim();
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
                RCLCPP_INFO(get_logger(), "Pedestrian simulator configured (paused; will start service)");
                _pedsim_configured = true;
                startPedsim();
                return;
            }
            rclcpp::sleep_for(1s);
        }
        RCLCPP_WARN(get_logger(),
                    "Pedestrian simulator did not respond in 20 attempts");
    }

    // Call /pedestrian_simulator/start so pedsim's update timer begins
    // running. Pedestrians don't physically advance while paused -- but
    // the Loop runs and predictions get published, which is what the MPC
    // needs to pass its data-ready check.
    void startPedsim()
    {
        if (_pedsim_started)
            return;
        if (!_ped_start_client->service_is_ready())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "/pedestrian_simulator/start not ready yet");
            return;
        }
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        _ped_start_client->async_send_request(request);
        _pedsim_started = true;
        RCLCPP_INFO(get_logger(),
                    "Pedestrian simulator started (paused -- predictions only)");
    }

    void pausePedsim()
    {
        std_msgs::msg::Bool b;
        b.data = true;
        _ped_paused_pub->publish(b);
        _pedsim_paused = true;
    }

    void resumePedsim()
    {
        std_msgs::msg::Bool b;
        b.data = false;
        _ped_paused_pub->publish(b);
        _pedsim_paused = false;
        RCLCPP_INFO(get_logger(),
                    "Pedestrian simulator resumed in sync with robot motion");
    }

    // Re-randomize pedestrian start/goal/velocity for the next attempt.
    // Pedsim's update timer keeps running across resets, so this just snaps
    // every pedestrian to a fresh spawn. Called immediately at /reset_world
    // time WHILE PAUSED, so the new pedestrians sit at their spawn poses
    // until resumePedsim() fires from motion detection.
    void resetPedsim()
    {
        _reset_simulation_pub->publish(std_msgs::msg::Empty());
        RCLCPP_INFO(get_logger(), "Pedestrian scenario reset (paused)");
    }

    void onGoal(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        // Manual override path -- the auto-loop continues normally after
        // this attempt finishes (or 60s timeout).
        RCLCPP_INFO(get_logger(),
                    "Manual goal received frame=%s position=(%.2f, %.2f)",
                    msg->header.frame_id.c_str(),
                    msg->pose.position.x, msg->pose.position.y);
        // If pedsim is currently paused (e.g. a manual goal arrives during
        // initial bring-up before the first auto goal), arm the motion
        // trigger so pedestrians start walking when the robot does. A
        // mid-run manual goal (pedsim already running) doesn't touch
        // pedsim state.
        if (_pedsim_paused)
            _waiting_for_robot_motion = true;
        sendNavigateGoal(*msg);
    }

    void sendRandomGoal()
    {
        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = now();
        goal.header.frame_id = "map";
        std::uniform_real_distribution<double> x_dist(X_MIN, X_MAX);
        std::uniform_real_distribution<double> y_dist(Y_MIN, Y_MAX);
        goal.pose.position.x = x_dist(_rng);
        goal.pose.position.y = y_dist(_rng);
        goal.pose.orientation.w = 1.0;
        _current_goal_x = goal.pose.position.x;
        _current_goal_y = goal.pose.position.y;
        _current_goal_active = true;
        RCLCPP_INFO(get_logger(), "Auto goal (%.2f, %.2f)",
                    goal.pose.position.x, goal.pose.position.y);
        sendNavigateGoal(goal);
    }

    void sendNavigateGoal(const geometry_msgs::msg::PoseStamped &goal)
    {
        if (_goal_in_flight)
        {
            // Strict serialization: never let a second goal overlap the
            // first. Without this, every new goal preempts the previous,
            // result callbacks chain into resetAndRelaunch, and the system
            // enters a ~3s Begin-navigating -> Aborting loop while the
            // robot crawls forward.
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "sendNavigateGoal called while a goal is in flight; dropping");
            return;
        }

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

        _goal_in_flight = true;
        // _waiting_for_robot_motion is armed by callers that want the next
        // robot motion to trigger a pedsim start/reset. Initial kickoff and
        // post-reset relaunch arm it; ABORT retry does not (pedsim state
        // should survive those); a manual goal arms it only if pedsim has
        // not been started yet.

        auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        opts.goal_response_callback =
            [this, goal_copy = goal](const GoalHandle::SharedPtr &handle) {
                if (handle)
                    return;
                // Goal rejected (typical cause: bt_navigator lifecycle not
                // active yet on first attempt). Clear in-flight flag and
                // schedule a short retry so we don't burn the full 60s
                // per-attempt timeout waiting for nothing.
                RCLCPP_WARN(get_logger(),
                            "NavigateToPose goal rejected; retrying in 3s");
                _goal_in_flight = false;
                if (_timeout_timer)
                    _timeout_timer->cancel();
                _restart_timer = create_wall_timer(3s, [this, goal_copy]() {
                    _restart_timer->cancel();
                    _relaunch_pending = false;
                    sendNavigateGoal(goal_copy);
                });
                _relaunch_pending = true;
            };
        opts.result_callback =
            std::bind(&ScenarioOrchestrator::onNavigateResult, this, std::placeholders::_1);
        _navigate_client->async_send_goal(action_goal, opts);
        _goal_pose_pub->publish(action_goal.pose);

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
        _goal_in_flight = false;
        if (_timeout_timer)
            _timeout_timer->cancel();

        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            // The robot reached the destination -- per user spec, this is
            // the trigger for resetting the world and starting the next
            // random goal. Full resetAndRelaunch (gazebo/reset_world +
            // /lmpcc/reset_environment + 800ms + new goal).
            RCLCPP_INFO(get_logger(), "NavigateToPose succeeded; resetting world");
            resetAndRelaunch();
            break;
        case rclcpp_action::ResultCode::ABORTED:
            // bt_navigator gave up on this attempt (e.g. MPC repeatedly
            // failed the QP § 7-1 grid-path issue). Do NOT teleport the
            // robot back to origin -- that hides forward progress. Just
            // retry from the robot's current pose with a fresh random goal
            // after a short settle delay.
            RCLCPP_WARN(get_logger(),
                        "NavigateToPose aborted; retrying without reset");
            scheduleRetryWithoutReset();
            break;
        case rclcpp_action::ResultCode::CANCELED:
            // Cancel originates from our own per-attempt 60s timeout path
            // (cancelAndReset), which already kicked off resetAndRelaunch.
            // Don't double-schedule here.
            RCLCPP_INFO(get_logger(),
                        "NavigateToPose canceled (timeout path already resetting)");
            break;
        default:
            RCLCPP_WARN(get_logger(),
                        "NavigateToPose ended with unknown code %d",
                        static_cast<int>(result.code));
            scheduleRetryWithoutReset();
            break;
        }
    }

    // Retry path without world reset: used on bt_navigator aborts so the
    // robot keeps its current pose and progress.
    void scheduleRetryWithoutReset()
    {
        if (_relaunch_pending)
            return;
        _relaunch_pending = true;
        if (_restart_timer)
            _restart_timer->cancel();
        _restart_timer = create_wall_timer(800ms, [this]() {
            _restart_timer->cancel();
            _relaunch_pending = false;
            _goal_reached_locally = false;
            sendRandomGoal();
        });
    }

    void cancelAndReset()
    {
        _navigate_client->async_cancel_all_goals();
        // resetAndRelaunch() is idempotent (guards on _relaunch_pending), so
        // calling it here covers the edge case where the cancel does not
        // produce a result callback (e.g. the first goal was sent before
        // bt_navigator was lifecycle-active and never got accepted).
        resetAndRelaunch();
    }

    // resetScenario() (gazebo/reset_world + /lmpcc/reset_environment) is
    // async. Sending a new NavigateToPose immediately would race the reset
    // and feed planner_server a stale robot pose. We give Gazebo ~800ms to
    // settle and then send the next random goal -- empirically the same
    // pause goal_publisher.py + JackalPlanner used to get implicitly via
    // the 60s timeout path.
    //
    // Idempotent on purpose: both cancelAndReset() (timeout path) and
    // onNavigateResult() (result callback path) can fire for the same
    // attempt. Without the _relaunch_pending guard each cycle would
    // double-schedule the 800ms timer, producing a tight loop where every
    // new goal preempts the previous one and the robot crawls (~0.03 m/s
    // instead of ~0.5 m/s).
    void resetAndRelaunch()
    {
        if (_relaunch_pending)
            return;
        _relaunch_pending = true;
        resetScenario();
        if (_restart_timer)
            _restart_timer->cancel();
        _restart_timer = create_wall_timer(800ms, [this]() {
            _restart_timer->cancel();
            _relaunch_pending = false;
            _goal_reached_locally = false;
            // Arm motion-based pedsim re-randomization for the new attempt.
            _waiting_for_robot_motion = true;
            sendRandomGoal();
        });
    }

    void resetScenario()
    {
        std::lock_guard<std::mutex> lock(_reset_mutex);
        if (_gazebo_reset_client->wait_for_service(50ms))
        {
            auto request = std::make_shared<std_srvs::srv::Empty::Request>();
            _gazebo_reset_client->async_send_request(request);
        }
        // Note: an earlier draft also called the EKF's /set_pose to snap
        // /odometry/filtered back to origin. In practice that wedged the
        // EKF after the first reset and Nav2 stopped seeing robot motion.
        // /gazebo/reset_world alone is enough: the diff_drive plugin
        // republishes wheel encoders from the post-reset pose so EKF
        // naturally converges, provided this node runs with
        // use_sim_time:=true (set in the launch file).
        //
        // Pedsim sequence: pause -> reset (re-randomize spawns) -> wait for
        // robot motion -> resume. Pausing before the reset means the new
        // pedestrian spawns sit still until resumePedsim() fires from
        // onOdom() motion detection, even though pedsim's update timer is
        // still ticking. resetPedsim() keeps publishing predictions for
        // the MPC's data-ready check.
        pausePedsim();
        resetPedsim();

        for (int i = 0; i < CAMERA_BUFFER; ++i)
        {
            _x_buffer[i] = 0.;
            _y_buffer[i] = 0.;
        }
    }

    void onOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        // Mark that odometry is flowing. The actual first-goal kickoff
        // happens in checkInitialKickoff() once the Nav2 lifecycle has
        // finished activating -- sending earlier gets the goal rejected and
        // wastes the first 60s on the timeout fallback.
        _have_odom = true;

        // Motion-based pedsim resume: an upstream caller armed
        // _waiting_for_robot_motion before the goal was dispatched. Hold
        // pedsim paused until the robot's measured linear speed exceeds
        // MOTION_TRIGGER_SPEED, then unpause -- so pedestrians and the
        // robot begin walking in (almost) the same odom tick instead of
        // pedestrians getting a ~1-2s head start while Nav2 spins up its
        // first cmd_vel for the new goal. Pedsim itself was already started
        // (and possibly reset) before the goal was dispatched.
        if (_waiting_for_robot_motion)
        {
            constexpr double MOTION_TRIGGER_SPEED = 0.05;  // m/s
            const double vx = msg->twist.twist.linear.x;
            const double vy = msg->twist.twist.linear.y;
            if (vx * vx + vy * vy > MOTION_TRIGGER_SPEED * MOTION_TRIGGER_SPEED)
            {
                _waiting_for_robot_motion = false;
                resumePedsim();
            }
        }

        // Independent goal-reached detection: orchestrator measures the
        // straight-line distance from the robot's current pose to the
        // active goal. When inside 2m we trigger the reset path ourselves
        // because bt_navigator's SUCCEEDED result rarely actually reaches
        // the action client -- the goal gets preempted by our own
        // ABORTED-retry path before bt_navigator can finish sending it.
        // This sidesteps that race entirely.
        if (_current_goal_active && !_goal_reached_locally)
        {
            const double dx = msg->pose.pose.position.x - _current_goal_x;
            const double dy = msg->pose.pose.position.y - _current_goal_y;
            if (dx * dx + dy * dy < 2.0 * 2.0)
            {
                _goal_reached_locally = true;
                RCLCPP_INFO(get_logger(),
                            "Robot within 2m of goal (%.2f, %.2f); resetting world",
                            _current_goal_x, _current_goal_y);
                _navigate_client->async_cancel_all_goals();
                _current_goal_active = false;
                resetAndRelaunch();
            }
        }

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
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _ped_paused_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _goal_pose_pub;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _ped_start_client;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _gazebo_reset_client;
    rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr _ekf_reset_client;

    rclcpp_action::Client<NavigateToPose>::SharedPtr _navigate_client;

    rclcpp::TimerBase::SharedPtr _startup_timer;
    rclcpp::TimerBase::SharedPtr _timeout_timer;
    rclcpp::TimerBase::SharedPtr _restart_timer;
    rclcpp::TimerBase::SharedPtr _kickoff_timer;

    std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
    rclcpp::Time _prev_camera_stamp;
    double _x_buffer[CAMERA_BUFFER]{};
    double _y_buffer[CAMERA_BUFFER]{};

    double _timeout_seconds{60.0};
    std::mutex _reset_mutex;

    std::mt19937 _rng;
    bool _initial_goal_sent{false};
    bool _relaunch_pending{false};
    bool _goal_in_flight{false};
    bool _have_odom{false};
    bool _pedsim_configured{false};
    bool _pedsim_started{false};
    // Mirrors the last paused/resumed value we published. configurePedsim
    // pauses on startup, every reset cycle pauses again, and motion
    // detection resumes.
    bool _pedsim_paused{true};
    // Set true in checkInitialKickoff / resetAndRelaunch (and manual goal
    // when pedsim is paused). Cleared in onOdom() once the robot crosses
    // the motion threshold, at which point resumePedsim() fires. Gates
    // pedestrian motion so they never walk in front of a stationary robot
    // during Nav2's first-cmd latency.
    bool _waiting_for_robot_motion{false};

    // Independent goal-reached detection (bypasses the bt_navigator action
    // result race). _current_goal_{x,y} tracks the active goal target;
    // _current_goal_active is true between sendRandomGoal and reset;
    // _goal_reached_locally guards against firing the reset path twice for
    // the same goal during the brief window while the action is being
    // canceled.
    double _current_goal_x{0.0};
    double _current_goal_y{0.0};
    bool _current_goal_active{false};
    bool _goal_reached_locally{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScenarioOrchestrator>());
    rclcpp::shutdown();
    return 0;
}
