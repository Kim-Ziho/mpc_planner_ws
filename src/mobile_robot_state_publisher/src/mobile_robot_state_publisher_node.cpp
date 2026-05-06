// ROS 2 port of the original mobile_robot_state_publisher.
//
// Subscribes to Gazebo Classic's /gazebo/link_states (gazebo_msgs/msg/LinkStates),
// extracts the link whose name contains "base_link", and republishes:
//   - nav_msgs/Odometry on /odometry/filtered (ground truth, replacing the EKF)
//   - geometry_msgs/PoseStamped on a configurable robot_state_topic
//   - TF: odom -> base_link (broadcast directly so simulation pose matches RViz)
//   - static TF: odom -> map (the EKF that would normally do this is disabled)

#include <rclcpp/rclcpp.hpp>

#include <gazebo_msgs/msg/link_states.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <memory>
#include <string>

class MobileRobotStatePublisher : public rclcpp::Node
{
public:
    MobileRobotStatePublisher()
        : rclcpp::Node("mobile_robot_state_publisher")
    {
        rate_ = this->declare_parameter<double>("rate", 100.0);
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        root_frame_ = this->declare_parameter<std::string>("root_frame", "map");
        robot_state_topic_ = this->declare_parameter<std::string>("robot_state_topic", "/robot_state");
        vel_state_topic_ = this->declare_parameter<std::string>("vel_state_topic", "/gazebo/link_states");

        last_callback_ = this->now();

        link_sub_ = this->create_subscription<gazebo_msgs::msg::LinkStates>(
            vel_state_topic_, rclcpp::SensorDataQoS(),
            std::bind(&MobileRobotStatePublisher::linkStatesCallback, this, std::placeholders::_1));

        state_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(robot_state_topic_, 1);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 1);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        // Identity odom->map so the EKF's normal map publication isn't needed.
        geometry_msgs::msg::TransformStamped static_tf;
        static_tf.header.stamp = this->now();
        static_tf.header.frame_id = "odom";
        static_tf.child_frame_id = "map";
        static_tf.transform.rotation.w = 1.0;
        static_tf_broadcaster_->sendTransform(static_tf);

        RCLCPP_INFO(this->get_logger(), "mobile_robot_state_publisher ready (rate=%.1f Hz)", rate_);
    }

private:
    void linkStatesCallback(gazebo_msgs::msg::LinkStates::ConstSharedPtr msg)
    {
        const auto stamp = this->now();
        if ((stamp - last_callback_).seconds() < 1.0 / rate_)
            return;
        last_callback_ = stamp;

        // Pick the entry whose name contains base_frame_ (Gazebo prefixes link
        // names with the model namespace, e.g. "jackal::base_link").
        std::size_t index = 0;
        bool found = false;
        for (std::size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i].find(base_frame_) != std::string::npos)
            {
                index = i;
                found = true;
                break;
            }
        }
        if (!found)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Mobile robot state publisher: '%s' link not found in %zu entries",
                                 base_frame_.c_str(), msg->name.size());
            return;
        }

        const auto &pose = msg->pose[index];
        const auto &twist = msg->twist[index];

        // Odometry on the EKF topic so consumers don't need to switch sources.
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = base_frame_;
        odom.pose.pose = pose;
        odom.twist.twist = twist;
        odom_pub_->publish(odom);

        // Direct odom->base_link TF (replaces the disabled EKF transform).
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = "odom";
        tf.child_frame_id = base_frame_;
        tf.transform.translation.x = pose.position.x;
        tf.transform.translation.y = pose.position.y;
        tf.transform.translation.z = pose.position.z;
        tf.transform.rotation = pose.orientation;
        tf_broadcaster_->sendTransform(tf);

        // Compact PoseStamped: orientation slot carries (roll, pitch, yaw) and
        // z-position carries planar speed, matching the legacy contract used
        // by mpc_planner_rosnavigation's /input/state_pose subscriber.
        tf2::Quaternion q(pose.orientation.x, pose.orientation.y,
                          pose.orientation.z, pose.orientation.w);
        tf2::Matrix3x3 rot(q);
        double roll, pitch, yaw;
        rot.getRPY(roll, pitch, yaw);
        if (std::isnan(yaw))
        {
            roll = pitch = yaw = 0.0;
        }

        geometry_msgs::msg::PoseStamped state;
        state.header.stamp = stamp;
        state.header.frame_id = "odom";
        state.pose.position.x = pose.position.x;
        state.pose.position.y = pose.position.y;
        state.pose.position.z = std::sqrt(twist.linear.x * twist.linear.x +
                                          twist.linear.y * twist.linear.y);
        state.pose.orientation.x = roll;
        state.pose.orientation.y = pitch;
        state.pose.orientation.z = yaw;
        state_pub_->publish(state);
    }

    double rate_{100.0};
    std::string base_frame_;
    std::string root_frame_;
    std::string robot_state_topic_;
    std::string vel_state_topic_;

    rclcpp::Time last_callback_;

    rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr link_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr state_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MobileRobotStatePublisher>());
    rclcpp::shutdown();
    return 0;
}
