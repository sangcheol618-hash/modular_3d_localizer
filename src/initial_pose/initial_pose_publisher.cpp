#include "modular_3d_localizer/initial_pose/initial_pose_publisher.hpp"

#include <tf2/LinearMath/Quaternion.h>

namespace modular_3d_localizer
{
InitialPosePublisher::InitialPosePublisher()
: Node("initial_pose_publisher")
{
    const auto initial_pose_topic =
        declare_parameter<std::string>("initial_pose_topic", "/initialpose");
    const auto map_frame = declare_parameter<std::string>("map_frame", "map");
    const auto base_frame = declare_parameter<std::string>("base_frame", "base_link");
    const auto x = declare_parameter<double>("x", 0.0);
    const auto y = declare_parameter<double>("y", 0.0);
    const auto z = declare_parameter<double>("z", 0.0);
    const auto roll = declare_parameter<double>("roll", 0.0);
    const auto pitch = declare_parameter<double>("pitch", 0.0);
    const auto yaw = declare_parameter<double>("yaw", 0.0);

    rclcpp::QoS qos(1);
    qos.reliable();
    qos.transient_local();
    initial_pose_pub_ =
        create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            initial_pose_topic, qos);

    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.stamp = now();
    message.header.frame_id = map_frame;
    message.pose.pose.position.x = x;
    message.pose.pose.position.y = y;
    message.pose.pose.position.z = z;

    tf2::Quaternion orientation;
    orientation.setRPY(roll, pitch, yaw);
    message.pose.pose.orientation.x = orientation.x();
    message.pose.pose.orientation.y = orientation.y();
    message.pose.pose.orientation.z = orientation.z();
    message.pose.pose.orientation.w = orientation.w();

    initial_pose_pub_->publish(message);
    RCLCPP_INFO(
        get_logger(),
        "Published initial map-to-%s pose on %s: position=[%.3f, %.3f, %.3f], "
        "RPY=[%.3f, %.3f, %.3f] rad.",
        base_frame.c_str(), initial_pose_topic.c_str(), x, y, z, roll, pitch, yaw);
}
}  // namespace modular_3d_localizer
