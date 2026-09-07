#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <string>

namespace modular_3d_localizer
{
    class InitialPosePublisher : public rclcpp::Node
    {
    public:
        InitialPosePublisher();

    private:
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
            initial_pose_pub_;
    };
}  // namespace modular_3d_localizer
