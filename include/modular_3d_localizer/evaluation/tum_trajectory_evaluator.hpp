#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace modular_3d_localizer
{
class TumTrajectoryEvaluator : public rclcpp::Node
{
public:
    TumTrajectoryEvaluator();

private:
    struct TimedPose
    {
        double timestamp_seconds;
        geometry_msgs::msg::Pose pose;
    };

    bool loadGroundTruth(const std::string &path);
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_message);
    const TimedPose *findNearestGroundTruth(double timestamp_seconds) const;

    std::vector<TimedPose> ground_truth_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    double max_time_difference_seconds_{0.05};
    std::size_t matched_pose_count_{0};
    double translation_squared_error_sum_{0.0};
    double rotation_squared_error_sum_{0.0};
};
}  // namespace modular_3d_localizer
