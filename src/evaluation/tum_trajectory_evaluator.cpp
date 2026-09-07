#include "modular_3d_localizer/evaluation/tum_trajectory_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

#include <Eigen/Geometry>

namespace modular_3d_localizer
{
namespace
{
double toSeconds(const builtin_interfaces::msg::Time &stamp)
{
    return static_cast<double>(stamp.sec) +
           static_cast<double>(stamp.nanosec) * 1e-9;
}
}  // namespace

TumTrajectoryEvaluator::TumTrajectoryEvaluator()
: Node("tum_trajectory_evaluator")
{
    const auto ground_truth_path = declare_parameter<std::string>("ground_truth_path", "");
    const auto pose_topic = declare_parameter<std::string>("pose_topic", "/localization_pose");
    max_time_difference_seconds_ =
        declare_parameter<double>("max_time_difference_seconds", 0.05);

    if (!loadGroundTruth(ground_truth_path))
    {
        RCLCPP_ERROR(get_logger(), "Could not load TUM ground truth from '%s'.",
                     ground_truth_path.c_str());
        return;
    }

    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic, rclcpp::QoS(50),
        std::bind(&TumTrajectoryEvaluator::poseCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Evaluating %s against %zu TUM poses (max timestamp delta: %.3f s).",
                pose_topic.c_str(), ground_truth_.size(), max_time_difference_seconds_);
}

bool TumTrajectoryEvaluator::loadGroundTruth(const std::string &path)
{
    if (path.empty())
    {
        return false;
    }

    std::ifstream file(path);
    if (!file)
    {
        return false;
    }

    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty() || line.front() == '#')
        {
            continue;
        }

        TimedPose timed_pose;
        auto &position = timed_pose.pose.position;
        auto &orientation = timed_pose.pose.orientation;
        std::istringstream stream(line);
        if (!(stream >> timed_pose.timestamp_seconds >> position.x >> position.y >> position.z >>
              orientation.x >> orientation.y >> orientation.z >> orientation.w))
        {
            RCLCPP_WARN(get_logger(), "Ignoring malformed TUM trajectory row: %s", line.c_str());
            continue;
        }
        ground_truth_.push_back(timed_pose);
    }

    std::sort(ground_truth_.begin(), ground_truth_.end(),
              [](const TimedPose &left, const TimedPose &right) {
                  return left.timestamp_seconds < right.timestamp_seconds;
              });
    return !ground_truth_.empty();
}

const TumTrajectoryEvaluator::TimedPose *TumTrajectoryEvaluator::findNearestGroundTruth(
    const double timestamp_seconds) const
{
    const auto after = std::lower_bound(
        ground_truth_.begin(), ground_truth_.end(), timestamp_seconds,
        [](const TimedPose &pose, const double timestamp) {
            return pose.timestamp_seconds < timestamp;
        });

    const TimedPose *nearest = nullptr;
    if (after != ground_truth_.end())
    {
        nearest = &*after;
    }
    if (after != ground_truth_.begin())
    {
        const auto before = std::prev(after);
        if (!nearest || std::abs(before->timestamp_seconds - timestamp_seconds) <
                            std::abs(nearest->timestamp_seconds - timestamp_seconds))
        {
            nearest = &*before;
        }
    }
    if (!nearest ||
        std::abs(nearest->timestamp_seconds - timestamp_seconds) > max_time_difference_seconds_)
    {
        return nullptr;
    }
    return nearest;
}

void TumTrajectoryEvaluator::poseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr pose_message)
{
    const double timestamp_seconds = toSeconds(pose_message->header.stamp);
    const auto *ground_truth_pose = findNearestGroundTruth(timestamp_seconds);
    if (!ground_truth_pose)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "No TUM ground-truth pose within %.3f s of the estimate.",
                             max_time_difference_seconds_);
        return;
    }

    const auto &estimate = pose_message->pose;
    const auto &reference = ground_truth_pose->pose;
    const double dx = estimate.position.x - reference.position.x;
    const double dy = estimate.position.y - reference.position.y;
    const double dz = estimate.position.z - reference.position.z;
    const double translation_error = std::sqrt(dx * dx + dy * dy + dz * dz);

    Eigen::Quaterniond estimate_rotation(
        estimate.orientation.w, estimate.orientation.x, estimate.orientation.y,
        estimate.orientation.z);
    Eigen::Quaterniond reference_rotation(
        reference.orientation.w, reference.orientation.x, reference.orientation.y,
        reference.orientation.z);
    if (estimate_rotation.norm() == 0.0 || reference_rotation.norm() == 0.0)
    {
        RCLCPP_WARN(get_logger(), "Ignoring pose with a zero orientation quaternion.");
        return;
    }
    estimate_rotation.normalize();
    reference_rotation.normalize();
    const double rotation_error = 2.0 * std::acos(std::clamp(
        std::abs(estimate_rotation.dot(reference_rotation)), 0.0, 1.0));

    ++matched_pose_count_;
    translation_squared_error_sum_ += translation_error * translation_error;
    rotation_squared_error_sum_ += rotation_error * rotation_error;

    const double translation_rmse =
        std::sqrt(translation_squared_error_sum_ / matched_pose_count_);
    const double rotation_rmse =
        std::sqrt(rotation_squared_error_sum_ / matched_pose_count_);
    RCLCPP_INFO(get_logger(),
                "GT match %zu: translation_error=%.3f m, rotation_error=%.3f rad, "
                "RMSE=[%.3f m, %.3f rad].",
                matched_pose_count_, translation_error, rotation_error,
                translation_rmse, rotation_rmse);
}
}  // namespace modular_3d_localizer
