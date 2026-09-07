#pragma once

#include <deque>
#include <mutex>
#include <optional>
#include <string>

#include <nav_msgs/msg/odometry.hpp>

#include "modular_3d_localizer/motion_model/motion_model.hpp"

namespace modular_3d_localizer
{
struct OdometryMotionModelConfig
{
    std::string odom_frame{"odom"};
    std::string base_frame{"base_link"};
    double max_age_seconds{0.5};
    double history_duration_seconds{10.0};
};

class OdometryMotionModel : public MotionModel
{
public:
    explicit OdometryMotionModel(const OdometryMotionModelConfig &config);

    bool update(const nav_msgs::msg::Odometry &odometry);
    bool getLatestPose(
        const builtin_interfaces::msg::Time &stamp,
        Eigen::Matrix4f &pose_odom_base) const;
    bool getPoseAt(
        const builtin_interfaces::msg::Time &stamp,
        Eigen::Matrix4f &pose_odom_base) const;
    void resetReference() override;
    bool resetReferenceAt(const builtin_interfaces::msg::Time &stamp);
    bool predict(
        const Eigen::Matrix4f &current_pose_map_lidar,
        const Eigen::Matrix4f &base_to_lidar,
        const builtin_interfaces::msg::Time &scan_stamp,
        Eigen::Matrix4f &predicted_pose_map_lidar) override;

private:
    struct TimedPose
    {
        builtin_interfaces::msg::Time stamp;
        Eigen::Matrix4f pose;
    };

    bool isAgeAcceptable(double age_seconds) const;
    void pruneHistory();

    OdometryMotionModelConfig config_;
    mutable std::mutex mutex_;
    std::deque<TimedPose> pose_history_;
    std::optional<Eigen::Matrix4f> reference_pose_odom_base_;
};
}  // namespace modular_3d_localizer
