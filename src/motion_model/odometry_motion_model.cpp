#include "modular_3d_localizer/motion_model/odometry_motion_model.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

namespace modular_3d_localizer
{
namespace
{
double secondsBetween(const builtin_interfaces::msg::Time &later,
                      const builtin_interfaces::msg::Time &earlier)
{
    return static_cast<double>(later.sec) - static_cast<double>(earlier.sec) +
           (static_cast<double>(later.nanosec) - static_cast<double>(earlier.nanosec)) * 1e-9;
}
}  // namespace

OdometryMotionModel::OdometryMotionModel(const OdometryMotionModelConfig &config)
: config_(config)
{
}

bool OdometryMotionModel::update(const nav_msgs::msg::Odometry &odometry)
{
    if ((!odometry.header.frame_id.empty() && odometry.header.frame_id != config_.odom_frame) ||
        (!odometry.child_frame_id.empty() && odometry.child_frame_id != config_.base_frame))
    {
        return false;
    }

    const auto &position = odometry.pose.pose.position;
    const auto &orientation = odometry.pose.pose.orientation;
    Eigen::Quaternionf rotation(static_cast<float>(orientation.w),
                                static_cast<float>(orientation.x),
                                static_cast<float>(orientation.y),
                                static_cast<float>(orientation.z));
    if (rotation.norm() == 0.0F)
    {
        return false;
    }

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3, 3>(0, 0) = rotation.normalized().toRotationMatrix();
    pose(0, 3) = static_cast<float>(position.x);
    pose(1, 3) = static_cast<float>(position.y);
    pose(2, 3) = static_cast<float>(position.z);

    std::lock_guard<std::mutex> lock(mutex_);
    if (!pose_history_.empty())
    {
        const double delta = secondsBetween(odometry.header.stamp, pose_history_.back().stamp);
        if (delta < 0.0)
        {
            return false;
        }
        if (delta == 0.0)
        {
            pose_history_.back().pose = pose;
        }
        else
        {
            pose_history_.push_back(TimedPose{odometry.header.stamp, pose});
        }
    }
    else
    {
        pose_history_.push_back(TimedPose{odometry.header.stamp, pose});
    }
    pruneHistory();
    if (!reference_pose_odom_base_.has_value())
    {
        reference_pose_odom_base_ = pose;
    }
    return true;
}

bool OdometryMotionModel::getLatestPose(const builtin_interfaces::msg::Time &stamp,
                                        Eigen::Matrix4f &pose_odom_base) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (pose_history_.empty())
    {
        return false;
    }
    const auto &latest = pose_history_.back();
    if ((stamp.sec != 0 || stamp.nanosec != 0) &&
        (secondsBetween(stamp, latest.stamp) < 0.0 ||
         !isAgeAcceptable(std::abs(secondsBetween(stamp, latest.stamp)))))
    {
        return false;
    }
    pose_odom_base = latest.pose;
    return true;
}

bool OdometryMotionModel::getPoseAt(const builtin_interfaces::msg::Time &stamp,
                                    Eigen::Matrix4f &pose_odom_base) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (pose_history_.empty())
    {
        return false;
    }
    const auto after = std::upper_bound(
        pose_history_.begin(), pose_history_.end(), stamp,
        [](const builtin_interfaces::msg::Time &time, const TimedPose &entry) {
            return secondsBetween(time, entry.stamp) < 0.0;
        });
    if (after == pose_history_.begin())
    {
        return false;
    }

    const auto before = std::prev(after);
    const double age = secondsBetween(stamp, before->stamp);
    if (!isAgeAcceptable(age))
    {
        return false;
    }
    if (after == pose_history_.end())
    {
        pose_odom_base = before->pose;
        return true;
    }

    const double interval = secondsBetween(after->stamp, before->stamp);
    if (interval <= 0.0)
    {
        pose_odom_base = before->pose;
        return true;
    }
    const float ratio = static_cast<float>(std::clamp(age / interval, 0.0, 1.0));
    pose_odom_base = Eigen::Matrix4f::Identity();
    pose_odom_base.block<3, 1>(0, 3) =
        (1.0F - ratio) * before->pose.block<3, 1>(0, 3) +
        ratio * after->pose.block<3, 1>(0, 3);
    const Eigen::Quaternionf before_rotation(before->pose.block<3, 3>(0, 0));
    const Eigen::Quaternionf after_rotation(after->pose.block<3, 3>(0, 0));
    pose_odom_base.block<3, 3>(0, 0) =
        before_rotation.slerp(ratio, after_rotation).normalized().toRotationMatrix();
    return true;
}

void OdometryMotionModel::resetReference()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pose_history_.empty())
    {
        reference_pose_odom_base_ = pose_history_.back().pose;
    }
}

bool OdometryMotionModel::resetReferenceAt(const builtin_interfaces::msg::Time &stamp)
{
    Eigen::Matrix4f pose;
    if (!getPoseAt(stamp, pose))
    {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    reference_pose_odom_base_ = pose;
    return true;
}

bool OdometryMotionModel::predict(const Eigen::Matrix4f &current_pose_map_lidar,
                                  const Eigen::Matrix4f &base_to_lidar,
                                  const builtin_interfaces::msg::Time &scan_stamp,
                                  Eigen::Matrix4f &predicted_pose_map_lidar)
{
    Eigen::Matrix4f pose_odom_base;
    if (!getPoseAt(scan_stamp, pose_odom_base))
    {
        return false;
    }
    Eigen::Matrix4f reference_pose;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!reference_pose_odom_base_.has_value())
        {
            return false;
        }
        reference_pose = *reference_pose_odom_base_;
    }
    const Eigen::Matrix4f current_pose_map_base =
        current_pose_map_lidar * base_to_lidar.inverse();
    predicted_pose_map_lidar = current_pose_map_base *
        reference_pose.inverse() * pose_odom_base * base_to_lidar;
    return true;
}

bool OdometryMotionModel::isAgeAcceptable(const double age_seconds) const
{
    return age_seconds >= 0.0 &&
           (config_.max_age_seconds <= 0.0 || age_seconds <= config_.max_age_seconds);
}

void OdometryMotionModel::pruneHistory()
{
    if (config_.history_duration_seconds <= 0.0 || pose_history_.empty())
    {
        return;
    }
    const auto newest_stamp = pose_history_.back().stamp;
    while (pose_history_.size() > 1 &&
           secondsBetween(newest_stamp, pose_history_.front().stamp) >
               config_.history_duration_seconds)
    {
        pose_history_.pop_front();
    }
}
}  // namespace modular_3d_localizer
