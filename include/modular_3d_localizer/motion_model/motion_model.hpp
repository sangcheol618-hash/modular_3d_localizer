#pragma once

#include <Eigen/Core>

#include <builtin_interfaces/msg/time.hpp>

namespace modular_3d_localizer
{
class MotionModel
{
public:
    virtual ~MotionModel() = default;

    // Updates the reference used for the next relative-motion prediction.
    virtual void resetReference() = 0;

    // Returns true only when a prediction is available for this scan.
    virtual bool predict(
        const Eigen::Matrix4f &current_pose_map_lidar,
        const Eigen::Matrix4f &base_to_lidar,
        const builtin_interfaces::msg::Time &scan_stamp,
        Eigen::Matrix4f &predicted_pose_map_lidar) = 0;
};
}  // namespace modular_3d_localizer
