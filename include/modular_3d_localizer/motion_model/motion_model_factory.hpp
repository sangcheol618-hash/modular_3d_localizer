#pragma once

#include <memory>
#include <string>

#include "modular_3d_localizer/motion_model/motion_model.hpp"
#include "modular_3d_localizer/motion_model/odometry_motion_model.hpp"

namespace modular_3d_localizer
{
std::unique_ptr<MotionModel> createMotionModel(
    const std::string &motion_model_name,
    const OdometryMotionModelConfig &odometry_config);
}  // namespace modular_3d_localizer
