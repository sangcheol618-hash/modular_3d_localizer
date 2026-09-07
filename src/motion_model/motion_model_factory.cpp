#include "modular_3d_localizer/motion_model/motion_model_factory.hpp"

#include "modular_3d_localizer/motion_model/no_motion_model.hpp"

namespace modular_3d_localizer
{
std::unique_ptr<MotionModel> createMotionModel(
    const std::string &motion_model_name,
    const OdometryMotionModelConfig &odometry_config)
{
    if (motion_model_name == "none")
    {
        return std::make_unique<NoMotionModel>();
    }
    if (motion_model_name == "odometry")
    {
        return std::make_unique<OdometryMotionModel>(odometry_config);
    }

    return nullptr;
}
}  // namespace modular_3d_localizer
