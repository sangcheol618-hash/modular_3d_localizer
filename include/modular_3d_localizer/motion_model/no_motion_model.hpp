#pragma once

#include "modular_3d_localizer/motion_model/motion_model.hpp"

namespace modular_3d_localizer
{
class NoMotionModel : public MotionModel
{
public:
    void resetReference() override {}

    bool predict(
        const Eigen::Matrix4f &,
        const Eigen::Matrix4f &,
        const builtin_interfaces::msg::Time &,
        Eigen::Matrix4f &) override
    {
        return false;
    }
};
}  // namespace modular_3d_localizer
