#pragma once

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace modular_3d_localizer
{
    struct Static_transform
    {
        std::string parent_frame;
        std::string child_frame;
        Eigen::Vector3d translation;
        Eigen::Quaterniond rotation;
    };

}
