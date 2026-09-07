#pragma once

#include <limits>

#include <Eigen/Core>
#include <modular_3d_localizer/map_loader.hpp>

namespace modular_3d_localizer
{
    struct RegistrationResult
    {
        bool converged{false};
        Eigen::Matrix4f transform{Eigen::Matrix4f::Identity()};
        double fitness_score{std::numeric_limits<double>::infinity()};
    };

    class RegistrationBackend
    {
    public:
        virtual ~RegistrationBackend() = default;

        virtual void setTarget(MapLoader::PointCloud::ConstPtr target_cloud) = 0;
        virtual RegistrationResult align(
            MapLoader::PointCloud::ConstPtr source_cloud,
            const Eigen::Matrix4f &initial_guess) = 0;
    };
}  // namespace modular_3d_localizer
