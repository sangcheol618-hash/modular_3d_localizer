#pragma once

#include <modular_3d_localizer/registration_backend.hpp>
#include <pcl/registration/gicp.h>

namespace modular_3d_localizer
{
    struct GicpRegistrationConfig
    {
        int max_iterations{20};
        double max_correspondence_distance{3.0};
        double transformation_epsilon{1e-4};
        double euclidean_fitness_epsilon{1e-3};
        int correspondence_randomness{10};
    };

    class GicpRegistration : public RegistrationBackend
    {
        public:
            explicit GicpRegistration(const GicpRegistrationConfig &config);
            ~GicpRegistration() override = default;

            void setTarget(MapLoader::PointCloud::ConstPtr target_cloud) override;
            RegistrationResult align(
                MapLoader::PointCloud::ConstPtr source_cloud,
                const Eigen::Matrix4f &initial_guess) override;

        private:
            MapLoader::PointCloud::ConstPtr target_cloud_;
            pcl::GeneralizedIterativeClosestPoint<MapLoader::PointType, MapLoader::PointType> gicp_;
    };
} // namespace modular_3d_localizer
