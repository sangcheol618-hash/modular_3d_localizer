#pragma once

#include "modular_3d_localizer/localization_backend/gicp_registration.hpp"
#include "modular_3d_localizer/registration_backend.hpp"

namespace modular_3d_localizer
{
struct CoarseToFineGicpRegistrationConfig
{
    double coarse_voxel_leaf_size{2.0};
    double coarse_max_correspondence_distance{5.0};
    int coarse_max_iterations{8};
    GicpRegistrationConfig fine_config{};
};

// Runs a low-resolution GICP stage to improve the prior, then refines that
// result against the supplied target cloud at its original resolution.
class CoarseToFineGicpRegistration : public RegistrationBackend
{
public:
    explicit CoarseToFineGicpRegistration(
        const CoarseToFineGicpRegistrationConfig &config);

    void setTarget(MapLoader::PointCloud::ConstPtr target_cloud) override;
    RegistrationResult align(
        MapLoader::PointCloud::ConstPtr source_cloud,
        const Eigen::Matrix4f &initial_guess) override;

private:
    CoarseToFineGicpRegistrationConfig config_;
    MapLoader::PointCloud::ConstPtr target_cloud_;
    GicpRegistration coarse_registration_;
    GicpRegistration fine_registration_;
};
}  // namespace modular_3d_localizer
