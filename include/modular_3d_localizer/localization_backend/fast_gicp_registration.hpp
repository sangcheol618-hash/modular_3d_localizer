#pragma once

#include "modular_3d_localizer/registration_backend.hpp"

#include <fast_gicp/gicp/fast_gicp.hpp>

namespace modular_3d_localizer
{
struct FastGicpRegistrationConfig
{
    int max_iterations{20};
    double max_correspondence_distance{3.0};
    double transformation_epsilon{1e-4};
    int correspondence_randomness{10};
    int num_threads{0};  // 0 lets fast_gicp choose the OpenMP thread count.
};

class FastGicpRegistration : public RegistrationBackend
{
public:
    explicit FastGicpRegistration(const FastGicpRegistrationConfig &config);
    ~FastGicpRegistration() override = default;

    void setTarget(MapLoader::PointCloud::ConstPtr target_cloud) override;
    RegistrationResult align(
        MapLoader::PointCloud::ConstPtr source_cloud,
        const Eigen::Matrix4f &initial_guess) override;

private:
    fast_gicp::FastGICP<MapLoader::PointType, MapLoader::PointType> fast_gicp_;
};

}  // namespace modular_3d_localizer
