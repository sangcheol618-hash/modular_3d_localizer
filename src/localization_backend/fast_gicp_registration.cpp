#include "modular_3d_localizer/localization_backend/fast_gicp_registration.hpp"

namespace modular_3d_localizer
{
FastGicpRegistration::FastGicpRegistration(const FastGicpRegistrationConfig &config)
{
    fast_gicp_.setMaximumIterations(config.max_iterations);
    fast_gicp_.setMaxCorrespondenceDistance(config.max_correspondence_distance);
    fast_gicp_.setTransformationEpsilon(config.transformation_epsilon);
    fast_gicp_.setCorrespondenceRandomness(config.correspondence_randomness);
    fast_gicp_.setNumThreads(config.num_threads);
}

void FastGicpRegistration::setTarget(MapLoader::PointCloud::ConstPtr target_cloud)
{
    fast_gicp_.setInputTarget(target_cloud);
}

RegistrationResult FastGicpRegistration::align(
    MapLoader::PointCloud::ConstPtr source_cloud,
    const Eigen::Matrix4f &initial_guess)
{
    RegistrationResult result;
    fast_gicp_.setInputSource(source_cloud);
    MapLoader::PointCloud aligned_cloud;
    fast_gicp_.align(aligned_cloud, initial_guess);

    result.converged = fast_gicp_.hasConverged();
    if (result.converged)
    {
        result.transform = fast_gicp_.getFinalTransformation();
        result.fitness_score = fast_gicp_.getFitnessScore();
    }
    return result;
}
}  // namespace modular_3d_localizer
