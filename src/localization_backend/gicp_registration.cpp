#include "modular_3d_localizer/localization_backend/gicp_registration.hpp"

namespace modular_3d_localizer
{
    GicpRegistration::GicpRegistration(const GicpRegistrationConfig &config)
    {
        gicp_.setMaximumIterations(config.max_iterations);
        gicp_.setMaxCorrespondenceDistance(config.max_correspondence_distance);
        gicp_.setTransformationEpsilon(config.transformation_epsilon);
        gicp_.setEuclideanFitnessEpsilon(config.euclidean_fitness_epsilon);
        gicp_.setCorrespondenceRandomness(config.correspondence_randomness);
    }

    void GicpRegistration::setTarget(MapLoader::PointCloud::ConstPtr target_cloud)
    {
        target_cloud_ = target_cloud;
        gicp_.setInputTarget(target_cloud_);
    }

    RegistrationResult GicpRegistration::align(
        MapLoader::PointCloud::ConstPtr source_cloud,
        const Eigen::Matrix4f &initial_guess)
    {
        RegistrationResult result;
        gicp_.setInputSource(source_cloud);
        MapLoader::PointCloud aligned_cloud;
        gicp_.align(aligned_cloud, initial_guess);

        result.converged = gicp_.hasConverged();
        if (result.converged)
        {
            result.transform = gicp_.getFinalTransformation();
            result.fitness_score = gicp_.getFitnessScore();
        }
        return result;
    }
} // namespace modular_3d_localizer
