#include "modular_3d_localizer/localization_backend/coarse_to_fine_gicp_registration.hpp"

#include <algorithm>

#include "modular_3d_localizer/point_cloud_preprocessor.hpp"

namespace modular_3d_localizer
{
namespace
{
GicpRegistrationConfig makeCoarseConfig(
    const CoarseToFineGicpRegistrationConfig &config)
{
    auto coarse_config = config.fine_config;
    coarse_config.max_iterations = config.coarse_max_iterations;
    coarse_config.max_correspondence_distance =
        config.coarse_max_correspondence_distance;
    return coarse_config;
}

MapLoader::PointCloud::ConstPtr downsample(
    const MapLoader::PointCloud::ConstPtr &cloud, const double leaf_size)
{
    PointCloudPreprocessor preprocessor;
    PointCloudPreprocessorConfig config;
    config.voxel_leaf_size = leaf_size;
    config.auto_adjust_voxel_leaf_size = false;
    return preprocessor.process(cloud, config).cloud;
}
}  // namespace

CoarseToFineGicpRegistration::CoarseToFineGicpRegistration(
    const CoarseToFineGicpRegistrationConfig &config)
: config_(config),
  coarse_registration_(makeCoarseConfig(config)),
  fine_registration_(config.fine_config)
{
}

void CoarseToFineGicpRegistration::setTarget(
    MapLoader::PointCloud::ConstPtr target_cloud)
{
    target_cloud_ = target_cloud;
    fine_registration_.setTarget(target_cloud_);
    coarse_registration_.setTarget(downsample(
        target_cloud_, std::max(config_.coarse_voxel_leaf_size, 0.0)));
}

RegistrationResult CoarseToFineGicpRegistration::align(
    MapLoader::PointCloud::ConstPtr source_cloud,
    const Eigen::Matrix4f &initial_guess)
{
    if (!source_cloud || source_cloud->empty() || !target_cloud_ || target_cloud_->empty())
    {
        return {};
    }

    const auto coarse_source = downsample(
        source_cloud, std::max(config_.coarse_voxel_leaf_size, 0.0));
    const auto coarse_result = coarse_registration_.align(coarse_source, initial_guess);
    if (!coarse_result.converged)
    {
        return coarse_result;
    }

    return fine_registration_.align(source_cloud, coarse_result.transform);
}
}  // namespace modular_3d_localizer
