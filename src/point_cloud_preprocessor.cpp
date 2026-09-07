#include "modular_3d_localizer/point_cloud_preprocessor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

namespace modular_3d_localizer
{

PointCloudPreprocessResult PointCloudPreprocessor::process(
    MapLoader::PointCloud::ConstPtr input_cloud,
    const PointCloudPreprocessorConfig &config) const
{
    PointCloudPreprocessResult result;
    result.cloud = input_cloud;

    if (!input_cloud || input_cloud->empty())
    {
        return result;
    }

    const bool needs_bounds = config.voxel_leaf_size > 0.0 &&
        config.auto_adjust_voxel_leaf_size;

    if (needs_bounds)
    {
        pcl::getMinMax3D(
            *input_cloud, result.minimum_point, result.maximum_point);
        result.has_bounds = true;
    }

    if (config.voxel_leaf_size > 0.0)
    {
        result.effective_voxel_leaf_size = config.voxel_leaf_size;

        if (config.auto_adjust_voxel_leaf_size)
        {
            const double span_x = result.maximum_point.x - result.minimum_point.x;
            const double span_y = result.maximum_point.y - result.minimum_point.y;
            const double span_z = result.maximum_point.z - result.minimum_point.z;
            const double minimum_safe_leaf_size = 1.05 * std::cbrt(
                (span_x * span_y * span_z) /
                static_cast<double>(std::numeric_limits<std::int32_t>::max()));
            result.effective_voxel_leaf_size = std::max(
                config.voxel_leaf_size, minimum_safe_leaf_size);
        }

        auto downsampled_cloud = std::make_shared<MapLoader::PointCloud>();
        pcl::VoxelGrid<MapLoader::PointType> voxel_filter;
        voxel_filter.setInputCloud(input_cloud);
        voxel_filter.setLeafSize(
            static_cast<float>(result.effective_voxel_leaf_size),
            static_cast<float>(result.effective_voxel_leaf_size),
            static_cast<float>(result.effective_voxel_leaf_size));
        voxel_filter.filter(*downsampled_cloud);
        result.cloud = downsampled_cloud;
    }

    return result;
}

}  // namespace modular_3d_localizer
