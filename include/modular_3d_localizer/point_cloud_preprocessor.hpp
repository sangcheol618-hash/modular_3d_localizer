#pragma once

#include "modular_3d_localizer/map_loader.hpp"

namespace modular_3d_localizer
{

struct PointCloudPreprocessorConfig
{
    double voxel_leaf_size{0.2};
    bool auto_adjust_voxel_leaf_size{false};
};

struct PointCloudPreprocessResult
{
    MapLoader::PointCloud::ConstPtr cloud;
    bool has_bounds{false};
    MapLoader::PointType minimum_point;
    MapLoader::PointType maximum_point;
    double effective_voxel_leaf_size{0.0};
};

class PointCloudPreprocessor
{
public:
    PointCloudPreprocessResult process(
        MapLoader::PointCloud::ConstPtr input_cloud,
        const PointCloudPreprocessorConfig &config) const;
};

}  // namespace modular_3d_localizer
