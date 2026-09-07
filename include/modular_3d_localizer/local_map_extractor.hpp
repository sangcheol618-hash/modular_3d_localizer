#pragma once

#include <Eigen/Core>

#include "modular_3d_localizer/map_loader.hpp"

namespace modular_3d_localizer
{
class LocalMapExtractor
{
public:
    MapLoader::PointCloud::ConstPtr extract(
        MapLoader::PointCloud::ConstPtr global_map,
        const Eigen::Vector3f &center,
        float radius) const;
};
}  // namespace modular_3d_localizer
