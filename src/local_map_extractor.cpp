#include "modular_3d_localizer/local_map_extractor.hpp"

#include <pcl/filters/crop_box.h>

namespace modular_3d_localizer
{
MapLoader::PointCloud::ConstPtr LocalMapExtractor::extract(
    MapLoader::PointCloud::ConstPtr global_map,
    const Eigen::Vector3f &center,
    float radius) const
{
    auto local_map = std::make_shared<MapLoader::PointCloud>();
    pcl::CropBox<MapLoader::PointType> crop_box;
    crop_box.setInputCloud(global_map);
    crop_box.setMin(Eigen::Vector4f(
        center.x() - radius, center.y() - radius, center.z() - radius, 1.0F));
    crop_box.setMax(Eigen::Vector4f(
        center.x() + radius, center.y() + radius, center.z() + radius, 1.0F));
    crop_box.filter(*local_map);
    return local_map;
}
}  // namespace modular_3d_localizer
