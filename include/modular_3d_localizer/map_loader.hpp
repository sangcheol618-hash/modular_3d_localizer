#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>


namespace modular_3d_localizer
{
    class MapLoader
    {
    public:
        using PointType = pcl::PointXYZI;
        using PointCloud = pcl::PointCloud<PointType>;

        MapLoader();
        bool loadMap(const std::string &map_file);
        PointCloud::ConstPtr getMap() const;
    private:
        PointCloud::Ptr map_cloud_;
    };
} // namespace modular_3d_localizer
