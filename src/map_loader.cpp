#include "modular_3d_localizer/map_loader.hpp"

#include <filesystem>
#include <algorithm>
#include <cctype>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

namespace modular_3d_localizer
{
    namespace
    {
        void normalizeFieldNames(pcl::PCLPointCloud2 &cloud)
        {
            for (auto &field : cloud.fields)
            {
                std::transform(
                    field.name.begin(), field.name.end(), field.name.begin(),
                    [](unsigned char character)
                    {
                        return std::tolower(character);
                    }
                );

                if (field.name == "scalar_intensity")
                {
                    field.name = "intensity";
                }
            }
        }
    }  // namespace

    MapLoader::MapLoader() : map_cloud_(new PointCloud)
    {

    }

    bool MapLoader::loadMap(const std::string &map_file)
    {
        std::string extension = std::filesystem::path(map_file).extension().string();
        std::transform(
            extension.begin(), extension.end(), extension.begin(),
            [](unsigned char character)
            {
                return std::tolower(character);
            }
        );

        int result = -1;

        if (extension == ".pcd")
        {
            pcl::PCLPointCloud2 pcd_cloud;
            result = pcl::io::loadPCDFile(map_file, pcd_cloud);

            if (result == 0)
            {
                normalizeFieldNames(pcd_cloud);
                pcl::fromPCLPointCloud2(pcd_cloud, *map_cloud_);
            }
        }
        else if (extension == ".ply")
        {
            pcl::PCLPointCloud2 ply_cloud;
            result = pcl::io::loadPLYFile(map_file, ply_cloud);

            if (result == 0)
            {
                normalizeFieldNames(ply_cloud);
                pcl::fromPCLPointCloud2(ply_cloud, *map_cloud_);
            }
        }
        else
        {
            return false;
        }

        if (result != 0)
        {
            return false;
        }
        return true;
    }

    MapLoader::PointCloud::ConstPtr MapLoader::getMap() const
    {
        return map_cloud_;
    }

} // namespace modular_3d_localizer
