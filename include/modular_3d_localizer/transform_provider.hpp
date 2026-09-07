#pragma once

#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>
#include "static_transform.hpp"


namespace modular_3d_localizer
{
    class TransformProvider
    {
    public:
        bool addTransform(const Static_transform &transform);

        bool getTransform(
            const std::string &parent_frame,
            const std::string &child_frame,
            Static_transform &transform
        ) const;
        bool loadTransforms(
            const std::string &yaml_file,
            const std::string &lidar_frame,
            const std::string &camera_frame);

    private:
        std::vector<Static_transform> transforms_;

    };
}  // namespace modular_3d_localizer
