#include "modular_3d_localizer/transform_provider.hpp"

namespace modular_3d_localizer
{
    bool TransformProvider::addTransform(const Static_transform &transform)
    {
        transforms_.push_back(transform);
        return true;
    }

    bool TransformProvider::getTransform(
        const std::string &parent_frame,
        const std::string &child_frame,
        Static_transform &transform) const
    {
        for (const auto &t : transforms_)
        {
            if (t.parent_frame == parent_frame && t.child_frame == child_frame)
            {
                transform = t;
                return true;
            }
        }
        return false;
    }

    bool TransformProvider::loadTransforms(const std::string &yaml_path, const std::string &lidar_frame_, const std::string &camera_frame_  )
    {
        YAML::Node root = YAML::LoadFile(yaml_path);
        for (const auto &entry : root)
        {
            const std::string frame_name = entry.first.as<std::string>();

            if (frame_name != camera_frame_)
            {
                continue;
            }

            const YAML::Node & frame_data = entry.second;
            if (!frame_data["T_cam_lidar"]){
                continue;
            }
            const YAML::Node transform_node = frame_data["T_cam_lidar"];

            Eigen::Matrix4d transform_matrix;
            for (int row = 0; row < 4; ++row)
            {
                for (int col = 0; col < 4; ++col)
                {
                    transform_matrix(row, col) = transform_node[row][col].as<double>();
                }

            }
            Static_transform transform;

            transform.parent_frame = lidar_frame_;
            transform.child_frame = camera_frame_;

            transform.translation = transform_matrix.block<3, 1>(0, 3);
            transform.rotation = Eigen::Quaterniond(transform_matrix.block<3, 3>(0, 0));

            addTransform(transform);

        }
        return true;
    }
}  // namespace modular_3d_localizer
