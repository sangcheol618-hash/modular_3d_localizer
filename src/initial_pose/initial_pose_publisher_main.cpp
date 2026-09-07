#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "modular_3d_localizer/initial_pose/initial_pose_publisher.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<modular_3d_localizer::InitialPosePublisher>());
    rclcpp::shutdown();
    return 0;
}
