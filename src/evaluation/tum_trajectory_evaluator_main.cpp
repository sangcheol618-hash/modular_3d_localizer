#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "modular_3d_localizer/evaluation/tum_trajectory_evaluator.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<modular_3d_localizer::TumTrajectoryEvaluator>());
    rclcpp::shutdown();
    return 0;
}
