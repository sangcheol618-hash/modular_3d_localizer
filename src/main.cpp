#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <memory>

#include "modular_3d_localizer/modular_3d_localizer_node.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<modular_3d_localizer::Modular3DLocalizerNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
