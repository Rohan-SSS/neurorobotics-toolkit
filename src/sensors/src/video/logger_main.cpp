#include "sensors/video/logger.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoLoggerNode>();
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());

    node->configure();
    node->activate();
    // node->deactivate();
    // node->cleanup();
    // node->shutdown();

    executor.spin();
    rclcpp::shutdown();
    return 0;
}