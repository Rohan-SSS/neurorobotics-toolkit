#include "frontier_detection/map.hpp"

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	auto node = std::make_shared<OctoMapNode>("octomap_node");
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
