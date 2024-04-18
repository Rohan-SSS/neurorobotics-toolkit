#include "sensors/calibration/subscribers.h"

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;
	KalibrSubscriber::SharedPtr node = std::make_shared<KalibrSubscriber>("kalibr_logging_node", "/ws/ros_ws/kalibr_data/");
	executor.add_node(node);
	executor.spin();
	rclcpp::shutdown();
	return 0;
}
