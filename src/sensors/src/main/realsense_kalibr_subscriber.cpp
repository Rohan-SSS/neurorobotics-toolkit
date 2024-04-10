#include "sensors/realsense/subscribers.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;
	RealSenseKalibrSyncedIMUSubscriber::SharedPtr node = std::make_shared<RealSenseKalibrSyncedIMUSubscriber>("kalibr_logging_node", "/ws/ros_ws/kalibr_data/");
	executor.add_node(node);
	executor.spin();
	rclcpp::shutdown();
	return 0;
}
