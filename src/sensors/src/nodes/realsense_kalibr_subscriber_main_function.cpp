#include "sensors/nodes/subscribers.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RealSenseKalibrSyncedIMUSubscriber>("kalibr_logging_node", "/ws/ros_ws/kalibr_data/"));
	rclcpp::shutdown();
	return 0;
}
