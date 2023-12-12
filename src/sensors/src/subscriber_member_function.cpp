#include "sensors/subscribers.h"

int main(int argc, char * argv[])
{
	std::cout<<"Starting listener."<<std::endl;
	const std::string depthTopic = "/realsense_node/Intel_RealSense_D455/camera/depth";
	const std::string infraredTopic = "/realsense_node/Intel_RealSense_D455/camera/infrared";
	const std::string accelTopic = "/realsense_node/Intel_RealSense_D455/motion/accel";
	const std::string gyroTopic = "/realsense_node/Intel_RealSense_D455/motion/gyro";

	//rclcpp::Node::SharedPtr depthNode = std::make_shared<SimpleImageSubscriber>("realsense_depth_subscriber_node", depthTopic);
	//rclcpp::Node::SharedPtr infraredNode = std::make_shared<SimpleImageSubscriber>("realsense_infrared_subscriber_node", infraredTopic);
	//rclcpp::Node::SharedPtr accelNode = std::make_shared<SimpleImageSubscriber>("realsense_accel_subscriber_node", accelTopic);
	//rclcpp::Node::SharedPtr gyroNode = std::make_shared<SimpleImageSubscriber>("realsense_gyro_subscriber_node", gyroTopic);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SyncedSubscriber>("SyncedSub"));
	rclcpp::shutdown();
	std::cout<<"Exiting listener"<<std::endl;
	return 0;

}
