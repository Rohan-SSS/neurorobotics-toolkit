#include "sensors/common/subscribers.h"

int main(int argc, char * argv[]){
	std::cout<<"Spinning Up Node for debug lepton listener"<<std::endl;
	const std::string topic_name = "/lepton_node/Lepton35/device_0/thermal";
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SimpleImageSubscriber>("lepton_node", topic_name));
	rclcpp::shutdown();
	std::cout<<"Exiting listener"<<std::endl;
	return 0;
}
