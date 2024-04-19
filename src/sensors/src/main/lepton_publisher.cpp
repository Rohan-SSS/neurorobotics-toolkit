#include "sensors/lepton/publishers.h"
#include "rclcpp/rclcpp.hpp"
#include <vector>

int main(int argc, char * argv[]){
	std::vector<LeptonSensorProperties> props;
	LeptonSensorProperties thermal;
	thermal.frameWidth = 160;
	thermal.frameHeight = 120;
	thermal.frameRate = 9;
	thermal.format = UVC_FRAME_FORMAT_Y16;
	props.push_back(thermal);
	
	std::string nodeName = "lepton_node";
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LeptonPublisherNode>(nodeName, props));
	rclcpp::shutdown();
}
