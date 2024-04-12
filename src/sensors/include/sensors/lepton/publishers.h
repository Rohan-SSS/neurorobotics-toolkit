#include "sensors/lepton/lepton35.h"

class LeptonPublisherNode: public rclcpp::Node{
	public:
		LeptonPublisherNode(std::string &nodeName);
		LeptonPublisherNode(std::string &nodeName, std::vector<LeptonSensorProperties> &props);
	private:
		Lepton* hub;
		std::map<sensor_id, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> _frame_publishers;
		void frameCallback(Frame frame, sensor_id id);
};
