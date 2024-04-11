#include "rclcpp/rclcpp.hpp"

class Lepton{
	public:
		Lepton();
		~Lepton();
};

class LeptonNode: public rclcpp::Node{
	public:
		LeptonNode(std::string node_name);
		~LeptonNode();
};
