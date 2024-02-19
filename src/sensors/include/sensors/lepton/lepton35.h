#include "LeptonCamera.hpp"
#include "LeptonCameraSync.hpp"

class Lepton{
	public:
		Lepton();
		~Lepton();
	private:
		LeptonCamera *mpCamera;
};

class LeptonNode: public rclcpp::Node{
	public:
		LeptonNode(std::string node_name);
		~LeptonNode();
	private:
		Lepton* sensor;
};
