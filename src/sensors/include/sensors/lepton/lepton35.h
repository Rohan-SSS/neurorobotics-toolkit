#include "rclcpp/rclcpp.hpp"
#include "sensors/lepton/LeptonCamera.hpp"

class Lepton{
	public:
		Lepton();
		~Lepton();
		bool setup(std::vector<LeptonSensorProperties> &props, std::function<void(Frame, sensor_id)> cb);
		bool start();
		std::vector<LeptonBase*> sensors;
	private:
  		uvc_context_t *ctx;
  		uvc_device_t **devs;
  		uvc_error_t res;
		std::function<void(Frame, sensor_id)> originalCallback;
};

