#include "rclcpp/rclcpp.hpp"
#include "sensors/common/types.h"
#include "sensors/lepton/LeptonCamera.hpp"

class Lepton{
	public:
		Lepton();
		~Lepton();
		bool setup();
		bool start();
	private:
  		uvc_context_t *ctx;
  		uvc_device_t **devs;
  		uvc_error_t res;
		std::vector<LeptonBase*> sensors;
};

