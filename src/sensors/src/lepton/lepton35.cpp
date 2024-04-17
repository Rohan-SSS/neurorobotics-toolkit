#include "sensors/lepton/lepton35.h"

Lepton::Lepton(){
    res = uvc_init(&ctx, NULL);
    if (res < 0)
    {
        uvc_perror(res, "uvc_init");
    }
}

bool Lepton::setup(std::vector<LeptonSensorProperties> &props, std::function<void(Frame, sensor_id)> cb){
	originalCallback = cb;
	std::cout<<"Setting up Lepton Hub"<<std::endl;
    res = uvc_find_devices(ctx, &devs, 0x1E4E, 0x0100, NULL); /* filter devices: vendor_id, product_id, "serial_num" */
    std::cout<<"what is res "<<res<<std::endl;
	if (res < 0)
    {
		std::cout<<"no device found**************"<<std::endl;
        uvc_perror(res, "uvc_find_device"); /* no devices found */
    }
	std::cout<<"before callback"<<std::endl;
	std::function<void(Frame, sensor_id)> callback = [this](Frame frame, sensor_id id){
		originalCallback(frame, id);
	};
	std::cout<<"after callback"<<std::endl;

	int count_dev;

	int i =0;
	while(devs[i]!=NULL){
		std::cout<<"starting for loop "<<std::endl;

		LeptonCamera* camera = new LeptonCamera(devs[i], callback);
		std::cout<<"**********1************"<<std::endl;

		if(count_dev < props.size()){
			camera->prop = &props[count_dev];
		}
		else{
			std::cout<<"Fewer devices connected than properties provided"<<std::endl;
			break;
		}
		std::cout<<"**********3************"<<std::endl;

		camera->InitDevice();
//		std::cout<<camera->InitDevice()<<std::endl;

		sensors.push_back(camera);	
		count_dev++;
		i++;
		std::cout<<"one loop over "<<std::endl;
	}
	std::cout<<"loop over "<<std::endl;
	return true;
}

bool Lepton::start(){
	std::cout<<"initialise lepton base, for loop start "<<std::endl;
	for(LeptonBase* sensor: sensors){
		sensor->start();
	}
	std::cout<<"for loop over "<<std::endl;
	return true;
}
