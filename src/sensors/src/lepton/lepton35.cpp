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
    
	if (res < 0)
    {
		
        uvc_perror(res, "uvc_find_device"); /* no devices found */
    }
	
	std::function<void(Frame, sensor_id)> callback = [this](Frame frame, sensor_id id){
		originalCallback(frame, id);
	};
	

	int count_dev;

	int i =0;
	while(devs[i]!=NULL){
		

		LeptonCamera* camera = new LeptonCamera(devs[i], callback);
		

		if(count_dev < props.size()){
			camera->prop = &props[count_dev];
		}
		else{
			std::cout<<"Fewer devices connected than properties provided"<<std::endl;
			break;
		}
		

		camera->InitDevice();
//		std::cout<<camera->InitDevice()<<std::endl;

		sensors.push_back(camera);	
		count_dev++;
		i++;
		
	}
	
	return true;
}

bool Lepton::start(){
	
	for(LeptonBase* sensor: sensors){
		sensor->start();
	}
	
	return true;
}
