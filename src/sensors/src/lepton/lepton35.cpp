#include "sensors/lepton/lepton35.h"


Lepton::Lepton(){
    res = uvc_init(&ctx, NULL);
    if (res < 0)
    {
        uvc_perror(res, "uvc_init");
    }
}

bool Lepton::setup(){
	std::cout<<"Setting up Lepton Hub"<<std::endl;
    res = uvc_find_devices(ctx, &devs, 0x1E4E, 0x0100, NULL); /* filter devices: vendor_id, product_id, "serial_num" */
    if (res < 0)
    {
        uvc_perror(res, "uvc_find_device"); /* no devices found */
    }
	for(int i = 0; devs[i + 1] != NULL; i++){
		sensors.push_back(new LeptonCamera(devs[i]));	
	}
}

bool Lepton::start(){
	for(LeptonBase* sensor: sensors){
		sensor->start();
	}
}
