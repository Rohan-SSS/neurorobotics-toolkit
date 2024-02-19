#include "sensors/realsense/d455.h"

ROSSensor::ROSSensor(rs2::sensor sensor,
		std::function<void(rs2::frame)> cb): rs2::sensor(sensor), originalCallback(cb){
	std::cout<<"Creating ROS Sensor"<<std::endl;
	std::cout<<"creating sensor: "<<sensor.get_info(RS2_CAMERA_INFO_NAME)<<std::endl;
	callback = [this](rs2::frame frame){
		originalCallback(frame);
	};
}

bool ROSSensor::start(const std::vector<rs2::stream_profile> profiles){
	rs2::sensor::open(profiles);
	rs2::sensor::start(callback);
	return true;
}

void ROSSensor::stop(){
	rs2::sensor::stop();
}

RealSense::RealSense(std::function<void(rs2::frame)> frameCallback, std::function<void(rs2::frame)> imuCallback): originalFrameCallback(frameCallback), originalImuCallback(imuCallback){
	std::cout<<"Creating Real Sense Devices Hub"<<std::endl;

	std::function<void(rs2::frame)> _frameCallback = [this](rs2::frame frame){
		originalFrameCallback(frame);
	};
	
	std::function<void(rs2::frame)> _imuCallback = [this](rs2::frame frame){
		originalImuCallback(frame);
	};

  	auto _device_changed_cb = [this](rs2::event_information& info) {
		for(rs2::device& dev: devices){
			if(info.was_removed(dev)){
				std::cout<<dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)<<" Device Disconnected"<<std::endl;
			}
		}
		// TODO Check if need to put the following here:
		//queryAndConnect();
	};
	ctx.set_devices_changed_callback(_device_changed_cb);

	// Query and Connect a single RealSense Device to appropriate pipelines
	for(auto&& dev : ctx.query_devices()){
		devices.push_back(dev);
		for(auto&& sensor: dev.query_sensors()){
			ROSSensor* rosSensor;	
			if(sensor.is<rs2::depth_sensor>() || sensor.is<rs2::color_sensor>()){
				std::cout<<"setting frame callback"<<std::endl;
				rosSensor = new ROSSensor(sensor, _frameCallback);
			}
			else if(sensor.is<rs2::motion_sensor>()){	
				std::cout<<"setting imu callback"<<std::endl;
				rosSensor = new ROSSensor(sensor, _imuCallback);
			}
			else{
				std::cout<<"no callback set for a sensor: "<<sensor.get_info(RS2_CAMERA_INFO_NAME)<<std::endl;
				continue;
			}
			sensors.push_back(std::make_pair(
						rosSensor, std::make_pair(
							dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER),
							dev.get_info(RS2_CAMERA_INFO_NAME))
						));
		}
	}
	std::cout<<"Device Querying Done"<<std::endl;
}

std::vector<std::pair<ROSSensor*, std::pair<std::string, std::string>>> RealSense::queryAllSensors(){
	return sensors;
}
