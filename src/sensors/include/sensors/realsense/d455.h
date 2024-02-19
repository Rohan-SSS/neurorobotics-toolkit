#ifndef SENSOR_PROPERTIES
#define SENSOR_PROPERTIES
#include "sensors/common/properties.h"
#endif
#include "sensors/common/sensor.h"
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <map>



class ROSSensor: public rs2::sensor{
	public:
		ROSSensor(rs2::sensor sensor,
				std::function<void(rs2::frame)> cb);
		bool start(const std::vector<rs2::stream_profile> profiles);
		void stop();
	private:
		std::function<void(rs2::frame)> originalCallback;
		std::function<void(rs2::frame)> callback;
};


class RealSense{
	public:
		RealSense(std::function<void(rs2::frame)> frameCallback,
				std::function<void(rs2::frame)> imuCallback);
		~RealSense();
		int LaunchSensorProcess();
		std::vector<std::pair<ROSSensor*, std::pair<std::string, std::string>>> queryAllSensors();
	private:
		rs2::context ctx;
		std::vector<rs2::device> devices;
		std::vector<std::pair<ROSSensor*, std::pair<std::string, std::string>>> sensors;
		//std::function<void(rs2::frame)> frameCallback;
		std::function<void(rs2::frame)> originalFrameCallback;
		//std::function<void(rs2::frame)> imuCallback;
		std::function<void(rs2::frame)> originalImuCallback;
	
};
