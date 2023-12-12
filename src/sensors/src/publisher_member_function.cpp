#ifndef SENSOR_PROPERTIES
#define SENSOR_PROPERTIES
#include "sensors/properties.h"
#endif
#include "sensors/video.h"
#include "sensors/gps.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
	
	//TODO Replace with proper configuration mechanism
	std::vector<SensorProperties> props;
	SensorProperties infrared, depth, gyro, accel;
	props.push_back(infrared);
	depth.frameType = RS2_FORMAT_Z16;
	depth.streamDataType = RS2_STREAM_DEPTH;
	props.push_back(depth);
	gyro.frameType = RS2_FORMAT_MOTION_XYZ32F;
	gyro.streamDataType = RS2_STREAM_GYRO;
	gyro.isVideo = false;
	gyro.frameRate = 200; 
	props.push_back(gyro);
	accel.frameType = RS2_FORMAT_MOTION_XYZ32F;
	accel.streamDataType = RS2_STREAM_ACCEL;
	accel.isVideo = false;
	accel.frameRate = 63;
	props.push_back(accel);

	/**
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RealSenseNode>("realsense_node", props));
	rclcpp::shutdown();
	*/
	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(std::make_shared<SimpleImagePublisher>("video_node"));
	executor.add_node(std::make_shared<SimpleGpsPublisher>("gps_node", "DayGpsFile"));
	executor.spin();
	rclcpp::shutdown();
	return 0;
}
