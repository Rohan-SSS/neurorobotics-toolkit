#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "Sensor.h"
#include "Video.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensors/realsense.h"
#include "sensors/gps.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
	public:
		MinimalPublisher()
			: Node("minimal_publisher"), count_(0)
		{
			// Created Publisher for timed publishing of data
			publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
			timer_ = this->create_wall_timer(
					500ms, std::bind(&MinimalPublisher::timer_callback, this));
			
			// Creating SkyNetSensor object for fetching data
			YAML::Node cameraNode = YAML::LoadFile("/ws/SkyNet/Configurations/Laptop/orbslam2/VideoConfiguration/VideoConfiguration_02_02_2021_16_28_42_1.yaml");
			std::cout<<"Successfully read test configurations"<<std::endl;
			cap = new SkyNetSensor::Video(cameraNode, 0);
			cap->LaunchSensorProcess(0);
			std::cout<<"Successfully created video object reference"<<std::endl;
		}

	private:
		void timer_callback()
		{
			auto message = std_msgs::msg::String();
			message.data = "Hello, world! " + std::to_string(count_++);
			RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

			// Getting Frame From SkyNet Sensor
			//SkyNet::Sensor_Status status = cap->GetCurrentFrames(mVideoFrameLeft, mVideoFrameRight, mDepthFrame, mColourFrameLeftTimeStamp, mColourFrameRightTimeStamp, mDepthFrameTimeStamp);
			//std::cout<<"Sensor Status: "<<status<<std::endl;


			publisher_->publish(message);
		}
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
		SkyNetSensor::Video *cap;
		cv::Mat mVideoFrameLeft, mVideoFrameRight, mDepthFrame;
		double mColourFrameLeftTimeStamp, mColourFrameRightTimeStamp, mDepthFrameTimeStamp;  
		size_t count_;
};

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

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RealSenseNode>("realsense_node", props));
	rclcpp::shutdown();
	// rclcpp::init(argc, argv);
	// rclcpp::executors::MultiThreadedExecutor executor;
	// executor.add_node(std::make_shared<RealSenseNode>("realsense_node", props));
	// executor.add_node(std::make_shared<SimpleGpsPublisher>("gps_node"));
	// executor.spin();
	// rclcpp::shutdown();
	return 0;
}
