#ifndef SENSOR_PROPERTIES
#define SENSOR_PROPERTIES
#include "sensors/properties.h"
#endif
#ifndef UTIL_CLASSES
#define UTIL_CLASSES
#include "sensors/UtilClasses.h"
#endif
#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <mutex>
#include <atomic>
#include <fstream>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"



class VideoFileReader{
	public:
		VideoFileReader(rclcpp::Logger logger, std::function<void(cv::Mat)> cb, std::string filePath);
		void start();
		void AcquireVideoData();
	private:
		rclcpp::Logger LOGGER;
		std::string videoFilePath;
		
		std::function<void(cv::Mat)> callback;
		std::function<void(cv::Mat)> originalCallback;		
		
		std::thread *th;
};


class SimpleImagePublisher: public rclcpp::Node{
	public:
		SimpleImagePublisher(std::string nodeName);
	private:
		VideoFileReader* reader;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _frame_publisher;
		void frameCallback(cv::Mat);	
		int initializeFormatsMaps();
		std::map<int, std::string> _cv_format_to_ros_format;
};
