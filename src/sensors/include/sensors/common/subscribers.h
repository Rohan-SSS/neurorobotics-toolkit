#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"

class SimpleImageSubscriber: public rclcpp::Node{
	public:
		SimpleImageSubscriber(std::string node_name, const std::string topic_name);
		void callback(const sensor_msgs::msg::Image::SharedPtr msg);
	private:
		const std::string OPENCV_WINDOW = "Image window";
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
};

class SimpleImuSubscriber: public rclcpp::Node{
	public:
		SimpleImuSubscriber(std::string node_name, const std::string topic_name);
		void callback(const sensor_msgs::msg::Imu::SharedPtr msg);
	private:
		rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription;
};
