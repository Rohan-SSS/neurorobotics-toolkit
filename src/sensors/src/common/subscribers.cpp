#include "sensors/common/subscribers.h"

SimpleImageSubscriber::SimpleImageSubscriber(std::string node_name, const std::string topic_name): Node(node_name){
	subscription = this->create_subscription<sensor_msgs::msg::Image>(
			topic_name,
			10,
			std::bind(&SimpleImageSubscriber::callback, this, std::placeholders::_1));
	cv::namedWindow(OPENCV_WINDOW);
}

void SimpleImageSubscriber::callback(sensor_msgs::msg::Image::SharedPtr msg){
	std::cout<<"got message"<<std::endl;
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
	cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	cv::waitKey(1);
}


SimpleImuSubscriber::SimpleImuSubscriber(std::string node_name, const std::string topic_name): Node(node_name){
	subscription = this->create_subscription<sensor_msgs::msg::Image>(
			topic_name,
			10,
			std::bind(&SimpleImuSubscriber::callback, this, std::placeholders::_1));
}

void SimpleImuSubscriber::callback(sensor_msgs::msg::Imu::SharedPtr msg){
	std::cout<<"got message"<<std::endl;
}
