#include "sensors/subscribers.h"
#include "sensors/display.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;

SimpleImageSubscriber::SimpleImageSubscriber(std::string node_name, const std::string topic_name): Node(node_name){
	subscription = this->create_subscription<sensor_msgs::msg::Image>(
			topic_name,
			10,
			std::bind(&SimpleImageSubscriber::callback, this, _1));
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
			std::bind(&SimpleImuSubscriber::callback, this, _1));
}

void SimpleImuSubscriber::callback(sensor_msgs::msg::Imu::SharedPtr msg){
	std::cout<<"got message"<<std::endl;
}

SyncedSubscriber::SyncedSubscriber(std::string node_name): Node(node_name){
	RCLCPP_DEBUG(this->get_logger(), "Creating Subscriptions");
	depth.subscribe(this, "/realsense_node/Intel_RealSense_D455/camera/depth");
	ir.subscribe(this, "/realsense_node/Intel_RealSense_D455/camera/infrared");
	accel.subscribe(this, "/realsense_node/Intel_RealSense_D455/motion/accel");
	gyro.subscribe(this, "/realsense_node/Intel_RealSense_D455/motion/gyro");
	RCLCPP_DEBUG(this->get_logger(), "All 4 subscriptions created");
	sync = std::make_shared<Syncer>(Policy(10), depth, ir, accel, gyro);
	RCLCPP_DEBUG(this->get_logger(), "Created Syncer");
	sync->registerCallback(std::bind(&SyncedSubscriber::callback, this, _1, _2, _3, _4));
	RCLCPP_DEBUG(this->get_logger(), "Registered Callback");

}

void SyncedSubscriber::callback(
		const sensor_msgs::msg::Image::ConstSharedPtr &depth,
		const sensor_msgs::msg::Image::ConstSharedPtr &infrared,
		const sensor_msgs::msg::Imu::ConstSharedPtr &accel,
		const sensor_msgs::msg::Imu::ConstSharedPtr &gyro){
	RCLCPP_DEBUG(this->get_logger(), "Got Message");
	cv_bridge::CvImagePtr cv_ptr_depth;
	cv_ptr_depth = cv_bridge::toCvCopy(depth, depth->encoding);
	//cv::imshow(OPENCV_WINDOW_DEPTH, cv_ptr_depth->image);
	
	cv_bridge::CvImagePtr cv_ptr_infrared;
	cv_ptr_infrared = cv_bridge::toCvCopy(infrared, infrared->encoding);
	//cv::imshow(OPENCV_WINDOW_INFRA, cv_ptr_infrared->image);
	ShowManyImages("All Image Messages", 2, cv_ptr_depth->image, cv_ptr_infrared->image);

	if(count == 50){
		RCLCPP_INFO(this->get_logger(), "Depth Image timestamp        : %d", depth->header.stamp.sec);
		RCLCPP_INFO(this->get_logger(), "Infrared Image timestamp     : %d", infrared->header.stamp.sec);
		RCLCPP_INFO(this->get_logger(), "Gyroscope Image timestamp    : %d", accel->header.stamp.sec);
		RCLCPP_INFO(this->get_logger(), "Accelerometer Image timestamp: %d", gyro->header.stamp.sec);
		count = 0;
	}
	count++;

	cv::waitKey(1);
}
