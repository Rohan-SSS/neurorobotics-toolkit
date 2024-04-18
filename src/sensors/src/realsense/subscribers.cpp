#include "sensors/realsense/subscribers.h"
#include "sensors/common/display.h"

SyncedSubscriber::SyncedSubscriber(std::string node_name): Node(node_name){
	RCLCPP_DEBUG(this->get_logger(), "Creating Subscriptions");
	this->declare_parameter("realsense_talker_node_name", "realsense_talker");
	this->declare_parameter("realsense_device_name", "Intel_RealSense_D455");
	std::string talker_name = this->get_parameter("realsense_talker_node_name").as_string();
	std::string device_name = this->get_parameter("realsense_device_name").as_string();
	const std::string depth_topic = "/" + talker_name + "/" + device_name + "/camera/depth"; 
	const std::string ir_topic = "/" + talker_name + "/" + device_name + "/camera/infrared"; 
	const std::string accel_topic = "/" + talker_name + "/" + device_name + "/motion/accel"; 
	const std::string gyro_topic = "/" + talker_name + "/" + device_name + "/motion/gyro"; 
	depth.subscribe(this, depth_topic);
	ir.subscribe(this, ir_topic);
	accel.subscribe(this, accel_topic);
	gyro.subscribe(this, gyro_topic);
	RCLCPP_DEBUG(this->get_logger(), "All 4 subscriptions created");
	sync = std::make_shared<Syncer>(Policy(10), depth, ir, accel, gyro);
	RCLCPP_DEBUG(this->get_logger(), "Created Syncer");
	sync->registerCallback(std::bind(&SyncedSubscriber::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
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
