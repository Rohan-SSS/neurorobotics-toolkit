#include "sensors/realsense/publishers.h"

RealSensePublisherNode::RealSensePublisherNode(std::string nodeName, std::vector<RealSenseSensorProperties> &props) : Node(nodeName){
	//TODO Node configuration according to mode of data that can be published
	std::cout<<"Creating Real Sense Node"<<std::endl;
	hub = new RealSense();	
	hub->setup(props);
	for(auto &[id, topicPair]: hub->sensorTopicMap){
		const std::string topic_name = topicPair.first;
		bool isVideo = topicPair.second;
		addStreamPublisher(topic_name, id, isVideo);
	}
	std::cout<<"Added stream publishers"<<std::endl;

	std::function<void(Frame, sensor_id)> fcb = [this](Frame frame, sensor_id id){
		frameCallback(frame, id);	
	};
	std::function<void(ImuPoint, sensor_id)> icb = [this](ImuPoint point, sensor_id id){
		imuCallback(point, id);
	};
	hub->start(props, fcb, icb);
	std::cout<<"Started Real Sense Node"<<std::endl;
}

void RealSensePublisherNode::addStreamPublisher(std::string topic_name, sensor_id id, bool isVideo){
	std::cout<<"Creating Publisher for topic: "<<topic_name<<std::endl;
	if(isVideo){
		std::cout<<"Creating Image Publisher"<<std::endl;
		_frame_publishers[id] = rclcpp::Node::create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
	}
	else{
		std::cout<<"Creating Imu Publisher"<<std::endl;
		_imu_publishers[id] = rclcpp::Node::create_publisher<sensor_msgs::msg::Imu>(topic_name, 10);
	}	
}

void RealSensePublisherNode::frameCallback(Frame frame, sensor_id id){
	if(!_is_frame_time_initialised){
		_is_frame_time_initialised = true;
		_frame_base_time = this->now();
	}
	RCLCPP_DEBUG(this->get_logger(), "frame arrived");
	rclcpp::Time t(frame.timestamp);
	if(_frame_publishers.find(id) != _frame_publishers.end()){
		if(!frame.frame.empty()){
			sensor_msgs::msg::Image::UniquePtr img_msg_ptr(new sensor_msgs::msg::Image());
			cv_bridge::CvImage(
					std_msgs::msg::Header(),
					frame.format,
					frame.frame
				).toImageMsg(*img_msg_ptr);
			img_msg_ptr->header.stamp = t;
			img_msg_ptr->height = frame.height;
			img_msg_ptr->width = frame.width;
			img_msg_ptr->is_bigendian = false;
			img_msg_ptr->step = frame.width * frame.frame.elemSize();
			_frame_publishers[id]->publish(std::move(img_msg_ptr));
			RCLCPP_DEBUG(this->get_logger(), "frame available");
		}
	}
	else{
		RCLCPP_ERROR_SKIPFIRST(this->get_logger(), "no imu publisher available for the frame");
	}
}

void RealSensePublisherNode::imuCallback(ImuPoint point, sensor_id id){
	if(!_is_imu_time_initialised){
		_is_imu_time_initialised = true;
		_imu_base_time = this->now();
	}
	// TODO move realsense code to d455.cpp
	// TODO need to fix ros timestamp and ros_base_time correction here
	rclcpp::Time t(point.timestamp);
	if(_imu_publishers.find(id) != _imu_publishers.end()){
		RCLCPP_DEBUG(this->get_logger(), "IMU Frame arrived");
		sensor_msgs::msg::Imu imu_msg = sensor_msgs::msg::Imu();
		imu_msg.header.stamp = t;
		// Default values for unnecessary members
		imu_msg.orientation.x = 0.0;
		imu_msg.orientation.y = 0.0;
		imu_msg.orientation.z = 0.0;
		imu_msg.orientation.w = 0.0;

		imu_msg.orientation_covariance = { -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		imu_msg.linear_acceleration_covariance = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		imu_msg.angular_velocity_covariance = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		if(std::get<1>(id) == RS2_STREAM_GYRO){
			RCLCPP_DEBUG(this->get_logger(), "Got Gyro Data");
			imu_msg.angular_velocity.x = point.gyro.x;
			imu_msg.angular_velocity.y = point.gyro.y;
			imu_msg.angular_velocity.z = point.gyro.z;
		}
		else if(std::get<1>(id) == RS2_STREAM_ACCEL){
			RCLCPP_DEBUG(this->get_logger(), "Got Accel Data");
			imu_msg.linear_acceleration.x = point.accel.x;
			imu_msg.linear_acceleration.y = point.accel.y;
			imu_msg.linear_acceleration.z = point.accel.z;
		}
	_imu_publishers[id]->publish(imu_msg);
	}
	else{
		RCLCPP_ERROR_SKIPFIRST(this->get_logger(), "no imu publisher available for the frame");
	}
}
