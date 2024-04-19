#include "sensors/lepton/publishers.h"

LeptonPublisherNode::LeptonPublisherNode(std::string &nodeName): Node(nodeName){
	std::cout<<"Creating Publisher Node"<<std::endl;
}

LeptonPublisherNode::LeptonPublisherNode(std::string &nodeName, std::vector<LeptonSensorProperties> &props) : Node(nodeName){
	hub = new Lepton();

	std::function<void(Frame, sensor_id)> cb = [this](Frame frame, sensor_id id){
		frameCallback(frame, id);
	};

	hub->setup(props, cb);
	int index = 0;
	for(LeptonBase* sensor: hub->sensors){
		//image_transport::ImageTransport it_(node);
		const std::string topic_name = "~/Lepton35/device_" + std::to_string(index) + "/thermal" ;
		_frame_publishers[sensor->mDeviceSerialNumber] = rclcpp::Node::create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
		index++;
	}
	hub->start();
	
}

void LeptonPublisherNode::frameCallback(Frame frame, sensor_id id){
	
	RCLCPP_DEBUG(this->get_logger(), "frame arrived");
	rclcpp::Time t(frame.timestamp * 1e6);
	std::cout<<"time stamp of thermal in publishrer "<<frame.timestamp*1e6<<std::endl;
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
			RCLCPP_DEBUG(this->get_logger(), "frame published");
		}
		// _frame_publishers[id].pu
	}
	else{
		RCLCPP_ERROR_SKIPFIRST(this->get_logger(), "no imu publisher available for the frame");
	}
}
