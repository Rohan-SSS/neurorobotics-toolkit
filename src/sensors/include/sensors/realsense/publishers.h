#include "sensors/realsense/d455.h"
#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>
#include "sensor_msgs/msg/imu.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

class RealSensePublisherNode: public rclcpp::Node{
	public:
		RealSensePublisherNode(std::string node_name, std::vector<RealSenseSensorProperties> &props);
		bool setup(std::vector<RealSenseSensorProperties> &props);
	private:
		rclcpp::Time _ros_time_base;
		

		RealSense* hub;
		std::map<sensor_id, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> _frame_publishers;
		std::map<sensor_id, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> _imu_publishers;
		std::map<rs2_format, std::string> _rs_format_to_ros_format;

		void frameCallback(Frame frame, sensor_id id);
		void imuCallback(ImuPoint point, sensor_id id);
		void addStreamPublisher(std::string topic_name, sensor_id id, bool isVideo);
		bool _is_frame_time_initialised = false;
		bool _is_imu_time_initialised = false;
		rclcpp::Time _frame_base_time;
		rclcpp::Time _imu_base_time;	
};
