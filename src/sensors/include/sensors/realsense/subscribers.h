#include "sensors/realsense/d455.h"
#include "sensor_msgs/msg/imu.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#ifndef GPS_MSGS
#define GPS_MSGS
#endif
#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <cv_bridge/cv_bridge.h>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

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

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Imu, sensor_msgs::msg::Imu> Policy;
typedef message_filters::Synchronizer<Policy> Syncer;

class SyncedSubscriber: public rclcpp::Node{
	public:
		SyncedSubscriber(std::string node_name);
		void callback(
				const sensor_msgs::msg::Image::ConstSharedPtr &depth,
				const sensor_msgs::msg::Image::ConstSharedPtr &infrared,
				const sensor_msgs::msg::Imu::ConstSharedPtr &accel,
				const sensor_msgs::msg::Imu::ConstSharedPtr &gyro);
	private:
		message_filters::Subscriber<sensor_msgs::msg::Image> depth;
		message_filters::Subscriber<sensor_msgs::msg::Image> ir;
		message_filters::Subscriber<sensor_msgs::msg::Imu> accel;
		message_filters::Subscriber<sensor_msgs::msg::Imu> gyro;
		std::shared_ptr<Syncer> sync;
		const std::string OPENCV_WINDOW_DEPTH = "Depth window";
		const std::string OPENCV_WINDOW_INFRA = "Infrared window";
		int count;
};

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, sensor_msgs::msg::Imu> RealSenseKalibrIMUPolicy;
typedef message_filters::Synchronizer<RealSenseKalibrIMUPolicy> RealSenseKalibrIMUSyncer;


bool createDirectory(std::string dirPath);

class RealSenseKalibrSyncedIMUSubscriber: public rclcpp::Node{
	public:
		RealSenseKalibrSyncedIMUSubscriber(std::string node_name, std::string logDir);
		void imuCallback(
				const sensor_msgs::msg::Imu::ConstSharedPtr &accel,
				const sensor_msgs::msg::Imu::ConstSharedPtr &gyro);
		void frameCallback(const sensor_msgs::msg::Image::SharedPtr infrared);
	private:
		message_filters::Subscriber<sensor_msgs::msg::Imu> accel;
		message_filters::Subscriber<sensor_msgs::msg::Imu> gyro;
		rclcpp::callback_group::CallbackGroup::SharedPtr group1;
		rclcpp::callback_group::CallbackGroup::SharedPtr group2;
		rclcpp::SubscriptionOptions options1;
		rclcpp::SubscriptionOptions options2;
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr infrared;
		std::shared_ptr<RealSenseKalibrIMUSyncer> syncImu;
		int count = 0, count2 = 0;
		std::ofstream imuLogFile;
		std::string imuLogFilePath;
		std::string infraredLogDir;
		bool closeFileAfterWrite = false;
		const std::string INFRARED_WINDOW = "Infrared window";

		bool logIMUToFile(std::string log);
};
