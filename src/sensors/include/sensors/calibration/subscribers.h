#include "opencv2/opencv.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <cv_bridge/cv_bridge.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, sensor_msgs::msg::Imu> RealSenseKalibrIMUPolicy;
typedef message_filters::Synchronizer<RealSenseKalibrIMUPolicy> RealSenseKalibrIMUSyncer;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> KalibrImagePolicy;
typedef message_filters::Synchronizer<KalibrImagePolicy> KalibrImageSyncer;


bool createDirectory(std::string dirPath);

class KalibrSubscriber: public rclcpp::Node{
	public:
		KalibrSubscriber(std::string node_name, std::string logDir);
		void imuCallback(
				const sensor_msgs::msg::Imu::ConstSharedPtr &accel,
				const sensor_msgs::msg::Imu::ConstSharedPtr &gyro);
		void irFrameCallback(const sensor_msgs::msg::Image::SharedPtr infrared);
		void syncedFrameCallback(
				const sensor_msgs::msg::Image::ConstSharedPtr &infrared,
				const sensor_msgs::msg::Image::ConstSharedPtr &thermal);
		
	private:
		
		message_filters::Subscriber<sensor_msgs::msg::Imu> accel;
		message_filters::Subscriber<sensor_msgs::msg::Imu> gyro;
		message_filters::Subscriber<sensor_msgs::msg::Image> infrared_synced;
		message_filters::Subscriber<sensor_msgs::msg::Image> thermal_synced;
		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr infrared;
		rclcpp::callback_group::CallbackGroup::SharedPtr group1;
		rclcpp::callback_group::CallbackGroup::SharedPtr group2;
		rclcpp::callback_group::CallbackGroup::SharedPtr group3;
		rclcpp::SubscriptionOptions options1;
		rclcpp::SubscriptionOptions options2;
		rclcpp::SubscriptionOptions options3;
		std::shared_ptr<RealSenseKalibrIMUSyncer> syncImu;
		KalibrImagePolicy policy = KalibrImagePolicy(10);
		std::shared_ptr<KalibrImageSyncer> syncImage_;
		
		int count = 0, count2 = 0, count3 = 0;
		std::ofstream imuLogFile;
		std::string imuLogFilePath;
		std::string infraredLogDir;
		std::string syncedLogDir;
		bool closeFileAfterWrite = false;
		const std::string INFRARED_WINDOW = "Infrared window";

		bool logIMUToFile(std::string log);

		bool detectChessboardCorners(cv::Mat &img, cv::Size &patternsize, std::vector<cv::Point2f> &corners);
		bool foundRSPattern = false;
		bool foundLeptonPattern = false;
		std::vector<cv::Point2f> RSCorners;
		std::vector<cv::Point2f> LeptonCorners;
		cv::Size patternSize;

};
