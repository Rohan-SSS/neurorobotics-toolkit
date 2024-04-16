#include "sensors/realsense/subscribers.h"
#include "sensors/common/display.h"

SyncedSubscriber::SyncedSubscriber(std::string node_name): Node(node_name){
	RCLCPP_DEBUG(this->get_logger(), "Creating Subscriptions");
	depth.subscribe(this, "/talker/Intel_RealSense_D455/camera/depth");
	ir.subscribe(this, "/talker/Intel_RealSense_D455/camera/infrared");
	accel.subscribe(this, "/talker/Intel_RealSense_D455/motion/accel");
	gyro.subscribe(this, "/talker/Intel_RealSense_D455/motion/gyro");
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

bool createDirectory(std::string dirPath){
	if(!std::filesystem::is_directory(dirPath)){
		if(std::filesystem::is_regular_file(dirPath))
		{
			std::filesystem::remove(dirPath);
		}
		return std::filesystem::create_directory(dirPath);
	}
	return false;

}

RealSenseKalibrSyncedIMUSubscriber::RealSenseKalibrSyncedIMUSubscriber(std::string node_name, std::string logDir): Node(node_name){
	RCLCPP_INFO(this->get_logger(), "Starting RealSenseKalibrSyncedIMUSubscriber");

	rclcpp::SubscriptionOptions options;
	options.callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

	accel.subscribe(this, "/realsense_talker/Intel_RealSense_D455/motion/accel");//, rmw_qos_profile_default, options);
	gyro.subscribe(this, "/realsense_talker/Intel_RealSense_D455/motion/gyro");//, rmw_qos_profile_default, options);
	RCLCPP_DEBUG(this->get_logger(), "All 2 IMU subscriptions created");
	syncImu = std::make_shared<RealSenseKalibrIMUSyncer>(RealSenseKalibrIMUPolicy(10), accel, gyro);
	syncImu->registerCallback(std::bind(&RealSenseKalibrSyncedIMUSubscriber::imuCallback, this, std::placeholders::_1, std::placeholders::_2));
	RCLCPP_INFO(this->get_logger(), "Started RealSenseKalibrSyncedIMUSubscriber");

	if(createDirectory(logDir)){
		RCLCPP_INFO(this->get_logger(), "Created new folder for logging data");
	}

	RCLCPP_INFO(this->get_logger(), "Creating file for logging IMU");
	imuLogFilePath = logDir + "kalibr_imu.csv";
	imuLogFile.open(imuLogFilePath.c_str(), std::ios::app);
	RCLCPP_INFO(this->get_logger(), "Created file for logging IMU");
	RCLCPP_INFO(this->get_logger(), "Creating folders for infrared image logging");
	infraredLogDir = logDir + "infrared/";
	if(createDirectory(infraredLogDir)){
		RCLCPP_INFO(this->get_logger(), "Created new folder for logging indrared data");
	}
	infrared = this->create_subscription<sensor_msgs::msg::Image>(
			"/realsense_talker/Intel_RealSense_D455/camera/infrared",
			10,
			std::bind(&RealSenseKalibrSyncedIMUSubscriber::frameCallback, this, std::placeholders::_1));
	cv::namedWindow(INFRARED_WINDOW);

	RCLCPP_INFO(this->get_logger(), "Started Infrared Frame subscription");

}

std::string repeat(std::string s, int n)
{
    // Copying given string to temporary string.
	std::string s1 = s;

    for (int i=1; i<n;i++)
        s += s1; // Concatenating strings

    return s;
}

void RealSenseKalibrSyncedIMUSubscriber::frameCallback(const sensor_msgs::msg::Image::SharedPtr infrared){
	RCLCPP_DEBUG(this->get_logger(), "Got Message");

	std::string infra_s = std::to_string(infrared->header.stamp.sec);
	std::string infra_ns = std::to_string(infrared->header.stamp.nanosec);
	int isize = infra_ns.size();
	if(isize < 9){
		isize = infra_ns.size() - isize;
		infra_ns = repeat("0", isize) + infra_ns;
	}
	if(count2 == 200){
		RCLCPP_INFO(this->get_logger(), "Infrared Image timestamp    : " + infra_s + infra_ns);
		count2 = 0;
	}
	count2++;

	cv_bridge::CvImagePtr cv_ptr_infrared = cv_bridge::toCvCopy(infrared, infrared->encoding);
	ShowManyImages("All Image Messages", 1, cv_ptr_infrared->image);
	cv::waitKey(1);

	cv::imwrite(infraredLogDir + infra_s + infra_ns + ".png", cv_ptr_infrared->image);
}

void RealSenseKalibrSyncedIMUSubscriber::imuCallback(
		const sensor_msgs::msg::Imu::ConstSharedPtr &accel,
		const sensor_msgs::msg::Imu::ConstSharedPtr &gyro){
	RCLCPP_DEBUG(this->get_logger(), "Got Message");

	std::stringstream logstream;
	std::string s = std::to_string(gyro->header.stamp.sec);
	std::string ns = std::to_string(gyro->header.stamp.nanosec);
	std::string accel_s = std::to_string(accel->header.stamp.sec);
	std::string accel_ns = std::to_string(accel->header.stamp.nanosec);
	int size = ns.size();
	int asize = accel_ns.size();
	if(size < 9){
		size = ns.size() - size;
		ns = repeat("0", size) + ns;
	}
	if(asize < 9){
		asize = accel_ns.size() - asize;
		accel_ns = repeat("0", asize) + accel_ns;
	}
	logstream << std::setprecision(10) << std::fixed << s << ns << ",";
	logstream << std::setprecision(10) << std::fixed << std::to_string(gyro->angular_velocity.x) << ",";
	logstream << std::setprecision(10) << std::fixed << std::to_string(gyro->angular_velocity.y) << ",";
	logstream << std::setprecision(10) << std::fixed << std::to_string(gyro->angular_velocity.z) << ",";
	logstream << std::setprecision(10) << std::fixed << std::to_string(accel->linear_acceleration.x) << ",";
	logstream << std::setprecision(10) << std::fixed << std::to_string(accel->linear_acceleration.y) << ",";
	logstream << std::setprecision(10) << std::fixed << std::to_string(accel->linear_acceleration.z);
	std::string log = logstream.str();
	logIMUToFile(log);

	if(count == 200){
		RCLCPP_INFO(this->get_logger(), "Accelerometer Image timestamp    : " + accel_s + accel_ns);
		RCLCPP_INFO(this->get_logger(), "Gyroscope Image timestamp        : " + s + ns);
		count = 0;
	}
	count++;
}

bool RealSenseKalibrSyncedIMUSubscriber::logIMUToFile(std::string log){
	if(!imuLogFile.is_open()){
		imuLogFile.open(imuLogFilePath.c_str(), std::ios::app);
		if(!imuLogFile.is_open()){
			return false;
		}
	}
	imuLogFile << log << std::endl;
	if(closeFileAfterWrite){
		imuLogFile.close();
	}
	return true;
}

