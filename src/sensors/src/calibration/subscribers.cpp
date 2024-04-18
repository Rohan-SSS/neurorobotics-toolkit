#include "sensors/calibration/subscribers.h"
#include "sensors/common/display.h"

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

KalibrSubscriber::KalibrSubscriber(std::string node_name, std::string logDir): Node(node_name){
	RCLCPP_INFO(this->get_logger(), "Starting KalibrSubscriber");
	// Declaring Parameters and creating topic names;
	this->declare_parameter("realsense_talker_node_name", "realsense_talker");
	this->declare_parameter("realsense_device_name", "Intel_RealSense_D455");
	this->declare_parameter("lepton_talker_node_name", "lepton_talker");
	this->declare_parameter("lepton_device_name", "device_0");
	std::string talker_name = this->get_parameter("realsense_talker_node_name").as_string();
	std::string device_name = this->get_parameter("realsense_device_name").as_string();
	std::string lepton_talker_name = this->get_parameter("lepton_talker_node_name").as_string();
	std::string lepton_device_name = this->get_parameter("lepton_device_name").as_string();
	const std::string ir_topic = "/" + talker_name + "/" + device_name + "/camera/infrared"; 
	const std::string accel_topic = "/" + talker_name + "/" + device_name + "/motion/accel"; 
	const std::string gyro_topic = "/" + talker_name + "/" + device_name + "/motion/gyro"; 
	const std::string thermal_topic = "/" + lepton_talker_name +  "/Lepton35/" + lepton_device_name + "/thermal";

	// Logging Setup for IMU
	if(createDirectory(logDir)){
		RCLCPP_INFO(this->get_logger(), "Created new folder for logging data");
	}
	RCLCPP_INFO(this->get_logger(), "Creating file for logging IMU");
	imuLogFilePath = logDir + "imu.csv";
	imuLogFile.open(imuLogFilePath.c_str(), std::ios::app);
	RCLCPP_INFO(this->get_logger(), "Created file for logging IMU");
	RCLCPP_INFO(this->get_logger(), "Creating folders for infrared image logging");
	// IMU subscription
	options1.callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
	accel.subscribe(this, accel_topic, rmw_qos_profile_default, options1);
	gyro.subscribe(this, gyro_topic, rmw_qos_profile_default, options1);
	RCLCPP_DEBUG(this->get_logger(), "All 2 IMU subscriptions created");
	syncImu = std::make_shared<RealSenseKalibrIMUSyncer>(RealSenseKalibrIMUPolicy(10), accel, gyro);
	syncImu->registerCallback(std::bind(&KalibrSubscriber::imuCallback, this, std::placeholders::_1, std::placeholders::_2));
	RCLCPP_INFO(this->get_logger(), "Started IMU subscriber for KalibrSubscriber");

	// Logging Setup for Infrared at Real time
	infraredLogDir = logDir + "infrared/";
	if(createDirectory(infraredLogDir)){
		RCLCPP_INFO(this->get_logger(), "Created new folder for logging infrared data");
	}
	// Infrared Subscription at 20 Hz
	options2.callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
	infrared = this->create_subscription<sensor_msgs::msg::Image>(
			ir_topic,
			10,
			std::bind(&KalibrSubscriber::irFrameCallback, this, std::placeholders::_1),
			options2);
	cv::namedWindow(INFRARED_WINDOW);	
	RCLCPP_INFO(this->get_logger(), "Started Infrared Frame subscription");

	// Logging Setup for Synced Thermal and Infrared at Real time
	syncedLogDir = logDir + "synced/";
	if(createDirectory(syncedLogDir)){
		RCLCPP_INFO(this->get_logger(), "Created new folder for logging synced data");
	}
	else{
		RCLCPP_INFO(this->get_logger(), "Unable to create folder for synced image logging");
	}
	if(createDirectory(syncedLogDir + "thermal/")){
		RCLCPP_INFO(this->get_logger(), "Created new folder for logging synced thermal data");
	}
	if(createDirectory(syncedLogDir + "infrared/")){
		RCLCPP_INFO(this->get_logger(), "Created new folder for logging synced infrared data");
	}
	options3.callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
	infrared_synced.subscribe(this, ir_topic, rmw_qos_profile_default, options3);
	thermal_synced.subscribe(this, thermal_topic, rmw_qos_profile_default, options3);
	RCLCPP_DEBUG(this->get_logger(), "All 2 Synced Image subscriptions created");
	syncImage = std::make_shared<KalibrImageSyncer>(KalibrImagePolicy(10), infrared_synced, thermal_synced);
	syncImage->registerCallback(std::bind(&KalibrSubscriber::syncedFrameCallback, this, std::placeholders::_1, std::placeholders::_2));
	RCLCPP_INFO(this->get_logger(), "Started Synced Image subscriber for KalibrSubscriber");

}

std::string repeat(std::string s, int n)
{
    // Copying given string to temporary string.
	std::string s1 = s;

    for (int i=1; i<n;i++)
        s += s1; // Concatenating strings

    return s;
}

void KalibrSubscriber::irFrameCallback(const sensor_msgs::msg::Image::SharedPtr infrared){
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

void KalibrSubscriber::syncedFrameCallback(
		const sensor_msgs::msg::Image::ConstSharedPtr &infrared,
		const sensor_msgs::msg::Image::ConstSharedPtr &thermal){
	RCLCPP_DEBUG(this->get_logger(), "Got Message");
	std::string infra_s = std::to_string(infrared->header.stamp.sec);
	std::string infra_ns = std::to_string(infrared->header.stamp.nanosec);
	std::string thermal_s = std::to_string(thermal->header.stamp.sec);
	std::string thermal_ns = std::to_string(thermal->header.stamp.nanosec);
	int isize = infra_ns.size();
	int tsize = thermal_ns.size();
	if(isize < 9){
		isize = infra_ns.size() - isize;
		infra_ns = repeat("0", isize) + infra_ns;
	}
	if(tsize < 9){
		tsize = thermal_ns.size() - tsize;
		thermal_ns = repeat("0", tsize) + thermal_ns;
	}
	if(count3 == 200){
		RCLCPP_INFO(this->get_logger(), "Inside Synced Frame Subscriber");
		RCLCPP_INFO(this->get_logger(), "Infrared Image timestamp    : " + infra_s + infra_ns);
		RCLCPP_INFO(this->get_logger(), "Thermal Image timestamp    : " + thermal_s + thermal_ns);
		count3 = 0;
	}
	count3++;
	cv_bridge::CvImagePtr cv_ptr_infrared = cv_bridge::toCvCopy(infrared, infrared->encoding);
	cv_bridge::CvImagePtr cv_ptr_thermal = cv_bridge::toCvCopy(thermal, thermal->encoding);
	ShowManyImages("All Image Messages in Synced Frame Callback", 2, cv_ptr_infrared->image, cv_ptr_thermal->image);
	cv::waitKey(1);
	cv::imwrite(syncedLogDir + "infrared/" + infra_s + infra_ns + ".png", cv_ptr_infrared->image);
	cv::imwrite(syncedLogDir + "thermal/" + thermal_s + thermal_ns + ".png", cv_ptr_thermal->image);
}

void KalibrSubscriber::imuCallback(
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

bool KalibrSubscriber::logIMUToFile(std::string log){
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

