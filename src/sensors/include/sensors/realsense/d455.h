#include "sensors/common/sensor.h"
#include <librealsense2/rs.hpp>
#include "sensors/common/types.h"
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <map>
#include <tuple>

typedef std::tuple<std::string, std::string, int> device_id;
typedef std::tuple<int, rs2_stream, rs2_format, device_id> sensor_id;

class RealSenseSensor: public rs2::sensor{
	public:
		RealSenseSensor(rs2::sensor sensor, device_id did);
		bool start(std::function<void(rs2::frame, sensor_id)> cb);
		void stop();
		bool is_initialized = false;
		bool is_started = false;
		bool is_video = true;
		device_id dev_id;
		sensor_id id;
		std::vector<rs2::stream_profile> profiles;
	private:
		std::function<void(rs2::frame, sensor_id)> originalCallback;
		std::function<void(rs2::frame)> callback;
};

class RealSenseSensorProperties{
	public:
		int frameWidth = 640;
		int frameHeight = 480;
		int frameRate = 15;
		rs2_format frameType = RS2_FORMAT_Y8;
		rs2_stream streamDataType = RS2_STREAM_INFRARED;
		bool isVideo = true;
		rs2::stream_profile profile;
		int deviceID = 0;
		device_id dev_id;
};

std::string getTopicNameFromProfile(rs2::stream_profile &profile, std::pair<std::string, std::string> dev_id);

class RealSense{
	public:
		RealSense();
		~RealSense();
		int LaunchSensorProcess();
		std::vector<RealSenseSensor*> queryAllSensors();
		void setup(std::vector<RealSenseSensorProperties> &props);
		bool start(std::vector<RealSenseSensorProperties> &props, std::function<void(Frame frame, sensor_id)> frameCallback, std::function<void(ImuPoint point, sensor_id)> imuCallback);
		std::map<sensor_id, std::pair<std::string, bool>> sensorTopicMap;
	private:
		std::map<rs2_format, int> _rs_format_to_cv_format;
		std::map<rs2_format, std::string> _rs_format_to_ros_format;
		void initializeFormatsMaps();
		rs2::context ctx;
		std::vector<rs2::device> devices;
		std::vector<RealSenseSensor*> sensors;
		//std::function<void(rs2::frame)> frameCallback;
		std::function<void(Frame frame, sensor_id)> originalFrameCallback;
		//std::function<void(rs2::frame)> imuCallback;
		std::function<void(ImuPoint point, sensor_id)> originalImuCallback;
		void querySensorOptionsAndChoose(RealSenseSensor* sensor,
				std::vector<RealSenseSensorProperties> &props);
		bool rs2FrameToSkyNetImageFrame(rs2::frame &frame, Frame &image);
		bool rs2FrameToSkyNetImuPoint(rs2::frame &frame, ImuPoint &point, sensor_id id);
		bool _is_frame_time_initialised = false;
		bool _is_imu_time_initialised = false;
		double _frame_base_time;
		double _imu_base_time;	
		uint64_t millisecondsToNanoseconds(double timestamp_ms);
		double systemTimeToNS(rs2::frame &frame, double base_time = 0.0);
};
