#include "sensors/realsense/d455.h"

void querySensorOptionsAndChoose(ROSSensor* sensor,
		std::vector<SensorProperties> &props,
		std::vector<rs2::stream_profile> &profiles);

std::string getTopicNameFromProfile(rs2::stream_profile &profile, std::pair<std::string, std::string> dev_id);

class RealSenseNode: public rclcpp::Node{
	public:
		RealSenseNode(std::string node_name, std::vector<SensorProperties> &props);
		bool setup(std::vector<SensorProperties> &props);
	private:
		bool _is_frame_time_initialised = false;
		bool _is_imu_time_initialised = false;
		double _frame_base_time;
		double _imu_base_time;
		rclcpp::Time _ros_time_base;
		

		RealSense* hub;
		std::map<sensor_id, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> _frame_publishers;
		std::map<sensor_id, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> _imu_publishers;
		std::map<rs2_format, int> _rs_format_to_cv_format;
		std::map<rs2_format, std::string> _rs_format_to_ros_format;

		void frameCallback(rs2::frame);
		void imuCallback(rs2::frame);
		void addStreamProfile(rs2::stream_profile &profile,
				std::pair<std::string, std::string> dev_id);
		sensor_id getSensorID(rs2::stream_profile &stream_profile);
		void initializeFormatsMaps();
		uint64_t millisecondsToNanoseconds(double timestamp_ms);
		rclcpp::Time frameSystemTimeSec(rs2::frame frame);
		rclcpp::Time imuSystemTimeSec(rs2::frame frame);
};
