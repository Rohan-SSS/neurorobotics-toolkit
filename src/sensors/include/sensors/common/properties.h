#include <librealsense2/rs.hpp>

typedef std::pair<int, rs2_stream> sensor_id;

class SensorProperties{
	public:
		int frameWidth = 640;
		int frameHeight = 480;
		int frameRate = 15;
		rs2_format frameType = RS2_FORMAT_Y8;
		rs2_stream streamDataType = RS2_STREAM_INFRARED;
		bool isVideo = true;
};
