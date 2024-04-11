#include "opencv2/opencv.hpp"
#include <librealsense2/rs.hpp>

struct Frame{
	public:
		cv::Mat frame;
		double timestamp;
		std::string format;
		int height, width;
};

struct ImuPoint{
	public:
		cv::Point3d gyro;
		cv::Point3d accel;
		double timestamp;
};
