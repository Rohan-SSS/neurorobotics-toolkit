#include "sensors/video.h"

VideoFileReader::VideoFileReader(rclcpp::Logger logger, std::function<void(cv::Mat)> cb, std::string filePath): LOGGER(logger), originalCallback(cb){
	RCLCPP_INFO(LOGGER, "Creating Video File Reader");
	videoFilePath = filePath;		
	callback = [this](cv::Mat frame){
		RCLCPP_DEBUG(LOGGER, "Inside video file reader callback");
		originalCallback(frame);
	};
}

void VideoFileReader::start(){
	RCLCPP_INFO(LOGGER, "Starting Video Frame Publication");
	th = new std::thread(&VideoFileReader::AcquireVideoData, this);
}

void VideoFileReader::AcquireVideoData(){
	RCLCPP_INFO(LOGGER, "Starting Acquisition Thread for Video data");
	cv::VideoCapture cap(videoFilePath);
	if(cap.isOpened()){
		RCLCPP_INFO(LOGGER, "Successfully opened video file");
	}
	else{
		RCLCPP_ERROR(LOGGER, "Error in opening video file");
	}
	double fps = cap.get(cv::CAP_PROP_FPS);
	RCLCPP_INFO(LOGGER, "Video fps: %f", fps);
	cv::Mat frame;
	bool read_success;
	while(true){
		read_success = cap.read(frame);
		if(!read_success){
			RCLCPP_INFO(LOGGER, "Reached Video end");
			break;
		}
		else{
			RCLCPP_DEBUG(LOGGER, "received frame successfully");
			callback(frame);
		}
	}
}

int SimpleImagePublisher::initializeFormatsMaps(){
	_cv_format_to_ros_format[CV_8UC1] = sensor_msgs::image_encodings::MONO8; 
	_cv_format_to_ros_format[CV_16UC1] = sensor_msgs::image_encodings::MONO16;
	_cv_format_to_ros_format[CV_8UC3] = sensor_msgs::image_encodings::BGR8;
	_cv_format_to_ros_format[CV_8UC4] = sensor_msgs::image_encodings::BGRA8;
	_cv_format_to_ros_format[CV_8UC2] = sensor_msgs::image_encodings::YUV422;
	_cv_format_to_ros_format[CV_8UC1] = sensor_msgs::image_encodings::TYPE_8UC1;
	_cv_format_to_ros_format[CV_16UC1] = sensor_msgs::image_encodings::TYPE_16UC1;
	return 0;
}

SimpleImagePublisher::SimpleImagePublisher(std::string nodeName): rclcpp::Node(nodeName){

	initializeFormatsMaps();
	_frame_publisher = rclcpp::Node::create_publisher<sensor_msgs::msg::Image>("~/frame", 10);

	std::string p = "/ws/SkyNet/Flight Data/23 June 2023/FLIGHT 2/Log/2023_6_15_10_53_37/frames.avi";
	RCLCPP_INFO(this->get_logger(), "Creating Video Frame Publisher");
	std::function<void(cv::Mat)> callbackFunction = [this](cv::Mat frame){
		frameCallback(frame);
	};
	reader = new VideoFileReader(this->get_logger(), callbackFunction, p);
	reader->start();
}

void SimpleImagePublisher::frameCallback(cv::Mat image){

	sensor_msgs::msg::Image::UniquePtr img_msg_ptr(new sensor_msgs::msg::Image());
	cv_bridge::CvImage(
			std_msgs::msg::Header(),
			_cv_format_to_ros_format[image.type()],
			image
		).toImageMsg(*img_msg_ptr);
	img_msg_ptr->height = image.rows;
	img_msg_ptr->width = image.cols;
	img_msg_ptr->is_bigendian = false;
	img_msg_ptr->step = image.cols * image.elemSize();
	_frame_publisher->publish(std::move(img_msg_ptr));
}
