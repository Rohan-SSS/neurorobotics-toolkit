#include "sensors/video/common.h"
#include <gst/app/gstappsrc.h>

class VideoLogger{
	public:
		VideoLogger(rclcpp::Logger logger, std::string &outputPath, VideoProperties &props);
		~VideoLogger();
		void run();
		bool start();
		int getStatus(){return mpVideoLoggerStatus;}
		void logFrame(cv::Mat &image);

	private:
		rclcpp::Logger mpLogger;

		GstElement* mpPipeline;
		GstElement* mpAppsrc;
		GstElement* mpVideoconvert;
		GstElement* mpX264enc;
		GstElement* mpMp4mux;
		GstElement* mpFilesink;
		GstMapInfo mpMap;
		
		std::string mpOutputFilePath;
		int mpOutputImageWidth;
		int mpOutputImageHeight;
		int mpNumChannels;
		int mpInputDataFPS;
		int mpOutputFrameSize;
		int mpVideoLoggerStatus = 0;
		int mpFrameCounter;

};

class VideoLoggerNode: public rclcpp_lifecycle::LifecycleNode{
	public:
		VideoLoggerNode();
		void frameCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img);

        CallbackReturn on_configure(const rclcpp_lifecycle::State &);
        CallbackReturn on_activate(const rclcpp_lifecycle::State &);
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

	private:
		std::string mpOutputFilePath;
		std::string mpTopicName;
		std::string mpImageType;

		int mpOutputImageWidth;
		int mpOutputImageHeight;
		int mpInputDataFPS;
		int mpNumChannels;

		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mpFrameSubscriber;

		std::unique_ptr<VideoLogger> mpVideoLogger;
};
