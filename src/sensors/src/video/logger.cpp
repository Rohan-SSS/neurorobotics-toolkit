#include "sensors/video/logger.h"

VideoLogger::VideoLogger(rclcpp::Logger logger, std::string &outputPath, VideoProperties &props)
    : mpLogger(logger){ 
    mpOutputFilePath = outputPath;
    mpOutputImageWidth = props.mWidth;
    mpOutputImageHeight = props.mHeight;
    mpInputDataFPS = props.mFPS;
    mpOutputFrameSize = mpOutputImageWidth * mpOutputImageHeight * mpNumChannels;
    mpFrameCounter = 0;
    
    gst_init(nullptr, nullptr);

    mpPipeline = gst_pipeline_new("video-logger-pipeline");
    mpAppsrc = gst_element_factory_make("appsrc", "mysource");
    mpVideoconvert = gst_element_factory_make("videoconvert", "myconvert");
    mpX264enc = gst_element_factory_make("x264enc", "myencoder");
    mpMp4mux = gst_element_factory_make("mp4mux", "mymux");
    mpFilesink = gst_element_factory_make("filesink", "myfileoutput");

    check_element_creation(mpPipeline, "pipeline");
    check_element_creation(mpAppsrc, "appsrc");
    check_element_creation(mpVideoconvert, "videoconvert");
    check_element_creation(mpX264enc, "x264encoder");
    check_element_creation(mpMp4mux, "mp4muxer");
    check_element_creation(mpFilesink, "filesink");

    if (!mpPipeline || !mpAppsrc || !mpVideoconvert || !mpX264enc || !mpMp4mux || !mpFilesink) {
        RCLCPP_ERROR(mpLogger, "Not all elements could be created.");
        mpVideoLoggerStatus = -1;
        return;
    }

    cv::namedWindow("Video Logger Output", cv::WINDOW_AUTOSIZE);
    RCLCPP_INFO(mpLogger, "Display window initialized");

    mpVideoLoggerStatus = 1;
    g_object_set(mpFilesink, "location", mpOutputFilePath.c_str(), NULL);

    gst_bin_add_many(GST_BIN(mpPipeline), mpAppsrc, mpVideoconvert, mpX264enc, mpMp4mux, mpFilesink, NULL);

    if (!gst_element_link_many(mpAppsrc, mpVideoconvert, mpX264enc, mpMp4mux, mpFilesink, NULL)) {
        RCLCPP_ERROR(mpLogger, "Elements could not be linked");
        mpVideoLoggerStatus = -1;
        return;
    }

    std::string stringCaps = "video/x-raw, format=(string)BGR, width=(int)" + 
                            std::to_string(mpOutputImageWidth) + 
                            ", height=(int)" + std::to_string(mpOutputImageHeight) + 
                            ", framerate=(fraction)" + std::to_string(mpInputDataFPS) + "/1";

    GstCaps* caps = gst_caps_from_string(stringCaps.c_str());
    g_object_set(mpAppsrc, "caps", caps, nullptr);
    gst_caps_unref(caps);
    g_object_set(mpAppsrc, "format", GST_FORMAT_TIME, nullptr);
}

bool VideoLogger::start() {
    GstStateChangeReturn ret = gst_element_set_state(mpPipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        return false;
    }
    return true;
}

VideoLogger::~VideoLogger() {
    gst_app_src_end_of_stream(GST_APP_SRC(mpAppsrc));

    GstBus *bus = gst_element_get_bus(mpPipeline);
    GstMessage *msg = gst_bus_timed_pop_filtered(bus, GST_SECOND * 10,
        static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

    if (msg != NULL) {
        gst_message_unref(msg);
    }
    gst_object_unref(bus);
    gst_element_set_state(mpPipeline, GST_STATE_NULL);
    gst_object_unref(mpPipeline);
	
	cv::destroyWindow("Video Logger Output");
}

void VideoLogger::logFrame(cv::Mat &image) {
    if (image.empty()) {
        RCLCPP_WARN(mpLogger, "Received empty image frame");
        return;
    }
    cv::Mat display_image = image;

    GstBuffer *buf = gst_buffer_new_and_alloc(mpOutputFrameSize * image.elemSize());
    if (!buf) {
        RCLCPP_ERROR(mpLogger, "Failed to allocate buffer");
        return;
    }
    GstMapInfo map;
    if (!gst_buffer_map(buf, &map, GST_MAP_WRITE)) {
        RCLCPP_ERROR(mpLogger, "Failed to map buffer");
        gst_buffer_unref(buf); 
        return;
    }

    memcpy(map.data, image.data, mpOutputFrameSize * image.elemSize());

    GstClockTime current_time = gst_clock_get_time(gst_system_clock_obtain());
    GST_BUFFER_PTS(buf) = current_time;
    GST_BUFFER_DTS(buf) = GST_BUFFER_PTS(buf);
    GST_BUFFER_DURATION(buf) = gst_util_uint64_scale_int(1, GST_SECOND, mpInputDataFPS);

    gst_buffer_unmap(buf, &map);

    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(mpAppsrc), buf);
    if (ret != GST_FLOW_OK) {
        RCLCPP_ERROR(mpLogger, "Failed to push buffer: %s", gst_flow_get_name(ret));
        gst_buffer_unref(buf);  
        return;
    }

    try {
        cv::imshow("Video Logger Output", display_image);
        cv::waitKey(30);  
    } catch (const cv::Exception& e) {
        RCLCPP_WARN_ONCE(mpLogger, "Display error: %s", e.what());
    }

    mpFrameCounter++;
}

VideoLoggerNode::VideoLoggerNode() : rclcpp_lifecycle::LifecycleNode("video_logger"){

	ParameterDescriptor outputFilePathDesc;
	outputFilePathDesc.description = "Path to output file for received video";
	outputFilePathDesc.type = 4;
	this->declare_parameter<std::string>("output_file_path", "/ws/data/output.mp4", outputFilePathDesc);

	ParameterDescriptor widthDesc;
	widthDesc.description = "Width of output image";
	widthDesc.type = 2;
	this->declare_parameter<int>("width", 640, widthDesc);

	ParameterDescriptor heightDesc;
	heightDesc.description = "Height of output image";
	heightDesc.type = 2;
	this->declare_parameter<int>("height", 480, heightDesc);

	ParameterDescriptor fpsDesc;
	fpsDesc.description = "FPS of input image stream";
	fpsDesc.type = 2;
	this->declare_parameter<int>("fps", 30, fpsDesc);

	ParameterDescriptor topicNameDesc;
	topicNameDesc.description = "Name of camera topic to subscribe to";
	topicNameDesc.type = 4;
	this->declare_parameter<std::string>("camera_topic", "/camera", topicNameDesc);

	ParameterDescriptor imageTypeDesc;
	imageTypeDesc.description = "Type of Image received by logger";
	imageTypeDesc.type = 4;
	this->declare_parameter<std::string>("image_type", "bgr", imageTypeDesc);

}

CallbackReturn VideoLoggerNode::on_configure(const rclcpp_lifecycle::State &){
	mpOutputFilePath = this->get_parameter("output_file_path").as_string();
	mpOutputImageWidth = this->get_parameter("width").as_int();
	mpOutputImageHeight = this->get_parameter("height").as_int();
	mpInputDataFPS = this->get_parameter("fps").as_int();
	mpTopicName = this->get_parameter("camera_topic").as_string();
	mpImageType = this->get_parameter("image_type").as_string();
	if(mpImageType.compare("bgr") == 0){
		mpNumChannels = 3;	
	}

	VideoProperties props;
	props.mWidth = mpOutputImageWidth;
	props.mHeight = mpOutputImageHeight;
	props.mNumChannels = mpNumChannels;
	props.mFPS = mpInputDataFPS;

	mpVideoLogger = std::make_unique<VideoLogger>(this->get_logger(), mpOutputFilePath, props);

	mpFrameSubscriber = this->create_subscription<sensor_msgs::msg::Image>(
		mpTopicName.c_str(),
		10,
		std::bind(&VideoLoggerNode::frameCallback, this, std::placeholders::_1)
	);

	RCLCPP_INFO(this->get_logger(), "Created Video Logger Node");
	return CallbackReturn::SUCCESS;
}

CallbackReturn VideoLoggerNode::on_activate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(this->get_logger(), "Starting Video Logger Node");
    
    if (!mpVideoLogger) {
        RCLCPP_ERROR(this->get_logger(), "Video Logger instance not initialized");
        return CallbackReturn::ERROR;
    }
    
    if (!mpVideoLogger->start()) {
        RCLCPP_ERROR(this->get_logger(), "Could not start Video Logger Node");
        return CallbackReturn::ERROR;
    }
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn VideoLoggerNode::on_deactivate(const rclcpp_lifecycle::State &){
    RCLCPP_INFO(this->get_logger(), "Video Logger Node deactivated");
	return CallbackReturn::SUCCESS;
}

CallbackReturn VideoLoggerNode::on_cleanup(const rclcpp_lifecycle::State &){
	mpFrameSubscriber.reset();
	mpVideoLogger.reset();
    RCLCPP_INFO(this->get_logger(), "Video Logger Node cleaned");
	return CallbackReturn::SUCCESS;
}

CallbackReturn VideoLoggerNode::on_shutdown(const rclcpp_lifecycle::State &){
	return CallbackReturn::SUCCESS;
}

void VideoLoggerNode::frameCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img) {
    try {
        cv_bridge::CvImagePtr cv_ptr_img = cv_bridge::toCvCopy(img, img->encoding);
        
        if (!mpVideoLogger) {
            RCLCPP_ERROR(this->get_logger(), "Video logger not initialized");
            return;
        }

        if (!cv_ptr_img->image.empty()) {
            cv::Mat image;
            if (cv_ptr_img->image.rows == mpOutputImageHeight && 
                cv_ptr_img->image.cols == mpOutputImageWidth) {
                image = cv_ptr_img->image.clone();
            } else {
                cv::resize(cv_ptr_img->image, image, 
                          cv::Size(mpOutputImageWidth, mpOutputImageHeight));
            }
            mpVideoLogger->logFrame(image);
        }
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV error in callback: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in callback: %s", e.what());
    }
}