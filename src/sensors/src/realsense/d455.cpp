#include "sensors/realsense/d455.h"

RealSenseSensor::RealSenseSensor(rs2::sensor sensor, device_id did): rs2::sensor(sensor), dev_id(did){
	std::cout<<"Creating ROS Sensor"<<std::endl;
	std::cout<<"creating sensor: "<<sensor.get_info(RS2_CAMERA_INFO_NAME)<<std::endl;
}

bool RealSenseSensor::start(std::function<void(rs2::frame, sensor_id)> cb){
	originalCallback = cb;
	callback = [this](rs2::frame frame){
		rs2::stream_profile profile = frame.get_profile();
		id = std::make_tuple(profile.stream_index(), profile.stream_type(), profile.format(), dev_id);
		originalCallback(frame, id);
	};
	rs2::sensor::open(profiles);
	rs2::sensor::start(callback);
	std::cout<<"Started RealSense Sensor with: "<< profiles.size()<< " profiles"<<std::endl;
	return true;
}

void RealSenseSensor::stop(){
	rs2::sensor::stop();
}

std::string getTopicNameFromProfile(rs2::stream_profile &profile, device_id dev_id){
	rs2_stream stream_data_type = profile.stream_type();
	std::string name = std::get<1>(dev_id) + "/", sensor_name = "", sensor_type = "", topic_name = "";
	std::string stream_index = "stream_" + std::to_string(profile.stream_index());
	std::string id = "device_" + std::to_string(std::get<2>(dev_id)) + "/";
	std::cout<<"Device ID for topic: "<< id <<std::endl;
	if(profile.is<rs2::video_stream_profile>()){
		std::cout<<"creating topic for a video stream"<<std::endl;
		sensor_name = "camera/";
		if(stream_data_type == RS2_STREAM_INFRARED){
			sensor_type = "infrared/";
		}
		else if(stream_data_type == RS2_STREAM_DEPTH){
			sensor_type = "depth/";
		}
		else if(stream_data_type == RS2_STREAM_COLOR){
			sensor_type = "rgb/";
		}
		else if(stream_data_type == RS2_STREAM_FISHEYE){
			sensor_type = "fisheye/";
		}
		else{
			std::cerr<<"stream_data_type:<< "<<stream_data_type<<" not supported"<<std::endl;
		}
		topic_name = "~/" + name + id + sensor_name + sensor_type + stream_index;
		std::cout<<"Topic Name for Stream Profile: "<<topic_name<<std::endl;	
		//this->_frame_publishers[id] = this->create_publisher<rclcpp::Publisher<sensor_msgs::msg::Image>>(topic_name, 10);
	}
	else if(profile.is<rs2::motion_stream_profile>()){
		sensor_name = "motion/";
		if(stream_data_type == RS2_STREAM_GYRO){
			sensor_type = "gyro/";	
		}
		else if(stream_data_type == RS2_STREAM_ACCEL){
			sensor_type = "accel/";
		}
		else{
			std::cerr<<"stream_data_type: "<<stream_data_type<<" not supported"<<std::endl;
		}
		topic_name = "~/" + name + id + sensor_name + sensor_type + stream_index;
		std::cout<<"Topic Name for Stream Profile: "<<topic_name<<std::endl;	
		//_imu_publishers[id] = this->create_publisher<rclcpp::Publisher<sensor_msgs::msg::Image>>(topic_name, 10);
	}
	else{
		std::cerr<<"Pose not supported"<<std::endl;
	}
	return topic_name;
}

RealSense::RealSense(){
	std::cout<<"Creating Real Sense Devices Hub"<<std::endl;
	initializeFormatsMaps();
}

void RealSense::querySensorOptionsAndChoose(RealSenseSensor* sensor, std::vector<RealSenseSensorProperties> &props){
	std::cout << std::endl;
	std::cout<<"starting sensor: "<<sensor->get_info(RS2_CAMERA_INFO_NAME)<<std::endl;
	std::vector<rs2::stream_profile> stream_profiles = sensor->get_stream_profiles();
	std::cout << "Sensor provides "<< stream_profiles.size()<<" stream profiles." << std::endl;

	for (rs2::stream_profile stream_profile : stream_profiles)
	{
		rs2_stream stream_data_type = stream_profile.stream_type();
		for(RealSenseSensorProperties &prop: props){
			if(stream_data_type == prop.streamDataType){
				//int stream_index = stream_profile.stream_index();
				std::string stream_name = stream_profile.stream_name();
				//int unique_stream_id = stream_profile.unique_id();
				if(stream_profile.format() == prop.frameType){
					if (stream_profile.is<rs2::video_stream_profile>())
					{
						// Video Streams Profile Setup
						rs2::video_stream_profile video_stream_profile = stream_profile.as<rs2::video_stream_profile>();
						if(video_stream_profile.width() == prop.frameWidth && video_stream_profile.height() == prop.frameHeight){
							if(video_stream_profile.fps() == prop.frameRate){
								if(std::find(sensor->profiles.begin(),
											sensor->profiles.end(),
											stream_profile) == sensor->profiles.end()){
									//TODO add node publisher setup here as well
									//Ensure all setup for all stakeholders is synchronised
									std::cout << "Video Stream Data Type: "<< stream_data_type;
									std::cout << " Format: " << video_stream_profile.format();
									std::cout << " Unique ID: " << stream_profile.unique_id();
									std::cout << " Resolution: " << video_stream_profile.width();
									std::cout << "x" << video_stream_profile.height();
									std::cout << "FPS: " << video_stream_profile.fps() << "Hz";
									std::cout<<" Stream Type: "<<stream_profile.stream_type();
									std::cout << " stream index: " << stream_profile.stream_index()<<std::endl;
									sensor->profiles.push_back(stream_profile);
									sensor_id id = std::make_tuple(stream_profile.stream_index(), stream_profile.stream_type(), stream_profile.format(), sensor->dev_id);
									std::replace(std::get<1>(sensor->dev_id).begin(), std::get<1>(sensor->dev_id).end(), ' ', '_');
									sensorTopicMap[id] = std::make_pair(getTopicNameFromProfile(stream_profile, sensor->dev_id), true);
									std::replace(std::get<1>(sensor->dev_id).begin(), std::get<1>(sensor->dev_id).end(), '_', ' ');
									sensor->is_initialized = true;
									sensor->is_video = true;
									prop.isVideo = true;
								}
							}
						}
					}
					else if(stream_profile.is<rs2::motion_stream_profile>()){
						if(prop.frameRate == stream_profile.fps()){
							if(std::find(sensor->profiles.begin(),
										sensor->profiles.end(),
										stream_profile) == sensor->profiles.end()){
								std::cout<<"Motion Stream Data Type: "<<stream_data_type;
								std::cout<<" Format: "<<stream_profile.format();
								std::cout<<" Unique ID: "<<stream_profile.unique_id();
								std::cout<<" FPS: "<<stream_profile.fps();
								std::cout<<" Stream Type: "<<stream_profile.stream_type();
								std::cout<<" stream index: "<<stream_profile.stream_index()<<std::endl;
								sensor->profiles.push_back(stream_profile);
								sensor_id id = std::make_tuple(stream_profile.stream_index(), stream_profile.stream_type(), stream_profile.format(), sensor->dev_id);
								std::replace(std::get<1>(sensor->dev_id).begin(), std::get<1>(sensor->dev_id).end(), ' ', '_');
								sensorTopicMap[id] = std::make_pair(getTopicNameFromProfile(stream_profile, sensor->dev_id), false);
								std::replace(std::get<1>(sensor->dev_id).begin(), std::get<1>(sensor->dev_id).end(), '_', ' ');
								sensor->is_initialized = true;
								sensor->is_video = false;
								prop.isVideo = false;
							}
						}
					}
				}
			}
		}
	}
}

void RealSense::setup(std::vector<RealSenseSensorProperties> &props){
	std::cout<<"Setting Up Real Sense Devices Hub"<<std::endl;

	// std::function<void(rs2::frame, device_id)> _frameCallback = [this](rs2::frame frame){
	// 	originalFrameCallback(frame);
	// };
	//
	// std::function<void(rs2::frame, device_id)> _imuCallback = [this](rs2::frame frame){
	// 	originalImuCallback(frame);
	// };

  	auto _device_changed_cb = [this](rs2::event_information& info) {
		for(rs2::device& dev: devices){
			if(info.was_removed(dev)){
				std::cout<<dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)<<" Device Disconnected"<<std::endl;
			}
		}
		// TODO Check if need to put the following here:
		//queryAndConnect();
	};
	ctx.set_devices_changed_callback(_device_changed_cb);

	// Query and Connect a single RealSense Device to appropriate pipelines
	int index = 0;
	for(auto&& dev : ctx.query_devices()){
		devices.push_back(dev);
		device_id dev_id = std::make_tuple(
							dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER),
							dev.get_info(RS2_CAMERA_INFO_NAME),
							index);
		std::cout<<"Number of sensors in device "<<index<< " are "<<dev.query_sensors().size()<<std::endl;
		for(auto&& sensor: dev.query_sensors()){
			RealSenseSensor* rosSensor = new RealSenseSensor(sensor, dev_id);	
			sensors.push_back(rosSensor);
		}
	}
	std::cout<<"Device Querying Done"<<std::endl;
	bool status;
	for(RealSenseSensor* sensor: sensors){
		querySensorOptionsAndChoose(sensor, props);
	}
	for(auto it = sensors.begin(); it < sensors.end();){
		RealSenseSensor* sensor = *it;
		if(sensor->is_initialized){
			it++;	
		}
		else{
			it = sensors.erase(it);
		}
	}
	std::cout<<"Done Setup"<<std::endl;
}

std::vector<RealSenseSensor*> RealSense::queryAllSensors(){
	return sensors;
}

void RealSense::initializeFormatsMaps()
{
    // from rs2_format to OpenCV format
    // https://docs.opencv.org/3.4/d1/d1b/group__core__hal__interface.html
    // https://docs.opencv.org/2.4/modules/core/doc/basic_structures.html
    // CV_<bit-depth>{U|S|F}C(<number_of_channels>)
    // where U is unsigned integer type, S is signed integer type, and F is float type.
    // For example, CV_8UC1 means a 8-bit single-channel array,
    // CV_32FC2 means a 2-channel (complex) floating-point array, and so on.
    _rs_format_to_cv_format[RS2_FORMAT_Y8] = CV_8UC1;
    _rs_format_to_cv_format[RS2_FORMAT_Y16] = CV_16UC1;
    _rs_format_to_cv_format[RS2_FORMAT_Z16] = CV_16UC1;
    _rs_format_to_cv_format[RS2_FORMAT_RGB8] = CV_8UC3;
    _rs_format_to_cv_format[RS2_FORMAT_BGR8] = CV_8UC3;
    _rs_format_to_cv_format[RS2_FORMAT_RGBA8] = CV_8UC4;
    _rs_format_to_cv_format[RS2_FORMAT_BGRA8] = CV_8UC4;
    _rs_format_to_cv_format[RS2_FORMAT_YUYV] = CV_8UC2;
    _rs_format_to_cv_format[RS2_FORMAT_UYVY] = CV_8UC2;
    // _rs_format_to_cv_format[RS2_FORMAT_M420] = not supported yet in ROS2
    _rs_format_to_cv_format[RS2_FORMAT_RAW8] = CV_8UC1;
    _rs_format_to_cv_format[RS2_FORMAT_RAW10] = CV_16UC1;
    _rs_format_to_cv_format[RS2_FORMAT_RAW16] = CV_16UC1;

    // from rs2_format to ROS2 image msg encoding (format)
    // http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html
    // http://docs.ros.org/en/jade/api/sensor_msgs/html/image__encodings_8h_source.html
    _rs_format_to_ros_format[RS2_FORMAT_Y8] = sensor_msgs::image_encodings::MONO8;
    _rs_format_to_ros_format[RS2_FORMAT_Y16] = sensor_msgs::image_encodings::MONO16;
    _rs_format_to_ros_format[RS2_FORMAT_Z16] = sensor_msgs::image_encodings::TYPE_16UC1;
    _rs_format_to_ros_format[RS2_FORMAT_RGB8] = sensor_msgs::image_encodings::RGB8;
    _rs_format_to_ros_format[RS2_FORMAT_BGR8] = sensor_msgs::image_encodings::BGR8;
    _rs_format_to_ros_format[RS2_FORMAT_RGBA8] = sensor_msgs::image_encodings::RGBA8;
    _rs_format_to_ros_format[RS2_FORMAT_BGRA8] = sensor_msgs::image_encodings::BGRA8;
    _rs_format_to_ros_format[RS2_FORMAT_YUYV] = sensor_msgs::image_encodings::YUV422_YUY2;
    _rs_format_to_ros_format[RS2_FORMAT_UYVY] = sensor_msgs::image_encodings::YUV422;
    // _rs_format_to_ros_format[RS2_FORMAT_M420] =  not supported yet in ROS2
    _rs_format_to_ros_format[RS2_FORMAT_RAW8] = sensor_msgs::image_encodings::TYPE_8UC1;
    _rs_format_to_ros_format[RS2_FORMAT_RAW10] = sensor_msgs::image_encodings::TYPE_16UC1;
    _rs_format_to_ros_format[RS2_FORMAT_RAW16] = sensor_msgs::image_encodings::TYPE_16UC1;
}

uint64_t RealSense::millisecondsToNanoseconds(double timestamp_ms)
{
        // modf breaks input into an integral and fractional part
        double int_part_ms, fract_part_ms;
        fract_part_ms = modf(timestamp_ms, &int_part_ms);

        //convert both parts to ns
        static constexpr uint64_t milli_to_nano = 1000000;
        uint64_t int_part_ns = static_cast<uint64_t>(int_part_ms) * milli_to_nano;
        uint64_t fract_part_ns = static_cast<uint64_t>(std::round(fract_part_ms * milli_to_nano));

        return int_part_ns + fract_part_ns;
}

double RealSense::systemTimeToNS(rs2::frame &frame, double base_time){
    double timestamp_ms = frame.get_timestamp();
	double elapsed_camera_ns;
    if (frame.get_frame_timestamp_domain() == RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK)
    {
        elapsed_camera_ns = millisecondsToNanoseconds(timestamp_ms - base_time);
	}
	else{
        elapsed_camera_ns = millisecondsToNanoseconds(timestamp_ms);
	}
	return elapsed_camera_ns;
}

bool RealSense::rs2FrameToSkyNetImageFrame(rs2::frame &frame, Frame &image){
	rs2::stream_profile stream_profile = frame.get_profile();
	if(!_is_frame_time_initialised){
		_is_frame_time_initialised = true;
		_frame_base_time = frame.get_timestamp();
	}
	if(frame.is<rs2::frameset>()){
		return false;
	}
	else{
		image.timestamp = systemTimeToNS(frame, _frame_base_time);
		rs2::video_frame video = frame.as<rs2::video_frame>();
		cv::Mat im(video.get_height(), video.get_width(), _rs_format_to_cv_format[stream_profile.format()]);
		im.data = (uint8_t*)frame.get_data();
		image.frame = im;
		image.height = video.get_height();
		image.width = video.get_width();
		image.format = _rs_format_to_ros_format[stream_profile.format()];
		return true;
	}
}

bool RealSense::rs2FrameToSkyNetImuPoint(rs2::frame &frame, ImuPoint &point, sensor_id id){
	if(!_is_imu_time_initialised){
		_is_imu_time_initialised = true;
		_imu_base_time = frame.get_timestamp();
	}
	point.timestamp = systemTimeToNS(frame, _imu_base_time);
	rs2_stream stream_type = std::get<rs2_stream>(id);
	rs2::motion_frame motion = frame.as<rs2::motion_frame>();
	rs2_vector data = motion.get_motion_data();
	if(stream_type == RS2_STREAM_GYRO){
		point.gyro = cv::Point3d(data.x, data.y, data.z);
		point.accel = cv::Point3d(0, 0, 0);
		return true;
	}
	else if(stream_type == RS2_STREAM_ACCEL){
		point.accel = cv::Point3d(data.x, data.y, data.z);
		point.gyro = cv::Point3d(0, 0, 0);
		return true;
	}	
	return false;
}

bool RealSense::start(std::vector<RealSenseSensorProperties> &props, std::function<void(Frame frame, sensor_id)> frameCallback, std::function<void(ImuPoint point, sensor_id)> imuCallback){
	originalFrameCallback = frameCallback;
	originalImuCallback = imuCallback;
	std::function<void(rs2::frame, sensor_id id)> _frameCallback = [this](rs2::frame frame, sensor_id id){
		Frame image;
		rs2FrameToSkyNetImageFrame(frame, image);
		originalFrameCallback(image, id);
	}; 
	std::function<void(rs2::frame, sensor_id id)> _imuCallback = [this](rs2::frame frame, sensor_id id){
		ImuPoint point;
		rs2FrameToSkyNetImuPoint(frame, point, id);
		originalImuCallback(point, id);
	};
	int index = 0;
	for(RealSenseSensor* sensor: sensors){
		std::cout<<"Starting Sensor "<< index << std::endl;
		if(sensor->is_video){
			sensor->start(_frameCallback);
			sensor->is_started = true;
		}
		else{
			sensor->start(_imuCallback);
			sensor->is_started = true;
		}
		index++;
	}
	return true;
}
