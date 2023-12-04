#include "sensors/realsense.h"

ROSSensor::ROSSensor(rs2::sensor sensor,
		std::function<void(rs2::frame)> cb): rs2::sensor(sensor), originalCallback(cb){
	std::cout<<"Creating ROS Sensor"<<std::endl;
	std::cout<<"creating sensor: "<<sensor.get_info(RS2_CAMERA_INFO_NAME)<<std::endl;
	callback = [this](rs2::frame frame){
		originalCallback(frame);
	};
}

bool ROSSensor::start(const std::vector<rs2::stream_profile> profiles){
	rs2::sensor::open(profiles);
	rs2::sensor::start(callback);
	return true;
}

void ROSSensor::stop(){
	rs2::sensor::stop();
}

RealSense::RealSense(std::function<void(rs2::frame)> frameCallback, std::function<void(rs2::frame)> imuCallback): originalFrameCallback(frameCallback), originalImuCallback(imuCallback){
	std::cout<<"Creating Real Sense Devices Hub"<<std::endl;

	std::function<void(rs2::frame)> _frameCallback = [this](rs2::frame frame){
		originalFrameCallback(frame);
	};
	
	std::function<void(rs2::frame)> _imuCallback = [this](rs2::frame frame){
		originalImuCallback(frame);
	};

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
	for(auto&& dev : ctx.query_devices()){
		devices.push_back(dev);
		for(auto&& sensor: dev.query_sensors()){
			ROSSensor* rosSensor;	
			if(sensor.is<rs2::depth_sensor>() || sensor.is<rs2::color_sensor>()){
				std::cout<<"setting frame callback"<<std::endl;
				rosSensor = new ROSSensor(sensor, _frameCallback);
			}
			else if(sensor.is<rs2::motion_sensor>()){	
				std::cout<<"setting imu callback"<<std::endl;
				rosSensor = new ROSSensor(sensor, _imuCallback);
			}
			else{
				std::cout<<"no callback set for a sensor: "<<sensor.get_info(RS2_CAMERA_INFO_NAME)<<std::endl;
				continue;
			}
			sensors.push_back(std::make_pair(
						rosSensor, std::make_pair(
							dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER),
							dev.get_info(RS2_CAMERA_INFO_NAME))
						));
		}
	}
	std::cout<<"Device Querying Done"<<std::endl;
}

std::vector<std::pair<ROSSensor*, std::pair<std::string, std::string>>> RealSense::queryAllSensors(){
	return sensors;
}

RealSenseNode::RealSenseNode(std::string nodeName, std::vector<SensorProperties> &props) : Node(nodeName){
	//TODO Node configuration according to mode of data that can be published
	std::cout<<"Creating Real Sense Node"<<std::endl;
	std::function<void(rs2::frame)> frameCallbackFunction = [this](rs2::frame frame){
		frameCallback(frame);
	};
	std::function<void(rs2::frame)> imuCallbackFunction = [this](rs2::frame frame){
		imuCallback(frame);
	};
	hub = new RealSense(frameCallbackFunction, imuCallbackFunction);	
	initializeFormatsMaps();
	setup(props);
}

sensor_id RealSenseNode::getSensorID(rs2::stream_profile &stream_profile){
	int stream_index = stream_profile.stream_index();
	rs2_stream stream_type = stream_profile.stream_type();
	sensor_id id = std::make_pair(stream_index, stream_type);
	return id;
}


void RealSenseNode::initializeFormatsMaps()
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


uint64_t RealSenseNode::millisecondsToNanoseconds(double timestamp_ms)
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

rclcpp::Time RealSenseNode::frameSystemTimeSec(rs2::frame frame)
{
    double timestamp_ms = frame.get_timestamp();
    if (frame.get_frame_timestamp_domain() == RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK)
    {
        double elapsed_camera_ns = millisecondsToNanoseconds(timestamp_ms - _frame_base_time);

        auto duration = rclcpp::Duration(elapsed_camera_ns);

        return rclcpp::Time(_ros_time_base + duration);
    }
    else
    {
        return rclcpp::Time(millisecondsToNanoseconds(timestamp_ms));
    }
}


rclcpp::Time RealSenseNode::imuSystemTimeSec(rs2::frame frame)
{
    double timestamp_ms = frame.get_timestamp();
    if (frame.get_frame_timestamp_domain() == RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK)
    {
        double elapsed_camera_ns = millisecondsToNanoseconds(timestamp_ms - _imu_base_time);

        auto duration = rclcpp::Duration(elapsed_camera_ns);

        return rclcpp::Time(_ros_time_base + duration);
    }
    else
    {
        return rclcpp::Time(millisecondsToNanoseconds(timestamp_ms));
    }
}

void RealSenseNode::frameCallback(rs2::frame frame){
	RCLCPP_DEBUG(this->get_logger(), "frame arrived");
	rs2::stream_profile stream_profile = frame.get_profile();
	sensor_id id = getSensorID(stream_profile);
	if(!_is_frame_time_initialised){
		_is_frame_time_initialised = true;
		_ros_time_base = this->now();
		_frame_base_time = frame.get_timestamp();
	}
	rclcpp::Time t(frameSystemTimeSec(frame));
	if(_frame_publishers.find(id) != _frame_publishers.end()){
		if(frame.is<rs2::frameset>()){
			RCLCPP_DEBUG(this->get_logger(), "frame set available");
		}
		else{
			rs2::video_frame video = frame.as<rs2::video_frame>();
			cv::Mat image(video.get_height(), video.get_width(), _rs_format_to_cv_format[stream_profile.format()]);
			image.data = (uint8_t*)frame.get_data();
			if(!image.empty()){
				sensor_msgs::msg::Image::UniquePtr img_msg_ptr(new sensor_msgs::msg::Image());
				cv_bridge::CvImage(
						std_msgs::msg::Header(),
						_rs_format_to_ros_format[stream_profile.format()],
						image
					).toImageMsg(*img_msg_ptr);
				img_msg_ptr->header.stamp = t;
				img_msg_ptr->height = video.get_height();
				img_msg_ptr->width = video.get_width();
				img_msg_ptr->is_bigendian = false;
				img_msg_ptr->step = video.get_width() * image.elemSize();
				_frame_publishers[id]->publish(std::move(img_msg_ptr));

				RCLCPP_DEBUG(this->get_logger(), "frame available");
			}
		}
	}
	else{
		RCLCPP_ERROR_SKIPFIRST(this->get_logger(), "no imu publisher available for the frame");
	}
}

void RealSenseNode::imuCallback(rs2::frame frame){
	rs2::stream_profile stream_profile = frame.get_profile();
	sensor_id id = getSensorID(stream_profile);
	if(!_is_imu_time_initialised){
		_is_imu_time_initialised = true;
		_imu_base_time = frame.get_timestamp();
	}
	rclcpp::Time t(frameSystemTimeSec(frame));
	if(_imu_publishers.find(id) != _imu_publishers.end()){
		rs2::motion_frame motion = frame.as<rs2::motion_frame>();
		//_imu_publishers[id].publish();
		//double frameTime = frame.get_timestamp();
		RCLCPP_DEBUG(this->get_logger(), "IMU Frame arrived");
		sensor_msgs::msg::Imu imu_msg = sensor_msgs::msg::Imu();
		imu_msg.header.stamp = t;
		// Default values for unnecessary members
		imu_msg.orientation.x = 0.0;
		imu_msg.orientation.y = 0.0;
		imu_msg.orientation.z = 0.0;
		imu_msg.orientation.w = 0.0;

		imu_msg.orientation_covariance = { -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		imu_msg.linear_acceleration_covariance = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		imu_msg.angular_velocity_covariance = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		rs2_vector data = motion.get_motion_data();
		if(id.second == RS2_STREAM_GYRO){
			RCLCPP_DEBUG(this->get_logger(), "Got Gyro Data");
			imu_msg.angular_velocity.x = data.x;
			imu_msg.angular_velocity.y = data.y;
			imu_msg.angular_velocity.z = data.z;
		}
		else if(id.second == RS2_STREAM_ACCEL){
			RCLCPP_DEBUG(this->get_logger(), "Got Accel Data");
			imu_msg.linear_acceleration.x = data.x;
			imu_msg.linear_acceleration.y = data.y;
			imu_msg.linear_acceleration.z = data.z;
		}
		_imu_publishers[id]->publish(imu_msg);
	}
	else{
		RCLCPP_ERROR_SKIPFIRST(this->get_logger(), "no imu publisher available for the frame");
	}
}


std::string getTopicNameFromProfile(rs2::stream_profile &profile, std::pair<std::string, std::string> dev_id){
	rs2_stream stream_data_type = profile.stream_type();
	std::string name = dev_id.second + "/", sensor_name = "", sensor_type = "", topic_name = "";
	if(profile.is<rs2::video_stream_profile>()){
		sensor_name = "camera/";
		if(stream_data_type == RS2_STREAM_INFRARED){
			sensor_type = "infrared";
		}
		else if(stream_data_type == RS2_STREAM_DEPTH){
			sensor_type = "depth";
		}
		else if(stream_data_type == RS2_STREAM_COLOR){
			sensor_type = "rgb";
		}
		else if(stream_data_type == RS2_STREAM_FISHEYE){
			sensor_type = "fisheye";
		}
		else{
			std::cerr<<"stream_data_type:<< "<<stream_data_type<<" not supported"<<std::endl;
		}
		topic_name = "~/" + name + sensor_name + sensor_type;
		std::cout<<"Topic Name for Stream Profile: "<<topic_name<<std::endl;	
		//this->_frame_publishers[id] = this->create_publisher<rclcpp::Publisher<sensor_msgs::msg::Image>>(topic_name, 10);
	}
	else if(profile.is<rs2::motion_stream_profile>()){
		sensor_name = "motion/";
		if(stream_data_type == RS2_STREAM_GYRO){
			sensor_type = "gyro";	
		}
		else if(stream_data_type == RS2_STREAM_ACCEL){
			sensor_type = "accel";
		}
		else{
			std::cerr<<"stream_data_type: "<<stream_data_type<<" not supported"<<std::endl;
		}
		topic_name = "~/" + name + sensor_name + sensor_type;
		std::cout<<"Topic Name for Stream Profile: "<<topic_name<<std::endl;	
		//_imu_publishers[id] = this->create_publisher<rclcpp::Publisher<sensor_msgs::msg::Image>>(topic_name, 10);
	}
	else{
		std::cerr<<"Pose not supported"<<std::endl;
	}
	return topic_name;
}

void RealSenseNode::addStreamProfile(rs2::stream_profile &profile, std::pair<std::string, std::string> dev_id){
	std::replace(dev_id.second.begin(), dev_id.second.end(), ' ', '_');
	const std::string topic_name = getTopicNameFromProfile(profile, dev_id);
	sensor_id id = std::make_pair(profile.stream_index(), profile.stream_type());
	if(profile.is<rs2::video_stream_profile>()){
		_frame_publishers[id] = rclcpp::Node::create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
	}
	else if(profile.is<rs2::motion_stream_profile>()){
		_imu_publishers[id] = rclcpp::Node::create_publisher<sensor_msgs::msg::Imu>(topic_name, 10);
	}
	else{
		std::cerr<<"Pose not supported"<<std::endl;
	}
	
}

void querySensorOptionsAndChoose(ROSSensor* sensor, std::vector<SensorProperties> &props, std::vector<rs2::stream_profile> &profiles){
	std::cout << std::endl;
	std::cout<<"starting sensor: "<<sensor->get_info(RS2_CAMERA_INFO_NAME)<<std::endl;
	std::vector<rs2::stream_profile> stream_profiles = sensor->get_stream_profiles();
	std::cout << "Sensor provides "<< stream_profiles.size()<<" stream profiles." << std::endl;

	for (rs2::stream_profile stream_profile : stream_profiles)
	{
		rs2_stream stream_data_type = stream_profile.stream_type();
		for(std::vector<SensorProperties>::iterator it = props.begin(); it != props.end();){
			SensorProperties prop = *it;
			if(stream_data_type == prop.streamDataType){
				//int stream_index = stream_profile.stream_index();
				std::string stream_name = stream_profile.stream_name();
				//int unique_stream_id = stream_profile.unique_id();
				if(stream_profile.format() == prop.frameType){
					if (stream_profile.is<rs2::video_stream_profile>() && prop.isVideo) 
					{
						// Video Streams Profile Setup
						rs2::video_stream_profile video_stream_profile = stream_profile.as<rs2::video_stream_profile>();
						if(video_stream_profile.width() == prop.frameWidth && video_stream_profile.height() == prop.frameHeight){
							if(video_stream_profile.fps() == prop.frameRate){
								//TODO add node publisher setup here as well
								//Ensure all setup for all stakeholders is synchronised
								std::cout << "Video Stream Data Type: "<< stream_data_type << " Format: " << video_stream_profile.format()<< " Unique ID: "<< stream_profile.unique_id() << " Resolution: " <<video_stream_profile.width() << "x" << video_stream_profile.height() << "FPS: " << video_stream_profile.fps() << "Hz "<< " stream index: "<<stream_profile.stream_index()<<std::endl;
								it = props.erase(it);
								profiles.push_back(stream_profile);
							}
							else{
								it++;
							}
						}
						else{
							it++;
						}
					}
					else if(stream_profile.is<rs2::motion_stream_profile>()){
						if(prop.frameRate == stream_profile.fps()){
							std::cout<<"Motion Stream Data Type: "<<stream_data_type<<" Format: "<<stream_profile.format()<<" Unique ID: "<<stream_profile.unique_id()<<"FPS: "<<stream_profile.fps()<<" stream index: "<<stream_profile.stream_index()<<std::endl;
							profiles.push_back(stream_profile);
							it = props.erase(it);
						}
						else{
							it++;
						}
					}
					else{
						it++;
					}
				}
				else{
					it++;
				}
			}
			else{
				it++;
			}
		}
	}
}

bool RealSenseNode::setup(std::vector<SensorProperties> &props){
	std::vector<rs2::stream_profile> profiles;
	for(auto& [sensor, dev_id]: hub->queryAllSensors()){
		profiles.clear();
		querySensorOptionsAndChoose(sensor, props, profiles);
		if(profiles.size() > 0){
			std::cout<<"number of profiles being pushed: "<<profiles.size()<<std::endl; 
			for(rs2::stream_profile sp: profiles){
				addStreamProfile(sp, dev_id);							
			}
			sensor->start(profiles);
		}
	}
	std::cout<<"Done Setup"<<std::endl;
	return true;
}
