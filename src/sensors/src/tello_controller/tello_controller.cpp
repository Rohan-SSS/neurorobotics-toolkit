#include <sensors/tello_controller/controller.h>
using namespace std::chrono_literals;
TelloControllerNode::TelloControllerNode(std::string nodeName):Node(nodeName){
    //initialize time keeper

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting node...");

    mpLastImageTime = std::chrono::steady_clock::now();

    mpLastCamInfoTime = std::chrono::steady_clock::now();

    //subsription
    mpFlightDataSubsriber = this->create_subscription<tello_msgs::msg::FlightData>(
        "flight_data",
        10,
        std::bind(&TelloControllerNode::FlightDataCallback,this,std::placeholders::_1)
    );

    mpImageRawSubscriber = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw",
        10,
        std::bind(&TelloControllerNode::ImageCallback,this,std::placeholders::_1)
    );

    mpCameraInforSubscriber = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info",
        10,
        std::bind(&TelloControllerNode::CameraInfoCallback, this, std::placeholders::_1)
    );

    mpActionResponseSubscriber = this->create_subscription<std_msgs::msg::String>(
        "tello_response",
        10,
        std::bind(&TelloControllerNode::Response,this, std::placeholders::_1)
    );

    mpTelloStateSubscriber = this->create_subscription<std_msgs::msg::String>(
        "tello_state",
        10,
        std::bind(&TelloControllerNode::TelloStateCallback, this, std::placeholders::_1)
    );

    mpActionClient = this->create_client<tello_msgs::srv::TelloAction>(
        "tello_action"
    );

    mpTeleopLifecycleClient = this->create_client<lifecycle_msgs::srv::ChangeState>(
        "tello_joy/change_state"
    );
    
    changeTeleopState("configure");

    using namespace std::chrono_literals;

    mpFlightChecker = this->create_wall_timer(15s,std::bind(&TelloControllerNode::timer_callback,this));

}

void TelloControllerNode::timer_callback(){
    //pre take off , check status of telemetry and frequency
    
    if(!mpReadyTakeOff){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Preflight Check");
        if (mpImageFreq > 0 && mpCameraInfoFreq > 0) {

            changeTeleopState("activate");
            mpReadyTakeOff = true;

            RCLCPP_INFO(this->get_logger(), "Camera and Image: OK");
            RCLCPP_INFO(this->get_logger(), "Ready To Take off...");

        }else{
            RCLCPP_ERROR(this->get_logger(), "Did not recieved any image...");
        }
    }
}
void TelloControllerNode::FlightDataCallback(const tello_msgs::msg::FlightData::ConstSharedPtr &flightData){

    //check if flight data empty
    //

    if(!flightData){
        RCLCPP_ERROR(this->get_logger(), ("Got empty Flight data"));
        return;
    }

    // TODO
    // more failsafe condition 
    if(flightData->bat<10){
        ActionRequestSender("land");
    }
    //RCLCPP_INFO(this->get_logger(), "flight data recieved");

}


void TelloControllerNode::ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img){
    //check frequency of image
    if(!img){

        RCLCPP_ERROR(this->get_logger(), "Got empty image message");
        return;

    }

        auto current = std::chrono::steady_clock::now();
        
        std::chrono::duration<double> diff = current - mpLastImageTime;

        mpImageFreq = 1.0 / diff.count();

        mpLastImageTime = current;

        //RCLCPP_INFO(this->get_logger(), "Image recieved frequency: %.2f Hz", mpImageFreq);
    
}

void TelloControllerNode::CameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camInfo){

    if(!camInfo){

        RCLCPP_ERROR(this->get_logger(), "Got empty camera info message");
        return;
    }

        auto current = std::chrono::steady_clock::now();

        std::chrono::duration<double> diff = current - mpLastCamInfoTime;

        mpCameraInfoFreq = 1.0 / diff.count();

        mpLastCamInfoTime = current;

        //RCLCPP_INFO(this->get_logger(), "Camera info frequency: %.2f Hz", mpCameraInfoFreq);
}
//this will be used for fail safe request sender
void TelloControllerNode::ActionRequestSender(const std::string &cmd){

    auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();

    request->cmd = cmd;

    mpActionRequest = cmd;

    using ServiceResponseFuture = rclcpp::Client<tello_msgs::srv::TelloAction>::SharedFuture;

    // Create response callback
    auto response_callback = [this, cmd](ServiceResponseFuture future) {
        try {
            auto result = future.get();
            if (result->OK) {
                RCLCPP_INFO(this->get_logger(), 
                    "Successfully sending command %s", cmd.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), 
                    "Failed to send command %s", cmd.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                "Exception while processing command: %s", e.what());
        }
    };

    auto future = mpActionClient->async_send_request(request);

    RCLCPP_INFO(this->get_logger(), "Sent request to send command %s", cmd.c_str());

}

//i think it will not be used as the driver already handle action and responses
void TelloControllerNode::Response(const std_msgs::msg::String::ConstSharedPtr &response){

    if(!response){
        RCLCPP_ERROR(this->get_logger(), "Got empty response message");
        return;
    }
}

void TelloControllerNode::changeTeleopState(const std::string state){

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();

    //manage teleop node state
    if(state == "configure")
        request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
    if(state == "activate")
        request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    if(state == "deactivate")
        request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
    if(state == "clean_up")
        request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;

    using ServiceResponseFuture = rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture;

    // Create response callback
    auto response_callback = [this, state](ServiceResponseFuture future) {
        try {
            auto result = future.get();
            if (result->success) {
                RCLCPP_INFO(this->get_logger(), 
                    "Successfully changed teleop node state to %s", state.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), 
                    "Failed to change teleop node state to %s", state.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                "Exception while processing teleop state change response: %s", e.what());
        }
    };

    mpTeleopLifecycleClient->async_send_request(request, response_callback);
    RCLCPP_INFO(this->get_logger(), "Sent request to change teleop state to %s", state.c_str());

}

void TelloControllerNode::TelloStateCallback(const std_msgs::msg::String::ConstSharedPtr &stateMessage){

    if(!stateMessage){
        RCLCPP_ERROR(this->get_logger(), "invalid state message");
    }
    
    mpTelloFlightState = stateMessage->data;
}