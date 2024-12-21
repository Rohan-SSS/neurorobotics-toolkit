#include <memory>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class IMUSubscriber : public rclcpp::Node
{
public:
    IMUSubscriber(const std::string &csv_file_path);
        
    
    ~IMUSubscriber();
    

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    std::ofstream csv_file_;
    std::string csv_file_path_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;

    // Calibration offsets
    double gyro_x_offset_;
    double gyro_y_offset_;
    double gyro_z_offset_;
    double accel_x_offset_;
    double accel_y_offset_;
    double accel_z_offset_;
};

