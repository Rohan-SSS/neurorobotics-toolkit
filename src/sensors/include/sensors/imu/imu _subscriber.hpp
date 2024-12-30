#include <memory>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class IMUSubscriber : public rclcpp::Node
{
public:
    IMUSubscriber(const std::string &nodeName);
    ~IMUSubscriber();

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    std::ofstream mpCsvFile;
    std::string mpCsvFilePath;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr mpSubscription;

    // Calibration offsets
    double mpGyroXOffset;
    double mpGyroYOffset;
    double mpGyroZOffset;
    double mpAccelXOffset;
    double mpAccelYOffset;
    double mpAccelZOffset;
};
