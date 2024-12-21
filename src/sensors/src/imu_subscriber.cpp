#include "sensors/imu_subscriber.hpp"

IMUSubscriber::IMUSubscriber(const std::string &csv_file_path)
    : Node("imu_subscriber"), csv_file_path_(csv_file_path)
{
    // Declare and retrieve calibration offsets
    this->declare_parameter<double>("gyro_x_offset", 0.0);
    this->declare_parameter<double>("gyro_y_offset", 0.0);
    this->declare_parameter<double>("gyro_z_offset", 0.0);
    this->declare_parameter<double>("accel_x_offset", 0.0);
    this->declare_parameter<double>("accel_y_offset", 0.0);
    this->declare_parameter<double>("accel_z_offset", 0.0);

    gyro_x_offset_ = this->get_parameter("gyro_x_offset").as_double();
    gyro_y_offset_ = this->get_parameter("gyro_y_offset").as_double();
    gyro_z_offset_ = this->get_parameter("gyro_z_offset").as_double();
    accel_x_offset_ = this->get_parameter("accel_x_offset").as_double();
    accel_y_offset_ = this->get_parameter("accel_y_offset").as_double();
    accel_z_offset_ = this->get_parameter("accel_z_offset").as_double();

    // Open the CSV file for logging IMU data
    csv_file_.open(csv_file_path_, std::ios::out | std::ios::trunc);
    if (!csv_file_.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", csv_file_path_.c_str());
        return;
    }
    csv_file_ << "timestamp,orientation_x,orientation_y,orientation_z,orientation_w,"
                << "angular_velocity_x,angular_velocity_y,angular_velocity_z,"
                << "linear_acceleration_x,linear_acceleration_y,linear_acceleration_z\n";

    // Subscribe to the IMU data topic
    subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/mpu6050", 10, std::bind(&IMUSubscriber::imu_callback, this, std::placeholders::_1));
}

IMUSubscriber::~IMUSubscriber()
{
    if (csv_file_.is_open())
    {
        csv_file_.close();
    }
}


void IMUSubscriber::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    if (!csv_file_.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "CSV file is not open.");
        return;
    }

    // Log IMU data to the CSV file with offsets applied
    csv_file_ << this->now().seconds() << ","
                << msg->orientation.x << "," << msg->orientation.y << "," << msg->orientation.z << "," << msg->orientation.w << ","
                << (msg->angular_velocity.x - gyro_x_offset_) << "," << (msg->angular_velocity.y - gyro_y_offset_) << "," << (msg->angular_velocity.z - gyro_z_offset_) << ","
                << (msg->linear_acceleration.x - accel_x_offset_) << "," << (msg->linear_acceleration.y - accel_y_offset_) << "," << (msg->linear_acceleration.z - accel_z_offset_) << "\n";

    RCLCPP_INFO(this->get_logger(), "Logged IMU data to CSV file.");
}


int main(int argc, char *argv[])
{
rclcpp::init(argc, argv);

std::string csv_file_path = "imu_data.csv";
if (argc > 1)
{
    csv_file_path = argv[1];
}

auto imu_subscriber_node = std::make_shared<IMUSubscriber>(csv_file_path);
rclcpp::spin(imu_subscriber_node);
rclcpp::shutdown();

return 0;
}
