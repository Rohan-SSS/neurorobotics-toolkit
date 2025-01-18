#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/msg/marker.hpp>
#include "frontier_detection/rrt.hpp"

class LocalDetectorNode : public rclcpp::Node
{
public:
	LocalDetectorNode();
	void RobotPositionCallback(const geometry_msgs::msg::Point::SharedPtr msg);
	void DetectFrontiers();
	void MapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
private:
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr mpRobotPositionSubscriber;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr mpMapSubscriber;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr mpPointPublisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mpMarkerPublisher;
    RrtFrontierDetector mpDetector;
    std::shared_ptr<octomap::OcTree> mpGlobalMap;
    Eigen::Vector3f mpRobotPosition;
};
