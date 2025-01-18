#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/msg/marker.hpp>
#include "frontier_detection/rrt.hpp"

class GlobalDetectorNode : public rclcpp::Node
{
public:
	GlobalDetectorNode();
	void DetectFrontiers();
	void MapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
	
private:
	rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr mpGlobalMapSubscriber;
	rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr mpPointPublisher;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mpMarkerPublisher;
	RrtFrontierDetector mpDetector;
	std::shared_ptr<octomap::OcTree> mpGlobalMap;

};
