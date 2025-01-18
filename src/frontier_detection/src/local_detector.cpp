#include "frontier_detection/local_detector.hpp"

LocalDetectorNode::LocalDetectorNode() : Node("global_detector"), mpDetector(this->get_logger(), 10, 0.2)
{
	mpRobotPosition = Eigen::Vector3f::Zero();
	mpMapSubscriber = this->create_subscription<octomap_msgs::msg::Octomap>(
			"octomap", 10, std::bind(&LocalDetectorNode::MapCallback, this, std::placeholders::_1));

	mpRobotPositionSubscriber = this->create_subscription<geometry_msgs::msg::Point>(
			"odom", 10, std::bind(&LocalDetectorNode::RobotPositionCallback, this, std::placeholders::_1));

	mpPointPublisher = this->create_publisher<geometry_msgs::msg::Point>("detected_points", 100);
	mpMarkerPublisher = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
	RCLCPP_INFO(this->get_logger(), "Global Detector node initialized");
}

void LocalDetectorNode::RobotPositionCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
	mpRobotPosition.x() = msg->x;
	mpRobotPosition.y() = msg->y;
	mpRobotPosition.z() = msg->z;
}

void LocalDetectorNode::DetectFrontiers()
{
	if (mpRobotPosition.isZero())
	{
		RCLCPP_WARN(this->get_logger(), "Robot position not received yet");
		return;
	}

	Eigen::Vector3f frontier_point;
	if (mpDetector.FindLocalFrontier(frontier_point, *mpGlobalMap, mpRobotPosition))
	{
		geometry_msgs::msg::Point point;
		point.x = frontier_point.x();
		point.y = frontier_point.y();
		point.z = frontier_point.z();
		mpPointPublisher->publish(point);
		RCLCPP_INFO(this->get_logger(), "Local frontier detected at: %f, %f, %f", point.x, point.y, point.z);

		// Publish visualization marker
		visualization_msgs::msg::Marker marker;
		marker.header.frame_id = "map";
		marker.header.stamp = this->now();
		marker.ns = "detected_points";
		marker.id = 0;
		marker.type = visualization_msgs::msg::Marker::SPHERE;
		marker.action = visualization_msgs::msg::Marker::ADD;
		marker.pose.position.x = point.x;
		marker.pose.position.y = point.y;
		marker.pose.position.z = point.z;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.2;
		marker.scale.y = 0.2;
		marker.scale.z = 0.2;
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;

		mpMarkerPublisher->publish(marker);
	}
}

void LocalDetectorNode::MapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
	octomap::AbstractOcTree *tree = octomap_msgs::fullMsgToMap(*msg);
	if (tree)
	{
		mpGlobalMap = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(tree));
		if (mpGlobalMap)
		{
			DetectFrontiers();
		}
	}
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}
