#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/msg/marker.hpp>
#include "rrt/rrt.h"

class GlobalDetector : public rclcpp::Node
{
public:
  GlobalDetector() : Node("global_detector"), detector(10, 0.2)
  {
    map_subscriber_ = this->create_subscription<octomap_msgs::msg::Octomap>(
        "octomap", 10, std::bind(&GlobalDetector::MapCallback, this, std::placeholders::_1));

    point_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("detected_points", 100);
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    RCLCPP_INFO(this->get_logger(), "Global Detector node initialized");
  }

  void DetectFrontiers()
  {
    Eigen::Vector3f frontier_point;
    if (detector.FindGlobalFrontier(frontier_point, *octree))
    {
      geometry_msgs::msg::Point point;
      point.x = frontier_point.x();
      point.y = frontier_point.y();
      point.z = frontier_point.z();
      point_publisher_->publish(point);
      RCLCPP_INFO(this->get_logger(), "Frontier detected at: %f, %f, %f", point.x, point.y, point.z);

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

      marker_publisher_->publish(marker);
    }
  }

private:
  void MapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    octomap::AbstractOcTree *tree = octomap_msgs::fullMsgToMap(*msg);
    if (tree)
    {
      octree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(tree));
      if (octree)
      {
        DetectFrontiers();
      }
    }
  }

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr point_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  RrtFrontierDetector detector;
  std::shared_ptr<octomap::OcTree> octree;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
