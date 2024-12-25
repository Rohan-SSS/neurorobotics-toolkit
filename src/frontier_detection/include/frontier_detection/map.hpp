#ifndef OCTOMAP_H
#define OCTOMAP_H
#include <rclcpp/rclcpp.hpp>
#include "octomap/octomap.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <mutex>
#include <memory>
#include <functional>
#include <thread>
#include <deque>
#include <geometry_msgs/msg/transform.hpp>
#include <octomap/Pointcloud.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include "custom_interfaces/msg/point_cloud3.hpp"
#include "octomap_ros/conversions.hpp"

using MapMsg = custom_interfaces::msg::PointCloud3;

class OctoMap{
	public:
		OctoMap(rclcpp::Logger logger);
		void UpdateMap(octomap::Pointcloud &cloud, octomap::point3d &pos);
		std::shared_ptr<octomap::OcTree> GetMap();
		void GetMap(std::shared_ptr<octomap::OcTree> globalMap);
	private:
		std::mutex mpMtxPointCloudUpdate;
    	bool ValidateOctree();
		std::shared_ptr<octomap::OcTree> mpGlobalMap;
		rclcpp::Logger mpLogger;

};

class OctoMapNode : public rclcpp::Node{
	public:
		OctoMapNode(std::string nodeName);
		void AddNewPointCloudCallback(const MapMsg::SharedPtr msg);
	private:
		rclcpp::Subscription<MapMsg>::SharedPtr mpPointCloudSubscriber;
		std::shared_ptr<OctoMap> mpGlobalMap;
		rclcpp::TimerBase::SharedPtr mpMapPublisherTimer;
		rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr mpMapPublisher;
		std::deque<MapMsg::SharedPtr> mpMsgQueue;
		std::mutex mpMtxMsgQueue;
		std::thread* mpThrMsgQueueProcess;
		// rclcpp::Publisher<sens>::SharedPtr mpAnnotatedFramePublisher;

		// conversion methods
		void ConvertPointCloud2ToOctomap(const sensor_msgs::msg::PointCloud2 &cloud_msg, octomap::Pointcloud &octomap_cloud);
		void CreatePoint3dFromTransform(const geometry_msgs::msg::Transform& transform, octomap::point3d& point);
		void PublishMap();
		void UpdateMap();

};
#endif
