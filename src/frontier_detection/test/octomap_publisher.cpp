#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class OctomapPublisher : public rclcpp::Node
{
public:
    OctomapPublisher()
        : Node("octomap_publisher")
    {
        std::string map_path = "src/frontier_detection/frontier_detection/map_resource/fr_079.bt";

        auto octree = std::make_shared<octomap::OcTree>(map_path);

        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("octomap_pointcloud", 10);
        octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("/octomap", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this, octree]()
            { this->publishPointCloudAndOctoMap(octree); });
    }

private:
    void publishPointCloudAndOctoMap(std::shared_ptr<octomap::OcTree> octree)
    {
        // Publish PointCloud
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        for (auto it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it)
        {
            if (octree->isNodeOccupied(*it))
            {
                pcl::PointXYZ point;
                point.x = it.getX();
                point.y = it.getY();
                point.z = it.getZ();
                pcl_cloud.push_back(point);
            }
        }

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(pcl_cloud, cloud_msg);
        cloud_msg.header.frame_id = "map";
        cloud_msg.header.stamp = this->now();
        pointcloud_pub_->publish(cloud_msg);
        RCLCPP_INFO(this->get_logger(), "PointCloud published");

        // Publish Octomap
        auto octomap_msg = std::make_shared<octomap_msgs::msg::Octomap>();
        if (octomap_msgs::fullMapToMsg(*octree, *octomap_msg))
        {
            octomap_msg->header.frame_id = "map";
            octomap_msg->header.stamp = this->now();
            octomap_pub_->publish(*octomap_msg);
            RCLCPP_INFO(this->get_logger(), "Octomap published");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Error converting octomap to message");
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OctomapPublisher>());
    rclcpp::shutdown();
    return 0;
}