#include <octomap/octomap.h>

#include <Eigen/Dense>
#include <chrono>
#include <flann/flann.hpp>
#include <memory>
#include <random>
#include <vector>

class RrtFrontierDetector
{
public:
    RrtFrontierDetector(double timeout, double steering_distance);
    bool FindGlobalFrontier(Eigen::Vector3f &frontier_point, octomap::OcTree &octomap);
    bool FindLocalFrontier(
        Eigen::Vector3f &frontier_point, octomap::OcTree &octomap, Eigen::Vector3f current_position);

private:
    bool CheckForTimeout(std::chrono::high_resolution_clock::time_point start_time);
    Eigen::Vector3f SampleSpace(octomap::point3d min_bound, octomap::point3d max_bound);
    Eigen::Vector3f FindNearestNode(
        Eigen::Vector3f point, std::shared_ptr<flann::Index<flann::L2<float>>> &index,
        std::vector<float> &dataset);
    void AddNodeToTree(
        Eigen::Vector3f point, std::shared_ptr<flann::Index<flann::L2<float>>> &index,
        std::vector<float> &dataset);
    Eigen::Vector3f Steer(Eigen::Vector3f point, Eigen::Vector3f random_point);
    bool CheckIfRouteFree(octomap::OcTree &octomap, Eigen::Vector3f point, Eigen::Vector3f new_point);
    bool CheckIfRouteContainsUnknown(
        octomap::OcTree &octomap, Eigen::Vector3f point, Eigen::Vector3f new_point);
    void SetMinMax(octomap::OcTree &octomap);
    double timeout_;
    double steering_distance_;
    std::vector<float> global_tree_points_{0, 0, 0};
    std::shared_ptr<flann::Index<flann::L2<float>>> global_index_;
    int random_seed_{0};
    std::mt19937 generator_;
    std::uniform_real_distribution<double> uniform_dist_;
};