#include "rrt.h"

#include <glog/logging.h>

using namespace std::chrono;

RrtFrontierDetector::RrtFrontierDetector(double timeout, double steering_distance)
    : timeout_(timeout), steering_distance_(steering_distance), generator_(random_seed_)
{
  flann::Matrix<float> initial(global_tree_points_.data(), 1, 3);
  flann::KDTreeIndexParams indexParams(4);
  indexParams["dynamic_rebuild"] = true;
  global_index_ =
      std::make_shared<flann::Index<flann::L2<float>>>(initial, indexParams, flann::L2<float>());
  global_index_->buildIndex();
}

Eigen::Vector3f RrtFrontierDetector::SampleSpace(
    octomap::point3d min_bound, octomap::point3d max_bound)
{
  std::uniform_real_distribution<double> uniform_dist_(0.0, 1.0);
  return Eigen::Vector3f(
      min_bound.x() + uniform_dist_(generator_) * (max_bound.x() - min_bound.x()),
      min_bound.y() + uniform_dist_(generator_) * (max_bound.y() - min_bound.y()),
      min_bound.z() + uniform_dist_(generator_) * (max_bound.z() - min_bound.z()));
}

bool RrtFrontierDetector::CheckForTimeout(high_resolution_clock::time_point start_time)
{
  return duration_cast<milliseconds>(high_resolution_clock::now() - start_time).count() > timeout_;
}

Eigen::Vector3f RrtFrontierDetector::FindNearestNode(
    Eigen::Vector3f point, std::shared_ptr<flann::Index<flann::L2<float>>> &index,
    std::vector<float> &dataset)
{
  std::vector<float> query = {point.x(), point.y(), point.z()};
  std::vector<int> indices(1);
  std::vector<float> dists(1);

  flann::Matrix<float> queryMatrix(query.data(), 1, 3);
  flann::Matrix<int> indicesMatrix(indices.data(), 1, 1);
  flann::Matrix<float> distsMatrix(dists.data(), 1, 1);
  flann::SearchParams params(128);
  index->knnSearch(queryMatrix, indicesMatrix, distsMatrix, 1, params);

  return Eigen::Vector3f(
      dataset[indices[0] * 3], dataset[indices[0] * 3 + 1], dataset[indices[0] * 3 + 2]);
}

Eigen::Vector3f RrtFrontierDetector::Steer(Eigen::Vector3f point, Eigen::Vector3f random_point)
{
  return point + (random_point - point) * steering_distance_;
}

void RrtFrontierDetector::AddNodeToTree(
    Eigen::Vector3f point, std::shared_ptr<flann::Index<flann::L2<float>>> &index,
    std::vector<float> &dataset)
{
  dataset.push_back(point.x());
  dataset.push_back(point.y());
  dataset.push_back(point.z());
  flann::Matrix<float> new_point(dataset.data(), 1, 3);
  index->addPoints(new_point);
}

bool RrtFrontierDetector::CheckIfRouteContainsUnknown(
    octomap::OcTree &octomap, Eigen::Vector3f point, Eigen::Vector3f new_point)
{
  auto resolution = octomap.getResolution();
  auto vector = new_point - point;
  auto distance = vector.norm();
  auto num_steps = distance / resolution;
  for (int i = 0; i < num_steps; i++)
  {
    auto step = point + vector * (i / num_steps);
    auto octomap_point = octomap::point3d(step.x(), step.y(), step.z());
    if (octomap.search(octomap_point) == nullptr)
      return true;
  }
  return false;
}

bool RrtFrontierDetector::CheckIfRouteFree(
    octomap::OcTree &octomap, Eigen::Vector3f point, Eigen::Vector3f new_point)
{
  auto resolution = octomap.getResolution();
  auto vector = new_point - point;
  auto distance = vector.norm();
  auto num_steps = distance / resolution;
  for (int i = 0; i < num_steps; i++)
  {
    auto step = point + vector * (i / num_steps);
    auto octomap_point = octomap::point3d(step.x(), step.y(), step.z());
    auto node = octomap.search(octomap_point);
    if (node == nullptr || octomap.isNodeOccupied(*node))
      return false;
  }
  return true;
}

void RrtFrontierDetector::SetMinMax(octomap::OcTree &tree)
{
  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float min_z = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::min();
  float max_y = std::numeric_limits<float>::min();
  float max_z = std::numeric_limits<float>::min();
  for (octomap::OcTree::leaf_iterator it = tree.begin_leafs(); it != tree.end_leafs(); ++it)
  {
    if (tree.isNodeOccupied(*it))
    {
      octomap::point3d point = it.getCoordinate();
      min_x = std::min(min_x, point.x());
      min_y = std::min(min_y, point.y());
      min_z = std::min(min_z, point.z());
      max_x = std::max(max_x, point.x());
      max_y = std::max(max_y, point.y());
      max_z = std::max(max_z, point.z());
    }
  }
  auto min_bound = octomap::point3d(min_x, min_y, min_z);
  auto max_bound = octomap::point3d(max_x, max_y, max_z);
  LOG(INFO) << "Min bounds = " << min_bound.x() << " " << min_bound.y() << " " << min_bound.z() << " \n";
  LOG(INFO) << "Max bounds = " << max_bound.x() << " " << max_bound.y() << " " << max_bound.z() << " \n";
  tree.setBBXMin(min_bound);
  tree.setBBXMax(max_bound);
}

bool RrtFrontierDetector::FindGlobalFrontier(
    Eigen::Vector3f &frontier_point, octomap::OcTree &octomap)
{
  SetMinMax(octomap);
  bool frontier_found = false;
  auto start_time = high_resolution_clock::now();
  while (!frontier_found)
  {
    if (CheckForTimeout(start_time))
    {
      LOG(WARNING) << "Timeout reached while searching for global frontier";
      return false;
    }
    Eigen::Vector3f random_point = SampleSpace(octomap.getBBXMin(), octomap.getBBXMax());
    auto nearest_node = FindNearestNode(random_point, global_index_, global_tree_points_);
    auto new_point = Steer(nearest_node, random_point);
    auto is_unknown = CheckIfRouteContainsUnknown(octomap, nearest_node, new_point);
    if (is_unknown)
    {
      frontier_found = true;
      frontier_point = new_point;
    }
    else
    {
      auto route_free = CheckIfRouteFree(octomap, nearest_node, new_point);
      if (route_free)
      {
        AddNodeToTree(new_point, global_index_, global_tree_points_);
        LOG(INFO) << "Added node to global tree, current size: " << global_tree_points_.size() / 3;
      }
    }
  }
  return true;
}

bool RrtFrontierDetector::FindLocalFrontier(
    Eigen::Vector3f &frontier_point, octomap::OcTree &octomap, Eigen::Vector3f current_position)
{
  std::vector<float> local_tree_points{current_position.x(), current_position.y(), current_position.z()};
  flann::Matrix<float> initial(local_tree_points.data(), 1, 3);
  flann::KDTreeIndexParams indexParams(4);
  indexParams["dynamic_rebuild"] = true;
  std::shared_ptr<flann::Index<flann::L2<float>>> local_index =
      std::make_shared<flann::Index<flann::L2<float>>>(initial, indexParams, flann::L2<float>());
  local_index->buildIndex();

  SetMinMax(octomap);
  bool frontier_found = false;
  auto start_time = high_resolution_clock::now();
  while (!frontier_found)
  {
    if (CheckForTimeout(start_time))
    {
      LOG(WARNING) << "Timeout reached while searching for local frontier";
      return false;
    }
    Eigen::Vector3f random_point = SampleSpace(octomap.getBBXMin(), octomap.getBBXMax());
    auto nearest_node = FindNearestNode(random_point, local_index, local_tree_points);
    auto new_point = Steer(nearest_node, random_point);
    auto is_unknown = CheckIfRouteContainsUnknown(octomap, nearest_node, new_point);
    if (is_unknown)
    {
      frontier_found = true;
      frontier_point = new_point;
    }
    else
    {
      auto route_free = CheckIfRouteFree(octomap, nearest_node, new_point);
      if (route_free)
      {
        AddNodeToTree(new_point, global_index_, global_tree_points_);
        LOG(INFO) << "Added node to global tree, current size: " << global_tree_points_.size() / 3;
      }
    }
  }
  return true;
}