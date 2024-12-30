#include <octomap/octomap.h>

#include <Eigen/Dense>
#include <filesystem>
#include <fstream>

#include "../src/rrt/rrt.h"

namespace fs = std::filesystem;

void set_min_max(octomap::OcTree *tree)
{
  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float min_z = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::min();
  float max_y = std::numeric_limits<float>::min();
  float max_z = std::numeric_limits<float>::min();
  for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(); it != tree->end_leafs(); ++it)
  {
    if (tree->isNodeOccupied(*it))
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
  tree->setBBXMin(min_bound);
  tree->setBBXMax(max_bound);
}
int main()
{
  std::string map_path = "src/frontier_detection/frontier_detection/map_resource/fr_079.bt";
  std::cout << "current path: " << fs::current_path() << std::endl;
  octomap::OcTree *tree = new octomap::OcTree(map_path);
  set_min_max(tree);
  std::cout << "tree->calcNumNodes(): " << tree->calcNumNodes() << std::endl;
  std::cout << "tree->getBBXMin(): " << tree->getBBXMin() << std::endl;
  std::cout << "tree->getBBXMax(): " << tree->getBBXMax() << std::endl;
  RrtFrontierDetector rrt_frontier_detector(100, 0.1);

  std::string frontier_points_csv =
      "src/frontier_detection/frontier_detection/map_resource/frontier_points.csv";

  std::ofstream frontier_file;
  if (fs::exists(frontier_points_csv))
  {
    fs::remove(frontier_points_csv);
  }
  frontier_file.open(frontier_points_csv);

  for (int iteration = 0; iteration < 100; ++iteration)
  {
    Eigen::Vector3f frontier_point;
    auto frontier_found = rrt_frontier_detector.FindGlobalFrontier(frontier_point, *tree);
    if (frontier_found)
    {
      frontier_file << frontier_point(0) << "," << frontier_point(1) << "," << frontier_point(2)
                    << std::endl;
    }
  }
  frontier_file.close();
  delete tree;
  return 0;
}
