cmake_minimum_required(VERSION 3.8)
project(frontier_detection)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_BUILD_TYPE Debug)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(glog REQUIRED)
find_package(flann REQUIRED)
find_package(visualization_msgs)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

add_library(RRT src/rrt/rrt.cpp)
target_link_libraries(RRT
  flann::flann
  ${OCTOMAP_LIBRARIES}
  glog::glog
  Eigen3::Eigen
)

add_executable(global_detector src/global_detector.cpp)
add_executable(local_detector src/local_detector.cpp)
add_executable(global_frontier_test test/global_frontier_test.cpp)
add_executable(octomap_publisher test/octomap_publisher.cpp)

target_link_libraries(global_detector
  RRT
  glog::glog
)
target_link_libraries(local_detector
  RRT
  glog::glog
)
target_link_libraries(global_frontier_test
  RRT
  ${OCTOMAP_LIBRARIES}
  glog::glog
  Eigen3::Eigen
)
target_link_libraries(octomap_publisher
  ${OCTOMAP_LIBRARIES}
)
ament_target_dependencies(octomap_publisher
  rclcpp
  octomap_msgs
  sensor_msgs
  PCL
  pcl_conversions
)

ament_target_dependencies(global_detector
  rclcpp
  geometry_msgs
  octomap_msgs
  visualization_msgs
)
ament_target_dependencies(local_detector
  rclcpp
  geometry_msgs
  octomap_msgs
  visualization_msgs
)
install(FILES
  map_resource/fr_079.bt
  DESTINATION share/${PROJECT_NAME}/maps
)

install(TARGETS
  octomap_publisher
  global_detector
  local_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS  
  global_frontier_test
  DESTINATION exe
)

ament_package()
