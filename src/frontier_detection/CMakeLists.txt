cmake_minimum_required(VERSION 3.8)
project(frontier_detection)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_BUILD_TYPE Release)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Check C++17 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)

if(COMPILER_SUPPORTS_CXX17)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
  add_definitions(-DCOMPILEDWITHC17)
  message(STATUS "Using flag -std=c++17.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
# TODO create octomap_ros dependency in docker image
# find_package(octomap_ros REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(flann REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  visualization_msgs
  sensor_msgs
  geometry_msgs
  custom_interfaces
  tf2
  tf2_ros
  octomap
  octomap_ros
  octomap_msgs
)

include_directories(
	${ament_INCLUDE_DIRS}
	$<INSTALL_INTERFACE:include>
	${EIGEN3_INCLUDE_DIR}
	include)

add_library(RRT src/rrt/rrt.cpp)
target_link_libraries(RRT
  flann::flann
  ${OCTOMAP_LIBRARIES}
  glog::glog
  Eigen3::Eigen
)

add_executable(global_frontier_detector src/global_detector.cpp)
add_executable(local_frontier_detector src/local_detector.cpp)
add_executable(global_frontier_test test/global_frontier_test.cpp)
add_executable(octomap_publisher test/octomap_publisher.cpp)

target_link_libraries(global_frontier_detector
  RRT
  glog::glog
)
target_link_libraries(local_frontier_detector
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

ament_target_dependencies(global_frontier_detector
  rclcpp
  geometry_msgs
  octomap_msgs
  visualization_msgs
)
ament_target_dependencies(local_frontier_detector
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
  global_frontier_detector
  local_frontier_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS  
  global_frontier_test
  DESTINATION exe
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

add_executable(octomap_node
	src/map.cpp
	src/map_driver.cpp	
)

ament_target_dependencies(octomap_node
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install (DIRECTORY include/ 
	DESTINATION include
)

install(TARGETS octomap_node
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION lib/${PROJECT_NAME}
	INCLUDES DESTINATION include/${PROJECT_NAME}
)

install(TARGETS octomap_node
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION lib/${PROJECT_NAME}
	INCLUDES DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()
