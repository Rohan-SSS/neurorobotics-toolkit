cmake_minimum_required(VERSION 3.5)
project(sensors)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# adding external dependencies for building
add_subdirectory(ext/LeptonSDK)
add_subdirectory(ext/libuvc)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV 4.4 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# add_subdirectory(${PROJECT_SOURCE_DIR}/ext/nmealib)
# Realsense executables
add_executable(realsense_talker
	src/main/realsense_publisher.cpp
	src/realsense/publishers.cpp
	src/realsense/d455.cpp)
add_executable(debug_realsense_listener
	src/main/subscriber.cpp
	src/realsense/subscribers.cpp)
add_executable(kalibr_realsense_listener
	src/main/realsense_kalibr_subscriber.cpp
	src/realsense/subscribers.cpp)

# Lepton executables
add_executable(lepton_talker
	src/main/lepton_publisher.cpp
	src/lepton/lepton35.cpp)

ament_target_dependencies(realsense_talker
	rclcpp
	std_msgs
	sensor_msgs
	cv_bridge
	message_filters)
ament_target_dependencies(debug_realsense_listener
	rclcpp
	std_msgs
	sensor_msgs
	cv_bridge
	message_filters)
ament_target_dependencies(kalibr_realsense_listener
	rclcpp
	std_msgs
	sensor_msgs
	cv_bridge
	message_filters)
ament_target_dependencies(lepton_talker
	rclcpp
	std_msgs
	sensor_msgs
	cv_bridge
	message_filters)

target_include_directories(realsense_talker
	PUBLIC ${OpenCV_INCLUDE_DIRS} 
	PUBLIC ${PROJECT_SOURCE_DIR}/include
	PUBLIC ${realsense2_INCLUDE_DIR})
target_include_directories(debug_realsense_listener
	PUBLIC ${OpenCV_INCLUDE_DIRS} 
	PUBLCI ${PROJECT_SOURCE_DIR}/include
	PUBLIC ${message_filters_INCLUDE_DIRS})
target_include_directories(kalibr_realsense_listener PUBLIC /ws/SkyNet/Sensor/include
	PUBLIC ${OpenCV_INCLUDE_DIRS}
	PUBLIC ${PROJECT_SOURCE_DIR}/include
	PUBLIC ${message_filters_INCLUDE_DIRS})
target_include_directories(lepton_talker
	PUBLIC ${OpenCV_INCLUDE_DIRS}
	PUBLIC ${PROJECT_SOURCE_DIR}/include
	PUBLIC ${PROJECT_SOURCE_DIR}/ext/LeptonSDK/include
	PUBLIC ${PROJECT_SOURCE_DIR}/ext/libuvc/include
	PUBLIC ${PROJECT_SOURCE_DIR}/ext/libuvc/include/libuvc
	PUBLIC ${PROJECT_SOURCE_DIR}/ext/libuvc/build/libuvc)

target_link_libraries(realsense_talker
	${OpenCV_LIBS}
	${realsense2_LIBRARY})
target_link_libraries(debug_realsense_listener
	${OpenCV_LIBS}
	${realsense2_LIBRARY})
target_link_libraries(kalibr_realsense_listener
	${OpenCV_LIBS}
	${realsense2_LIBRARY})
target_link_libraries(lepton_talker
	${OpenCV_LIBS}
	uvc
	LeptonSDK)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  realsense_talker
  debug_realsense_listener
  kalibr_realsense_listener
  lepton_talker
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
