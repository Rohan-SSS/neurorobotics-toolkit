cmake_minimum_required(VERSION 3.5)
project(sensors)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Yaml-cpp REQUIRED)
find_package(OpenCV 3.0 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

# add_subdirectory(${PROJECT_SOURCE_DIR}/ext/nmealib)

add_executable(talker src/publisher_member_function.cpp src/realsense.cpp src/gps.cpp src/TelemetryValidator.cpp)
add_executable(listener src/subscriber_member_function.cpp src/subscribers.cpp)

ament_target_dependencies(talker rclcpp std_msgs sensor_msgs cv_bridge message_filters)
ament_target_dependencies(listener rclcpp std_msgs sensor_msgs cv_bridge message_filters)

target_include_directories(talker PUBLIC /ws/SkyNet/Sensor/include
	PUBLIC ${OpenCV_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include PUBLIC ${realsense2_INCLUDE_DIR})
target_include_directories(listener PUBLIC /ws/SkyNet/Sensor/include
	PUBLIC ${OpenCV_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(talker
	${PROJECT_SOURCE_DIR}/ext/libSensor.so yaml-cpp
	${OpenCV_LIBS} ${realsense2_LIBRARY})
target_link_libraries(listener ${PROJECT_SOURCE_DIR}/ext/libSensor.so yaml-cpp ${OpenCV_LIBS} ${realsense2_LIBRARY})

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
  talker
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
