cmake_minimum_required(VERSION 3.8)
project(volume_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
find_package(message_filters REQUIRED)

# Build libELAS from source files
# Use SSE3 instruction set
add_definitions(-msse3) 

# libELAS sources
file(GLOB ELAS_SOURCES lib/libelas/src/*.cpp)
add_library(elas STATIC ${ELAS_SOURCES})

# Include directories for libELAS
target_include_directories(elas PUBLIC
  lib/libelas/include
  lib/libelas/include/libelas
)

# Include directories 
include_directories(
  include
  lib/libelas/include
  lib/libelas/include/libelas
)

# Include headers for opencv in all executables
include_directories(${OpenCV_INCLUDE_DIRS}) 

set(THIS_PACKAGE_INCLUDE_DEPENDS
	rclcpp
	sensor_msgs
	cv_bridge
	image_transport
	std_msgs
  message_filters
)

# Add the executable
add_executable(stereo_matcher 
  src/stereo_matcher.cpp
)

ament_target_dependencies(stereo_matcher
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Link against libelas
target_link_libraries(stereo_matcher
  elas
  ${OpenCV_LIBS}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS stereo_matcher
  elas
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
	INCLUDES DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY lib/libelas/include/libelas/
  DESTINATION include/${PROJECT_NAME}/libelas
)

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

ament_package()