cmake_minimum_required(VERSION 3.8)
project(stereo_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

# Check for libcamera
pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)

# Set include directories
include_directories(
  include
  ${LIBCAMERA_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Create the LCCV library
add_library(lccv_core
  src/lccv/lccv.cpp
  src/lccv/libcamera_app.cpp
  src/lccv/libcamera_app_options.cpp
)

target_include_directories(lccv_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBCAMERA_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(lccv_core PUBLIC
  PkgConfig::LIBCAMERA
  ${OpenCV_LIBS}
  pthread
)

# Create the ROS node executable
add_executable(stereo_node 
  src/main.cpp
  src/stereo_node.cpp
  src/icm20948.cpp
)

ament_target_dependencies(stereo_node
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  image_transport
  camera_info_manager
)

target_link_libraries(stereo_node
  lccv_core
)

# Add mag_viz_node executable
add_executable(mag_viz_node src/mag_viz_node.cpp)

ament_target_dependencies(mag_viz_node
  rclcpp
  sensor_msgs
  visualization_msgs
)

# Install targets
install(TARGETS
  stereo_node
  mag_viz_node
  lccv_core
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch and config files
install(DIRECTORY
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(lccv_core)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  cv_bridge
  image_transport
  camera_info_manager
  tf2
  tf2_ros
)

ament_package()

