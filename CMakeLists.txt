cmake_minimum_required(VERSION 3.8)
project(stereo_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

# Check for libcamera and libevent
pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)
pkg_check_modules(LIBEVENT REQUIRED IMPORTED_TARGET libevent_pthreads)

# Create the executable
add_executable(stereo_cam 
  src/stereo.cpp
  src/loop.cpp
)

# Include directories
target_include_directories(stereo_cam PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBCAMERA_INCLUDE_DIRS}
  ${LIBEVENT_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(stereo_cam
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBS}
  PkgConfig::LIBCAMERA
  PkgConfig::LIBEVENT
)

# Set C++ standard
target_compile_features(stereo_cam PUBLIC c_std_99 cxx_std_17)

# Install targets
install(TARGETS stereo_cam
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if you have any
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "*.launch.*"
)

ament_package()

