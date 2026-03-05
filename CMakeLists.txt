cmake_minimum_required(VERSION 3.8)
project(ptz_gz_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------------------------------- #
# Find gz-sim and dependencies.


# Garden
message(STATUS "Compiling against Gazebo Garden")
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-common4 REQUIRED)
find_package(ignition-rendering6 REQUIRED)
find_package(ignition-gazebo6 REQUIRED)

# --------------------------------------------------------------------------- #

# find dependencies
#find_package(RapidJSON REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)
find_package(ptz_action_server_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs)
find_package(controller_manager_msgs)

add_library(CameraZoomPlugin
  SHARED
  src/CameraZoomPlugin.cc
)
target_include_directories(CameraZoomPlugin PRIVATE
  include
)

  target_link_libraries(CameraZoomPlugin PRIVATE
    ignition-common4::ignition-common4
    ignition-rendering6::ignition-rendering6
    ignition-gazebo6::ignition-gazebo6
  )

# Build fake_axis_camera node
add_executable(fake_axis_camera_node
  src/fake_axis_camera.cpp
)

ament_target_dependencies(fake_axis_camera_node
  rclcpp
  rclcpp_action
  ptz_action_server_msgs
  std_msgs
  sensor_msgs
  controller_manager_msgs
)

install(TARGETS
  fake_axis_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  CameraZoomPlugin
  DESTINATION share/${PROJECT_NAME}/plugins
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


# Install launch files.
install(DIRECTORY
  launch models worlds config urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
