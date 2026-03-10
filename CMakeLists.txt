cmake_minimum_required(VERSION 3.8)
project(ptz_gz_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------------------------------- #
# Find gz-sim and dependencies for Gazebo Garden.

message(STATUS "Compiling against Gazebo Garden")
find_package(gz-cmake3 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

gz_find_package(gz-common5 REQUIRED)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

gz_find_package(gz-plugin2 REQUIRED)

gz_find_package(gz-rendering7 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering7_VERSION_MAJOR})

gz_find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

gz_find_package(gz-transport12 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

gz_find_package(gz-msgs9 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs9_VERSION_MAJOR})

# --------------------------------------------------------------------------- #

# find dependencies
#find_package(RapidJSON REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)
find_package(ptz_action_server_msgs)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs)
find_package(controller_manager_msgs)
find_package(controller_manager REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(position_controllers REQUIRED)
find_package(velocity_controllers REQUIRED)

add_library(CameraZoomPlugin
  SHARED
  src/CameraZoomPlugin.cc
)
target_include_directories(CameraZoomPlugin PRIVATE
  include
)

  target_link_libraries(CameraZoomPlugin PRIVATE
    gz-common5::gz-common5
    gz-plugin2::gz-plugin2
    gz-rendering7::gz-rendering7
    gz-sim7::gz-sim7
  )

# Build fake_axis_camera node (real controller switching)
add_executable(fake_axis_camera_node
  src/fake_axis_camera.cpp
)

# Build fake_axis_camera integration node (fake velocity via position)
add_executable(fake_axis_camera_int_node
  src/fake_axis_camera_int.cpp
)

ament_target_dependencies(fake_axis_camera_node
  rclcpp
  rclcpp_action
  ptz_action_server_msgs
  std_msgs
  sensor_msgs
  controller_manager_msgs
)

ament_target_dependencies(fake_axis_camera_int_node
  rclcpp
  rclcpp_action
  ptz_action_server_msgs
  std_msgs
  sensor_msgs
)

install(TARGETS
  fake_axis_camera_node
  fake_axis_camera_int_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/test_ptz_controllers.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  CameraZoomPlugin
  LIBRARY DESTINATION lib
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
