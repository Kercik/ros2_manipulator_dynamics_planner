cmake_minimum_required(VERSION 3.8)
project(ros2_manipulator_dynamics_planner)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Enable extra compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ament & ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)    # <--- added!
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(octomap REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(urdf REQUIRED)
find_package(fcl REQUIRED)

#
# 1) planning node
#
add_executable(planning src/planning.cpp)
ament_target_dependencies(
  planning
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  sensor_msgs
  octomap_msgs
  tf2_ros
)
install(TARGETS planning DESTINATION lib/${PROJECT_NAME})

#
# Install non-code resources: launch/, config/, urdf/, worlds/
#
install(
  DIRECTORY launch config urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

# (Optional) disable lint/test hooks to avoid long hangs
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
