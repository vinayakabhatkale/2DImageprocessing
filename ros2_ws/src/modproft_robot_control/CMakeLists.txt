cmake_minimum_required(VERSION 3.5)
project(modproft_robot_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(modproft_robot_control_msgs REQUIRED)

# Include Cpp include directory
include_directories(include)

# Create Cpp executable
#add_executable(moveit_action_server src/moveit_action_server.cpp)
#target_include_directories(
#  moveit_action_server
#  PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#ament_target_dependencies(
#  moveit_action_server
#  rclcpp
#  rclcpp_action
#  rclcpp_components
#  moveit_ros_planning_interface
#  modproft_robot_control_msgs
#  )
  
add_executable(moveit_action_server src/robot_controller.cpp)
target_include_directories(
  moveit_action_server
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  moveit_action_server
  rclcpp
  rclcpp_action
  rclcpp_components
  moveit_ros_planning_interface
  modproft_robot_control_msgs
  )

# Install Cpp executables
install(
  TARGETS
  moveit_action_server
  DESTINATION
  lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(
  PROGRAMS
  scripts/moveit_action_client.py
  DESTINATION
  lib/${PROJECT_NAME}
)

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

ament_package()
