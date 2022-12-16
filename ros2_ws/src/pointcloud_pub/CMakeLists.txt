cmake_minimum_required(VERSION 3.10.3)
project(pointcloud_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${pcl_conversions_INCLUDE_DIRS})

add_executable(pcd_publisher src/pcd_publisher.cpp)
ament_target_dependencies(pcd_publisher
  rclcpp
  sensor_msgs
  pcl_conversions
  ament_index_cpp
)
target_link_libraries(pcd_publisher ${PCL_LIBRARIES})

install(
TARGETS
    pcd_publisher
DESTINATION lib/${PROJECT_NAME}/)

ament_package()