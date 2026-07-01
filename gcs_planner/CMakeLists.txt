cmake_minimum_required(VERSION 3.8)
project(gcs_planner)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(planner_msgs REQUIRED)
find_package(gcs_core REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)

add_executable(planner_node src/planner_node.cpp src/main.cpp)
target_include_directories(planner_node PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(planner_node gcs_core::gcs_core Eigen3::Eigen ${PCL_LIBRARIES})

ament_target_dependencies(planner_node
  rclcpp tf2_ros sensor_msgs geometry_msgs nav_msgs visualization_msgs
  std_msgs planner_msgs pcl_conversions)

install(TARGETS planner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_package()