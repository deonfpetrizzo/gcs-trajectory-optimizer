cmake_minimum_required(VERSION 3.8)
project(traversability_core)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(traversability_core STATIC
  src/trav_types.cpp
  src/robot_model.cpp
  src/voxel_grid.cpp
  src/traversability_map.cpp
  src/occupancy_grid.cpp)
set_target_properties(traversability_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(traversability_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

target_link_libraries(traversability_core
  Eigen3::Eigen)

install(TARGETS traversability_core EXPORT export_traversability_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_traversability_core HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3)
ament_package()
