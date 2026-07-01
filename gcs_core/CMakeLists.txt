cmake_minimum_required(VERSION 3.8)
project(gcs_core)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp_vendor REQUIRED)
find_package(osqp REQUIRED)
find_package(Qhull REQUIRED)
find_package(Threads REQUIRED)

add_library(gcs_core STATIC
  gcs_pipeline.cpp
  gcs_pipeline_planners.cpp
  corridor_builder.cpp
  region_viz.cpp)
set_target_properties(gcs_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(gcs_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<INSTALL_INTERFACE:include>")

target_link_libraries(gcs_core
  Eigen3::Eigen
  osqp::osqp
  Qhull::qhullcpp
  Qhull::qhullstatic_r
  Threads::Threads)

install(TARGETS gcs_core EXPORT export_gcs_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(FILES gcs_pipeline.hpp corridor_builder.hpp region_viz.hpp DESTINATION include)

ament_export_targets(export_gcs_core HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3 osqp_vendor osqp Qhull)
ament_package()