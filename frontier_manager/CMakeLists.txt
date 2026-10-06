cmake_minimum_required(VERSION 3.8)
project(frontier_manager)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem iostreams serialization)
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")
find_package(environment_map REQUIRED)
find_package(frontier_perception REQUIRED)
find_package(path_searching REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(frontier_manager STATIC
  src/frontiers.cpp
  src/viewpoint_graph.cpp
)

# The public headers are consumed by the simulation package.
target_include_directories(frontier_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(frontier_manager Eigen3::Eigen)
ament_target_dependencies(frontier_manager
  environment_map
  frontier_perception
  path_searching
  rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS frontier_manager
  EXPORT frontier_managerTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(frontier_managerTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 frontier_perception environment_map path_searching)
ament_package()