cmake_minimum_required(VERSION 3.8)
project(path_searching)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(environment_map REQUIRED)

add_library(path_searching STATIC
  src/astar.cpp
  src/trajectory_planner.cpp
)

target_include_directories(path_searching PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(path_searching
  rclcpp
  environment_map
)

target_link_libraries(path_searching Eigen3::Eigen)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS path_searching
  EXPORT path_searchingTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(path_searchingTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  environment_map
  rclcpp
)
ament_package()