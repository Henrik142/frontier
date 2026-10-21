cmake_minimum_required(VERSION 3.8)
project(frontier_utils)

find_package(ament_cmake REQUIRED)
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)

find_package(environment_map REQUIRED)
find_package(frontier_manager REQUIRED)
find_package(frontier_perception REQUIRED)

find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_library(frontier_utils STATIC
  src/visualization_utils.cpp
)

target_include_directories(frontier_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(frontier_utils
  environment_map
  frontier_perception
  frontier_manager
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
)

target_include_directories(frontier_utils PUBLIC ${PCL_INCLUDE_DIRS})

target_link_libraries(frontier_utils Eigen3::Eigen ${PCL_LIBRARIES})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS frontier_utils
  EXPORT frontier_utilsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(frontier_utilsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  environment_map
  frontier_perception
  frontier_manager
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
)
ament_package()
