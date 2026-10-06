cmake_minimum_required(VERSION 3.8)
project(frontier_perception)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_library(frontier_perception STATIC
  src/perception_utils.cpp
  src/point_cloud_processor.cpp
)

target_include_directories(frontier_perception PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(frontier_perception
  rclcpp
  sensor_msgs
  pcl_conversions
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

target_include_directories(frontier_perception PUBLIC ${PCL_INCLUDE_DIRS})

target_link_libraries(frontier_perception Eigen3::Eigen ${PCL_LIBRARIES})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS frontier_perception
  EXPORT frontier_perceptionTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(frontier_perceptionTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  PCL
  tf2
  tf2_ros
  tf2_sensor_msgs
)
ament_package()
