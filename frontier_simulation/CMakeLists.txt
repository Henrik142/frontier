cmake_minimum_required(VERSION 3.8)
project(frontier_simulation)

find_package(ament_cmake REQUIRED)
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)

find_package(environment_map REQUIRED)
find_package(frontier_manager REQUIRED)
find_package(frontier_perception REQUIRED)
find_package(path_searching REQUIRED)
find_package(path_following REQUIRED)

find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


add_library(frontier_simulation STATIC
  src/hippo_visualization.cpp
  src/visualization_utils.cpp
  src/map_updater.cpp
  src/path_planning_manager.cpp
  src/astar_test.cpp
)

add_executable(hippo_visualization
  src/hippo_visualization_node.cpp
)

add_executable(astar_test
  src/astar_test_node.cpp
)

add_executable(map_and_pathplanning
  src/map_and_pathplanning_node.cpp
)

add_executable(path_follower
  src/path_follower_node.cpp
)

add_executable(point_cloud_processor
  src/point_cloud_processor_node.cpp
)

target_include_directories(frontier_simulation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(frontier_simulation
  environment_map
  frontier_perception
  frontier_manager
  path_searching
  path_following
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  sensor_msgs
)

target_include_directories(frontier_simulation PUBLIC ${PCL_INCLUDE_DIRS})

target_link_libraries(frontier_simulation Eigen3::Eigen ${PCL_LIBRARIES})

target_link_libraries(hippo_visualization frontier_simulation)

target_link_libraries(map_and_pathplanning frontier_simulation)

target_link_libraries(astar_test frontier_simulation)

target_link_libraries(point_cloud_processor frontier_simulation)

target_link_libraries(path_follower frontier_simulation)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS frontier_simulation
  EXPORT frontier_simulationTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS hippo_visualization
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS map_and_pathplanning
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS astar_test
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS path_follower
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS point_cloud_processor
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(frontier_simulationTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  environment_map
  frontier_perception
  frontier_manager
  path_searching
  path_following
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  sensor_msgs
)
ament_package()
