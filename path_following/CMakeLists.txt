cmake_minimum_required(VERSION 3.8)
project(path_following)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(hippo_control_msgs REQUIRED)

add_library(path_following STATIC
  src/path_follower.cpp
)

target_include_directories(path_following PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(path_following
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  hippo_control_msgs
)

target_link_libraries(path_following Eigen3::Eigen)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS path_following
  EXPORT path_followingTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(path_followingTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  hippo_control_msgs
)
ament_package()