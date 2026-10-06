cmake_minimum_required(VERSION 3.8)
project(environment_map)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")
find_package(PCL REQUIRED)

add_library(environment_map STATIC
  src/map.cpp
  src/raycast.cpp
  src/map_builder.cpp
)

target_include_directories(environment_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/include/pcl-1.14
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(environment_map Eigen3::Eigen ${PCL_LIBRARIES})

ament_target_dependencies(environment_map
  pcl_conversions
  Eigen3
  Boost
  PCL
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS environment_map
  EXPORT environment_mapTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(environment_mapTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 Boost PCL pcl_conversions)
ament_package()
