cmake_minimum_required(VERSION 3.8)
project(lkh_tsp_solver)

find_package(ament_cmake REQUIRED)

file(GLOB SRCS src/*.c)
list(FILTER SRCS EXCLUDE REGEX "/LKHmain\\.c$")

add_library(lkh_tsp_solver STATIC
  ${SRCS}
  src/lkh_interface.cpp
)

add_executable(lkh_tsp
  src/LKHmain.c
)

add_executable(lkh_tsp_test
  src/test_lkh_tsp.cpp
)

target_compile_definitions(lkh_tsp PRIVATE
  LKH_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
)

target_compile_definitions(lkh_tsp_test PRIVATE
  LKH_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
)

target_include_directories(lkh_tsp_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# LKH.h defines its globals as tentative definitions, which requires -fcommon on GCC >= 10
target_compile_options(lkh_tsp_solver PRIVATE $<$<COMPILE_LANGUAGE:C>:-fcommon>)
target_compile_options(lkh_tsp PRIVATE -fcommon)

target_link_libraries(lkh_tsp_solver PUBLIC m)

target_link_libraries(lkh_tsp lkh_tsp_solver)
target_link_libraries(lkh_tsp_test lkh_tsp_solver)

install(
  DIRECTORY include/lkh_tsp_solver
  DESTINATION include
)

install(
  TARGETS lkh_tsp_solver
  EXPORT lkh_tsp_solverTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS lkh_tsp lkh_tsp_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(lkh_tsp_solverTargets HAS_LIBRARY_TARGET)
ament_package()