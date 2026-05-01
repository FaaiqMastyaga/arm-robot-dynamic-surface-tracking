cmake_minimum_required(VERSION 3.8)
project(surface_tracking_aligner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(aimooe_msgs REQUIRED)
find_package(surface_tracking_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(rigid_body_aligner src/rigid_body_aligner.cpp)

target_include_directories(rigid_body_aligner PUBLIC
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
)

ament_target_dependencies(rigid_body_aligner
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_eigen
  aimooe_msgs
  surface_tracking_interfaces
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/calibration.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  rigid_body_aligner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
