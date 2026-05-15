cmake_minimum_required(VERSION 3.8)
project(surface_tracking_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Find Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# --- Build filter implementation into a shared library ---
add_library(${PROJECT_NAME} SHARED
  src/ema_filter.cpp
  src/kalman_filter.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

# --- Include Directories ---
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- Build the ROS 2 Node Executable ---
add_executable(velocity_estimator src/velocity_estimator.cpp)

# Link the node to ROS 2 dependencies
ament_target_dependencies(velocity_estimator
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Link the node to your custom filter library
target_link_libraries(velocity_estimator 
  ${PROJECT_NAME}
  Eigen3::Eigen
)

# --- Install the Executable ---
install(TARGETS ${PROJECT_NAME} velocity_estimator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export the include directories and libraries for the ROS 2 package to use
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
