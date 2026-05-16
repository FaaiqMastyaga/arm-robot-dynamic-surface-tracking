cmake_minimum_required(VERSION 3.8)
project(surface_tracking_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Find Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED) # <-- ADDED: Required for your register macro
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --- Build controller implementation into a shared library ---
add_library(${PROJECT_NAME} SHARED
  src/pid_controller.cpp
  src/pid_ff_controller.cpp
)

# --- Include Directories ---
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- Build the ROS 2 Node Executable ---
add_executable(dynamic_tracker src/dynamic_tracker.cpp)

# Link the node to ROS 2 dependencies
ament_target_dependencies(dynamic_tracker
  rclcpp
  rclcpp_components # <-- ADDED: Link the components library
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# <-- ADDED BLOCK: Link the node to your custom controller library! -->
target_link_libraries(dynamic_tracker 
  ${PROJECT_NAME}
)

# --- Install the Executable ---
install(TARGETS ${PROJECT_NAME} dynamic_tracker
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