cmake_minimum_required(VERSION 3.8)
project(f110_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF directory
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install config directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install world files
install(FILES
  iccas_f110.world
  DESTINATION share/${PROJECT_NAME}
)

# Build F1TENTH Drive Plugin
add_library(f1tenth_drive_plugin SHARED
  plugins/f1tenth_drive_plugin.cpp
)

target_include_directories(f1tenth_drive_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(f1tenth_drive_plugin
  gazebo_dev
  gazebo_ros
  rclcpp
  ackermann_msgs
)

# Install plugin
install(TARGETS f1tenth_drive_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()