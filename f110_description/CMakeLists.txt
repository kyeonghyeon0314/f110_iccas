cmake_minimum_required(VERSION 3.8)
project(f110_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)

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

install(PROGRAMS
  scripts/simple_drive_controller.py
  scripts/wheel_odometry_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
