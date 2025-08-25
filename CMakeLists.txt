cmake_minimum_required(VERSION 3.8)
project(handrobot_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(xacro REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  handrobot_ros2_control/hand_controller_node.py
  handrobot_ros2_control/gesture_command_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and other assets
install(DIRECTORY
  launch
  urdf
  meshes
  rviz
  ros2_control
  DESTINATION share/${PROJECT_NAME}
)

# Resource index
install(FILES
  resource/${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/resource
)

ament_package()
