cmake_minimum_required(VERSION 3.8)
project(drive_ros)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  drive_ros/drive_ros_bridge.py
  drive_ros/point_mass_sim.py
  drive_ros/p_controller.py
  drive_ros/keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
