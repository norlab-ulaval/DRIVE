cmake_minimum_required(VERSION 3.8)
project(drive_ros)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  drive_ros/drive_ros_bridge.py
  drive_ros/point_mass_sim.py
  drive_ros/p_controller.py
  drive_ros/keyboard_teleop.py
  drive_ros/calibration_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package()
