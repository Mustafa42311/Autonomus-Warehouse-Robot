cmake_minimum_required(VERSION 3.8)
project(warehouse_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)


find_package(ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/mapping_with_known_poses.py
  ${PROJECT_NAME}/range_to_laserscan_node.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
