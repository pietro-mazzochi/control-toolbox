cmake_minimum_required(VERSION 3.22)
project(base2joint_velocity_converter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp geometry_msgs std_msgs)

add_executable(joint_velocity_broadcaster_node src/joint_velocity_broadcaster.cpp)
ament_target_dependencies(joint_velocity_broadcaster_node rclcpp sensor_msgs std_msgs)

install(TARGETS
  ${PROJECT_NAME}_node
  joint_velocity_broadcaster_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)  
  
# ament_export_dependencies(ament_cmake)  
ament_package()
