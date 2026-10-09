cmake_minimum_required(VERSION 3.22)
project(ident_signals)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(prbs_generator_node src/prbs_generator_node.cpp src/prbs_generator.cpp)
ament_target_dependencies(prbs_generator_node rclcpp std_msgs sensor_msgs)

add_executable(stair_generator_node src/stair_generator_node.cpp)
ament_target_dependencies(stair_generator_node rclcpp std_msgs sensor_msgs)

add_executable(step_generator_node src/step_generator_node.cpp)
ament_target_dependencies(step_generator_node rclcpp std_msgs sensor_msgs)

add_executable(temp_generator_node src/temp_generator_node.cpp)
ament_target_dependencies(temp_generator_node rclcpp std_msgs sensor_msgs)

add_executable(multi_step_generator_node src/multi_step_generator_node.cpp)
ament_target_dependencies(multi_step_generator_node rclcpp std_msgs sensor_msgs)

install(TARGETS
  prbs_generator_node stair_generator_node step_generator_node multi_step_generator_node temp_generator_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  step_generator_node
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY include/
  DESTINATION include/)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()