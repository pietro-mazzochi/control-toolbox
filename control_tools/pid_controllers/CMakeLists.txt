cmake_minimum_required(VERSION 3.22)
project(pid_controllers)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(${PROJECT_NAME}_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
    src/analog_pid_controller.cpp 
    src/digital_pid_controller.cpp)

add_executable(digital_pid_controller_node src/digital_pid_controller_node.cpp)
target_link_libraries(digital_pid_controller_node ${PROJECT_NAME})
ament_target_dependencies(digital_pid_controller_node ${PROJECT_NAME}_msgs rclcpp std_msgs)

install(TARGETS ${PROJECT_NAME} digital_pid_controller_node
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)

ament_package()



