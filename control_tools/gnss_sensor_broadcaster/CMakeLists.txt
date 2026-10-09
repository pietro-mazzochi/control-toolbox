cmake_minimum_required(VERSION 3.8)
project(gnss_sensor_broadcaster)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcpputils REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)

# GNSS sensor broadcaster library
add_library(gnss_sensor_broadcaster SHARED
  src/gnss_sensor_broadcaster.cpp
)

target_include_directories(gnss_sensor_broadcaster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gnss_sensor_broadcaster
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  sensor_msgs
  geometry_msgs
  nav_msgs
  nmea_msgs
)

# Prevent pluginlib from using boost
target_compile_definitions(gnss_sensor_broadcaster PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(controller_interface gnss_sensor_broadcaster.xml)

# Install XML file
install(
  FILES gnss_sensor_broadcaster.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export hardware plugin
install(
  TARGETS gnss_sensor_broadcaster
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  sensor_msgs
  geometry_msgs
  nav_msgs
  nmea_msgs
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
