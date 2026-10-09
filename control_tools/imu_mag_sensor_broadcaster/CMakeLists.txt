cmake_minimum_required(VERSION 3.16)
project(imu_mag_sensor_broadcaster)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format -Werror=range-loop-construct
                      -Werror=missing-braces)
endif()

# using this instead of visibility macros
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  generate_parameter_library
  realtime_tools
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(imu_mag_sensor_broadcaster_parameters
  src/imu_mag_sensor_broadcaster_parameters.yaml
)

add_library(imu_mag_sensor_broadcaster SHARED
  src/imu_mag_sensor_broadcaster.cpp
)
target_compile_features(imu_mag_sensor_broadcaster PUBLIC cxx_std_17)
target_include_directories(imu_mag_sensor_broadcaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/imu_mag_sensor_broadcaster>
)
target_link_libraries(imu_mag_sensor_broadcaster PUBLIC
  imu_mag_sensor_broadcaster_parameters
)
ament_target_dependencies(imu_mag_sensor_broadcaster PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(
  controller_interface imu_mag_sensor_broadcaster.xml)

install(
  DIRECTORY include/
  DESTINATION include/imu_mag_sensor_broadcaster
)
install(
  TARGETS
    imu_mag_sensor_broadcaster
    imu_mag_sensor_broadcaster_parameters
  EXPORT export_imu_mag_sensor_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_imu_mag_sensor_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
