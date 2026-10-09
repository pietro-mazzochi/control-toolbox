// Copyright (c) 2024, ISI SIM
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "imu_mag_sensor_broadcaster/imu_mag_sensor_broadcaster.hpp"

#include <memory>
#include <string>

namespace imu_mag_sensor_broadcaster
{
  controller_interface::CallbackReturn ImuMagSensorBroadcaster::on_init()
  {
    try
    {
      param_listener_ = std::make_shared<ParamListener>(get_node());
      params_ = param_listener_->get_params();
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(
          get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }

    imu_state_interface_names_ = {
        params_.sensor_name + "/orientation.x",
        params_.sensor_name + "/orientation.y",
        params_.sensor_name + "/orientation.z",
        params_.sensor_name + "/orientation.w",
        params_.sensor_name + "/angular_velocity.x",
        params_.sensor_name + "/angular_velocity.y",
        params_.sensor_name + "/angular_velocity.z",
        params_.sensor_name + "/linear_acceleration.x",
        params_.sensor_name + "/linear_acceleration.y",
        params_.sensor_name + "/linear_acceleration.z"};

    magnetometer_state_interface_names_ = {
        params_.magnetometer_sensor_name + "/magnetic_field.x",
        params_.magnetometer_sensor_name + "/magnetic_field.y",
        params_.magnetometer_sensor_name + "/magnetic_field.z"};

    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn ImuMagSensorBroadcaster::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    params_ = param_listener_->get_params();

    try
    {
      // register IMU sensor data publisher
      imu_publisher_ =
          get_node()->create_publisher<sensor_msgs::msg::Imu>("~/imu", rclcpp::SystemDefaultsQoS());

      // register magnetometer sensor data publisher
      magnetometer_publisher_ =
          get_node()->create_publisher<sensor_msgs::msg::MagneticField>("~/magnetic_field", rclcpp::SystemDefaultsQoS());
    }
    catch (const std::exception &e)
    {
      fprintf(
          stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
          e.what());
      return CallbackReturn::ERROR;
    }

    RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration ImuMagSensorBroadcaster::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration ImuMagSensorBroadcaster::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names.insert(state_interfaces_config.names.end(),
                                         imu_state_interface_names_.begin(),
                                         imu_state_interface_names_.end());
    state_interfaces_config.names.insert(state_interfaces_config.names.end(),
                                         magnetometer_state_interface_names_.begin(),
                                         magnetometer_state_interface_names_.end());
    return state_interfaces_config;
  }

  controller_interface::CallbackReturn ImuMagSensorBroadcaster::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn ImuMagSensorBroadcaster::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type ImuMagSensorBroadcaster::update(
      const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
  {
    auto imu_message = sensor_msgs::msg::Imu();
    imu_message.header.stamp = time;
    imu_message.header.frame_id = params_.frame_id;

    imu_message.orientation.x = state_interfaces_[0].get_value();
    imu_message.orientation.y = state_interfaces_[1].get_value();
    imu_message.orientation.z = state_interfaces_[2].get_value();
    imu_message.orientation.w = state_interfaces_[3].get_value();
    imu_message.angular_velocity.x = state_interfaces_[4].get_value();
    imu_message.angular_velocity.y = state_interfaces_[5].get_value();
    imu_message.angular_velocity.z = state_interfaces_[6].get_value();
    imu_message.linear_acceleration.x = state_interfaces_[7].get_value();
    imu_message.linear_acceleration.y = state_interfaces_[8].get_value();
    imu_message.linear_acceleration.z = state_interfaces_[9].get_value();

    imu_publisher_->publish(imu_message);

    auto magnetometer_message = sensor_msgs::msg::MagneticField();
    magnetometer_message.header.stamp = time;
    magnetometer_message.header.frame_id = params_.magnetometer_frame_id;

    magnetometer_message.magnetic_field.x = state_interfaces_[10].get_value();
    magnetometer_message.magnetic_field.y = state_interfaces_[11].get_value();
    magnetometer_message.magnetic_field.z = state_interfaces_[12].get_value();

    magnetometer_publisher_->publish(magnetometer_message);

    return controller_interface::return_type::OK;
  }

} // namespace imu_mag_sensor_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    imu_mag_sensor_broadcaster::ImuMagSensorBroadcaster, controller_interface::ControllerInterface)
