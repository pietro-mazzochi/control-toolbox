// Copyright (c) 2025, ISI SIM
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "gnss_sensor_broadcaster/gnss_sensor_broadcaster.hpp"

#include <memory>
#include <string>
#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcpputils/split.hpp"
#include "rcutils/logging_macros.h"

namespace gnss_sensor_broadcaster
{

GnssSensorBroadcaster::GnssSensorBroadcaster()
: controller_interface::ControllerInterface()
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GnssSensorBroadcaster::on_init()
{
  try {
    // Use parameter_or methods instead of declare_parameter to avoid redeclaration errors
    if (get_node()->has_parameter("sensor_name")) {
      sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
    } else {
      sensor_name_ = get_node()->declare_parameter<std::string>("sensor_name", "gnss_sensor");
    }

    if (get_node()->has_parameter("frame_id")) {
      frame_id_ = get_node()->get_parameter("frame_id").as_string();
    } else {
      frame_id_ = get_node()->declare_parameter<std::string>("frame_id", "gnss_frame");
    }

    // Handle covariance parameters
    std::vector<double> covariance;
    if (get_node()->has_parameter("static_covariance_position")) {
      covariance = get_node()->get_parameter("static_covariance_position").as_double_array();
    } else {
      covariance = get_node()->declare_parameter<std::vector<double>>(
        "static_covariance_position", std::vector<double>(9, 0.0));
    }

    if (covariance.size() != 9) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "'static_covariance_position' deve ter exatamente 9 elementos. Retornando ERRO.");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    std::copy(covariance.begin(), covariance.end(), position_covariance_.begin());

    // Similar approach for velocity covariance
    std::vector<double> vel_covariance;
    if (get_node()->has_parameter("static_covariance_velocity")) {
      vel_covariance = get_node()->get_parameter("static_covariance_velocity").as_double_array();
    } else {
      vel_covariance = get_node()->declare_parameter<std::vector<double>>(
        "static_covariance_velocity", std::vector<double>(36, 0.0));
    }

    if (vel_covariance.size() != 36) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "'static_covariance_velocity' deve ter exatamente 36 elementos. Retornando ERRO.");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    std::copy(vel_covariance.begin(), vel_covariance.end(), velocity_covariance_.begin());

    // Define the state interfaces based on the sensor name
    state_interface_names_.push_back(sensor_name_ + "/latitude");
    state_interface_names_.push_back(sensor_name_ + "/longitude");
    state_interface_names_.push_back(sensor_name_ + "/altitude");
    state_interface_names_.push_back(sensor_name_ + "/velocity.north");
    state_interface_names_.push_back(sensor_name_ + "/velocity.east");
    state_interface_names_.push_back(sensor_name_ + "/velocity.down");
    state_interface_names_.push_back(sensor_name_ + "/fix_type");
    state_interface_names_.push_back(sensor_name_ + "/satellites");
    state_interface_names_.push_back(sensor_name_ + "/hdop");
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exceção durante inicialização: '%s'", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
GnssSensorBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
GnssSensorBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration interface_config;
  interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  interface_config.names = state_interface_names_;
  return interface_config;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GnssSensorBroadcaster::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  navsatfix_publisher_ = std::make_shared<NavSatFixPublisher>(get_node()->create_publisher<sensor_msgs::msg::NavSatFix>(
    "~/gnss", rclcpp::SystemDefaultsQoS()));

  velocity_publisher_ = std::make_shared<TwistStampedPublisher>(get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
    "~/velocity", rclcpp::SystemDefaultsQoS()));

  nmea_publisher_ = std::make_shared<NmeaSentencePublisher>(get_node()->create_publisher<nmea_msgs::msg::Sentence>(
    "~/nmea", rclcpp::SystemDefaultsQoS()));

  RCLCPP_INFO(get_node()->get_logger(), "Configurado com sensor '%s' e frame_id '%s'",
              sensor_name_.c_str(), frame_id_.c_str());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GnssSensorBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "GnssSensorBroadcaster ativado");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GnssSensorBroadcaster::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "GnssSensorBroadcaster desativado");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GnssSensorBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Verificar número correto de interfaces
  if (state_interfaces_.size() != state_interface_names_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                "Número incorreto de interfaces: %zu (esperado: %zu)",
                state_interfaces_.size(), state_interface_names_.size());
    return controller_interface::return_type::ERROR;
  }

  double latitude = state_interfaces_[LATITUDE].get_value();
  double longitude = state_interfaces_[LONGITUDE].get_value();
  double altitude = state_interfaces_[ALTITUDE].get_value();
  double vel_north = state_interfaces_[VELOCITY_NORTH].get_value();
  double vel_east = state_interfaces_[VELOCITY_EAST].get_value();
  double vel_down = state_interfaces_[VELOCITY_DOWN].get_value();
  double fix_type = state_interfaces_[FIX_TYPE].get_value();
  double satellites = state_interfaces_[SATELLITES].get_value();
  double hdop = state_interfaces_[HDOP].get_value();

  if (navsatfix_publisher_ && navsatfix_publisher_->trylock()) {
    auto& navsatfix_msg = navsatfix_publisher_->msg_;
    navsatfix_msg.header.stamp = time;
    navsatfix_msg.header.frame_id = frame_id_;

    navsatfix_msg.latitude = latitude;
    navsatfix_msg.longitude = longitude;
    navsatfix_msg.altitude = altitude;

    // (veja função DetermineFixType em ntrip_util.cpp)
    int fix = static_cast<int>(fix_type);
    if (fix == 0) {
      navsatfix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      RCLCPP_DEBUG(get_node()->get_logger(), "GNSS Status: NO FIX");
    } else if (fix == 1 || fix == 2) {
      navsatfix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      RCLCPP_DEBUG(get_node()->get_logger(), "GNSS Status: STANDARD FIX (2D/3D)");
    } else if (fix == 4) {
      // RTK Fixed tem a melhor precisão
      navsatfix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      RCLCPP_DEBUG(get_node()->get_logger(), "GNSS Status: RTK FIXED - Precisão centimétrica");
    } else if (fix == 5) {
      // RTK Float tem precisão intermediária
      navsatfix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
      RCLCPP_DEBUG(get_node()->get_logger(), "GNSS Status: RTK FLOAT - Precisão decimétrica");
    } else {
      navsatfix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    }

    navsatfix_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
                               sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;

    double position_variance;

    switch (fix) {
      case 4: // RTK Fixed - precisão centimétrica (1cm-4cm)
        position_variance = 0.0001; // ~1cm²
        break;
      case 5: // RTK Float - precisão decimétrica (10-20cm)
        position_variance = 0.01; // ~10cm²
        break;
      case 2: // 3D Fix - precisão métrica
      case 1: // 2D Fix
        if (std::isnan(hdop) || hdop <= 0) {
          position_variance = 4.0; // Valor padrão conservador ~2m²
        } else {
          position_variance = hdop * hdop; // hdop² como estimativa de variância
        }
        break;
      default: // No Fix
        position_variance = 100.0; // Valor alto para indicar baixa confiança
    }

    std::fill(navsatfix_msg.position_covariance.begin(), navsatfix_msg.position_covariance.end(), 0.0);
    navsatfix_msg.position_covariance[0] = position_variance; // xx (longitude)
    navsatfix_msg.position_covariance[4] = position_variance; // yy (latitude)
    navsatfix_msg.position_covariance[8] = position_variance * 2.0; // zz (altitude tem maior incerteza)

    navsatfix_msg.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    navsatfix_publisher_->unlockAndPublish();
  }

  if (velocity_publisher_ && velocity_publisher_->trylock()) {
    auto& velocity_msg = velocity_publisher_->msg_;
    velocity_msg.header.stamp = time;
    velocity_msg.header.frame_id = frame_id_;

    velocity_msg.twist.linear.x = vel_north;  // Norte para X
    velocity_msg.twist.linear.y = vel_east;   // Leste para Y
    velocity_msg.twist.linear.z = -vel_down;  // Baixo para -Z (para ENU)

    velocity_msg.twist.angular.x = 0.0;
    velocity_msg.twist.angular.y = 0.0;
    velocity_msg.twist.angular.z = 0.0;

    velocity_publisher_->unlockAndPublish();
  }

  if (nmea_publisher_ && nmea_publisher_->trylock()) {
    auto& nmea_msg = nmea_publisher_->msg_;
    nmea_msg.header.stamp = time;
    nmea_msg.header.frame_id = frame_id_;

    nmea_msg.sentence = generateGGAMessage(
        latitude, longitude, altitude,
        static_cast<int>(fix_type), static_cast<int>(satellites), hdop,
        time);

    nmea_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

std::string GnssSensorBroadcaster::generateGGAMessage(
  double lat, double lon, double alt,
  int fix_type, int satellites, double hdop,
  const rclcpp::Time& timestamp)
{
  time_t time_now = timestamp.seconds();
  std::tm* utc_time = std::gmtime(&time_now);

  char time_str[16];
  uint32_t nanoseconds = timestamp.nanoseconds() % 1000000000;
  double seconds_decimal = nanoseconds / 1.0e9;
  std::sprintf(time_str, "%02d%02d%05.2f",
              utc_time->tm_hour, utc_time->tm_min,
              utc_time->tm_sec + seconds_decimal);

  // Converter latitude para o formato DDMM.MMMMM
  double lat_abs = std::abs(lat);
  int lat_deg = static_cast<int>(lat_abs);
  double lat_min = (lat_abs - lat_deg) * 60.0;
  char lat_dir = lat >= 0.0 ? 'N' : 'S';

  // Converter longitude para o formato DDDMM.MMMMM
  double lon_abs = std::abs(lon);
  int lon_deg = static_cast<int>(lon_abs);
  double lon_min = (lon_abs - lon_deg) * 60.0;
  char lon_dir = lon >= 0.0 ? 'E' : 'W';

  // Mapear fix_type para qualidade NMEA GGA
  int quality;
  switch (fix_type) {
    case 4: quality = 4; break;  // RTK fixed
    case 5: quality = 5; break;  // RTK float
    case 2: quality = 1; break;  // 3D fix
    default: quality = 0; break; // No fix
  }

  char gga[128];
  std::sprintf(gga, "$GPGGA,%s,%02d%09.6f,%c,%03d%09.6f,%c,%d,%02d,%.1f,%.2f,M,0.0,M,,",
              time_str,
              lat_deg, lat_min, lat_dir,
              lon_deg, lon_min, lon_dir,
              quality,
              satellites,
              hdop,
              alt);

  std::string message(gga);
  unsigned char checksum = calculateChecksum(message);

  char final_msg[150];
  std::sprintf(final_msg, "%s*%02X\r\n", gga, checksum);

  return std::string(final_msg);
}

unsigned char GnssSensorBroadcaster::calculateChecksum(const std::string& message)
{
  unsigned char checksum = 0;

  // O checksum começa após o '$' e termina antes do '*'
  for (size_t i = 1; i < message.length() && message[i] != '*'; i++) {
    checksum ^= message[i];
  }

  return checksum;
}

}  // namespace gnss_sensor_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  gnss_sensor_broadcaster::GnssSensorBroadcaster, controller_interface::ControllerInterface)
