// Copyright (c) 2025, ISI SIM
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef GNSS_SENSOR_BROADCASTER__GNSS_SENSOR_BROADCASTER_HPP_
#define GNSS_SENSOR_BROADCASTER__GNSS_SENSOR_BROADCASTER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "nmea_msgs/msg/sentence.hpp"

namespace gnss_sensor_broadcaster
{

class GnssSensorBroadcaster : public controller_interface::ControllerInterface
{
public:
  GnssSensorBroadcaster();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  std::vector<std::string> state_interface_names_;

  // Ponteiros para publishers
  using NavSatFixPublisher = realtime_tools::RealtimePublisher<sensor_msgs::msg::NavSatFix>;
  using TwistStampedPublisher = realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>;
  std::shared_ptr<NavSatFixPublisher> navsatfix_publisher_;
  std::shared_ptr<TwistStampedPublisher> velocity_publisher_;

  // Config
  std::string sensor_name_;
  std::string frame_id_;

  // Covariances
  std::array<double, 9> position_covariance_{};  // para NavSatFix
  std::array<double, 36> velocity_covariance_{};  // para TwistStamped

  using NmeaSentencePublisher = realtime_tools::RealtimePublisher<nmea_msgs::msg::Sentence>;
  std::shared_ptr<NmeaSentencePublisher> nmea_publisher_;

  std::string generateGGAMessage(
      double lat, double lon, double alt,
      int fix_type, int satellites, double hdop,
      const rclcpp::Time& timestamp);


  unsigned char calculateChecksum(const std::string& message);


  enum StateInterfaces
  {
    LATITUDE = 0,
    LONGITUDE = 1,
    ALTITUDE = 2,
    VELOCITY_NORTH = 3,
    VELOCITY_EAST = 4,
    VELOCITY_DOWN = 5,
    FIX_TYPE = 6,
    SATELLITES = 7,
    HDOP = 8
  };
};

}  // namespace gnss_sensor_broadcaster

#endif  // GNSS_SENSOR_BROADCASTER__GNSS_SENSOR_BROADCASTER_HPP_
