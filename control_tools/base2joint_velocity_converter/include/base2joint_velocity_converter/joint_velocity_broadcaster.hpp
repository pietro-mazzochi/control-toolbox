// Copyright (c) 2024, ISI SIM
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef BASE2JOINT_VELOCITY_CONVERTER__JOINT_VELOCITY_BROADCASTER_HPP_
#define BASE2JOINT_VELOCITY_CONVERTER__JOINT_VELOCITY_BROADCASTER_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class JointVelocityBroadcaster : public rclcpp::Node
{
public:
  JointVelocityBroadcaster();

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;

  std::string joint_name_ = "";
};

#endif // BASE2JOINT_VELOCITY_CONVERTER__JOINT_VELOCITY_BROADCASTER_HPP_
