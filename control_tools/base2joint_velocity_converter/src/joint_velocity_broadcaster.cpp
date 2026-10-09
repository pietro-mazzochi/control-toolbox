// Copyright (c) 2024, ISI SIM
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "base2joint_velocity_converter/joint_velocity_broadcaster.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

JointVelocityBroadcaster::JointVelocityBroadcaster()
    : Node("joint_velocity_broadcaster")
{
  std::string joint_state_topic_;
  std::string velocity_broadcast_topic_;

  this->declare_parameter<std::string>("joint_name", "");
  this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
  this->declare_parameter<std::string>("velocity_broadcast_topic",
                                       "/joint/velocity");

  this->get_parameter("joint_name", joint_name_);
  this->get_parameter("joint_state_topic", joint_state_topic_);
  this->get_parameter("velocity_broadcast_topic", velocity_broadcast_topic_);

  state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, 1,
      std::bind(&JointVelocityBroadcaster::joint_state_callback, this,
                std::placeholders::_1));
  velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      velocity_broadcast_topic_, 1);
}

void JointVelocityBroadcaster::joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{

  for (unsigned int joint_index = 0; joint_index < msg->name.size();
       joint_index++)
  {
    if (msg->name[joint_index] == joint_name_)
    {
      double velocity = msg->velocity[joint_index];
      auto velocity_msg = std_msgs::msg::Float64MultiArray();
      velocity_msg.data.push_back(velocity);
      velocity_pub_->publish(velocity_msg);
      return;
    }
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointVelocityBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
