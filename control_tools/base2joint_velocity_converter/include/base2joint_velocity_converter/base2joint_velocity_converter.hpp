// Copyright (c) 2024, ISI SIM
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef BASE2JOINT_VELOCITY_CONVERTER__BASE2JOINT_VELOCITY_CONVERTER_HPP_
#define BASE2JOINT_VELOCITY_CONVERTER__BASE2JOINT_VELOCITY_CONVERTER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class Base2JointVelocityConverterNode : public rclcpp::Node
{
public:
    Base2JointVelocityConverterNode();

private:
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_wheel_pub_;

    double wheel_to_robot_center_;
    double wheel_radius_;
};

#endif // BASE2JOINT_VELOCITY_CONVERTER__BASE2JOINT_VELOCITY_CONVERTER_HPP_
