// Copyright (c) 2024, ISI SIM
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "base2joint_velocity_converter/base2joint_velocity_converter.hpp"

Base2JointVelocityConverterNode::Base2JointVelocityConverterNode()
    : Node("base2joint_velocity_converter")
{
    this->declare_parameter<double>("wheel_to_robot_center", 0.5);
    this->declare_parameter<double>("wheel_radius", 0.5);
    this->declare_parameter<std::string>("base_vel_topic", "/base_vel_cmd");
    this->declare_parameter<std::string>("left_wheel_vel_topic", "/left_wheel_pid/setpoint");
    this->declare_parameter<std::string>("right_wheel_vel_topic", "/right_wheel_pid/setpoint");

    std::string base_vel_topic_;
    std::string left_wheel_vel_topic_;
    std::string right_wheel_vel_topic_;

    this->get_parameter("wheel_to_robot_center", wheel_to_robot_center_);
    this->get_parameter("wheel_radius", wheel_radius_);
    this->get_parameter("base_vel_topic", base_vel_topic_);
    this->get_parameter("left_wheel_vel_topic", left_wheel_vel_topic_);
    this->get_parameter("right_wheel_vel_topic", right_wheel_vel_topic_);

    vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(base_vel_topic_, 1, std::bind(&Base2JointVelocityConverterNode::velocity_callback, this, std::placeholders::_1));

    left_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(left_wheel_vel_topic_, 1);
    right_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(right_wheel_vel_topic_, 1);
}

void Base2JointVelocityConverterNode::velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    double v = msg->linear.x;
    double omega = msg->angular.z;

    double left_wheel_velocity = (v - (omega * wheel_to_robot_center_)) / wheel_radius_;
    double right_wheel_velocity = (v + (omega * wheel_to_robot_center_)) / wheel_radius_;

    auto left_wheel_msg = std_msgs::msg::Float64MultiArray();
    auto right_wheel_msg = std_msgs::msg::Float64MultiArray();

    left_wheel_msg.data.push_back(left_wheel_velocity);
    right_wheel_msg.data.push_back(right_wheel_velocity);

    left_wheel_pub_->publish(left_wheel_msg);
    right_wheel_pub_->publish(right_wheel_msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Base2JointVelocityConverterNode>());
    rclcpp::shutdown();
    return 0;
}
